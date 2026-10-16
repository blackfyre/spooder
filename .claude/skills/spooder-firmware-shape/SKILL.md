---
name: spooder-firmware-shape
description: Structural rules for the Spooder hexapod firmware — module layout, dependency direction, the two concurrent loops and shared state, the I²C bus and address map, state machines, named constants and error reporting. Use before adding a subsystem or sensor, moving code between modules, introducing shared state between loops, or reviewing a change for structure.
---

# Spooder firmware shape

Adapted from architecture guidance on modular monoliths, explicit state machines, named constants, boundary error contracts and test boundaries. That guidance targets web back ends; only the parts that carry over to a single-binary microcontroller firmware are kept here. Where this skill and the code disagree, the code is the current state and this skill is the direction.

## Modules

One module per hardware capability or behaviour, under `src/<module>/`:

| Module | Owns | Hardware |
|---|---|---|
| `control` | Gamepad connection and input; temporary serial command hook | Bluepad32 over SPI to the NINA-W102; USB serial |
| `movement` | Body modes, gaits, inverse kinematics, servo output (see `spooder-leg-kinematics`) | 2 × PCA9685 |
| `mlc` | IMU activity recognition | LSM6DSOX on X-NUCLEO-IKS01A3 |
| `tof` | Ground distance | VL6180X |
| `display` | OLED output and the on-screen log | SH1107 |
| `reporting` | Serial debug helpers | USB serial |
| `globals` | Cross-cutting hardware handles only (`globalWire`) | — |

Rules:

- A module exposes `setupX()` and `loopX()` plus a small published API in its header. Everything else stays `static` in the `.cpp` (file-local). Today most module globals are non-static; do not add new non-static globals, and make existing ones `static` when you touch them.
- Hardware driver objects (`Adafruit_PWMServoDriver`, `Adafruit_VL6180X`, `LSM6DSOXSensor`, …) are owned by exactly one module. Other modules ask that module; they never call the driver directly.
- Dependencies point from behaviour to capability: `control` may call `movement`'s API; `movement` must not include `control`. No include cycles. `display`/`reporting` are leaf sinks any module may call.
- `main.cpp` only wires things: setup order and which loop calls which `loopX()`. No behaviour in `main.cpp`.
- Do not grow `globals/` into a shared-state bucket. Shared data belongs to the module that produces it, behind an accessor.
- Do not create `utils`/`helpers` catch-alls. Maths that serves movement (`lerp`, Bézier, `mapFloat`) lives in `movement`.

## Two loops, real concurrency

`setup()` ends with `Scheduler.startLoop(loop2)`. On the mbed core this creates an `rtos::Thread` at `osPriorityNormal` with a 4 KiB stack, and RTX round-robin is enabled (5-tick slice). So:

- `loop()` (main thread: MLC, ToF, display) and `loop2()` (gamepad) are **pre-emptively time-sliced**, not cooperative. Either can be interrupted mid-function.
- Any variable written in one loop and read in the other is shared state. Make it explicit: a small struct owned by the producer module, copied out under an `rtos::Mutex` (or a `core_util_critical_section_enter/exit` for a few bytes). Never share multi-field structs such as `Vector2`/`Vector3` without a lock — a torn read gives a leg target from two different frames.
- Values set from an ISR (e.g. `mems_event` in `mlc`) are `volatile`, set-only in the ISR, and consumed in a loop. Keep ISRs to setting a flag.
- `loop2` has a 4 KiB stack. Avoid large locals there (the 80-byte `sprintf` buffers in `control` are fine; arrays of `Vector3[10]` are not). Pass a larger `stackSize` to `startLoop` if needed.
- `delay()` yields to the other thread on mbed; busy-wait loops do not, apart from the time slice. Nothing on the main loop may block: modes and transitions advance per tick (`movementTick`).
- The main loop runs on a 20 ms period scheduler (`LOOP_PERIOD_MS` in `main.cpp`): `movementTick`, then MLC, ToF and display, then sleep for the rest of the period.
- Controller input crosses threads: when the gamepad drives movement, `control` (in `loop2`) publishes a `MovementCommand` snapshot under a lock and `loop()` copies it once per tick. `MovementCommand` is passed by value; movement never reads controller globals.

## Buses

- **SPI**: the NINA-W102 (Bluepad32 and the RGB LED via `BP32.digitalWrite`). Only `control` and `main` setup touch `BP32`.
- **I²C** (`Wire`, exposed as `globalWire` / `DEV_I2C`): shared by everything else. Single transfers are serialised by mbed's I²C mutex; multi-transfer sequences (read range, then read status) are not atomic across loops. Keep every bus user on one loop unless it takes a module-level lock.
- The bus runs at 400 kHz (`Wire.setClock` in `main.cpp`). Budget per 20 ms tick: about 2.5 ms for 18 servo writes (unchanged channels are skipped), under 2 ms for ToF and MLC. A full OLED frame costs about 25 ms, so `loopDisplay` redraws only when the log changed, at most every 200 ms. Do not add per-tick full-frame redraws or per-write register reads.

I²C address map — check for clashes before adding a device:

| Address (7-bit) | Device | Owner |
|---|---|---|
| `0x29` | VL6180X ToF (fixed) | `tof` |
| `0x3C` | SH1107 OLED | `display` |
| `0x40` | PCA9685 #1 — legs 1–3 | `movement` |
| `0x41` | PCA9685 #2 — legs 4–6 | `movement` |
| `0x6A` | LSM6DSOX (`LSM6DSOX_I2C_ADD_L` = `0xD5` 8-bit) | `mlc` |

The IKS01A3 shield carries further sensors (magnetometer, accelerometer, humidity, pressure, temperature) on the same bus; look up their addresses before choosing a new one. Both PCA9685s also answer the all-call address `0x70` by default, so treat it as taken.

## State and constants

- Behaviour that depends on a mode is an explicit state machine: one `enum class` for the states, one function that performs transitions, guards in that function. `body::Mode` with `resolveMode()` in `src/movement/body.cpp` is the model. `control` never sets movement state; it sends a `MovementCommand` whose `mode` is a request.
- Do not add boolean flags that duplicate a state (`isWalking` next to `mode == Mode::Walk`).
- Closed value sets are `enum class` with explicit values when the numbers mean something (MLC output codes 0/1/4/8/12 in `printMLCStatus` should become a named enum). No magic numbers for hardware constants: addresses, channels, pulse limits, link lengths and offsets are named `constexpr` in the owning module.
- Calibration data (per-leg offsets, pulse range) is one table per concept, in one place.

## Failures

- Hardware absence is a handled state, not a crash: follow `tof`'s pattern (`tofSensorAvailable`) — record availability in `setupX()`, make `loopX()` a no-op when absent, and log it to the display.
- Reserve halting loops (`mlc`'s blink-forever on program-load failure) for faults where continuing could damage hardware. Moving servos with a broken IMU is fine; driving servos with no valid leg targets is not.
- Unreachable or invalid targets are rejected before the state they would corrupt is updated.
- Report through `addToLogBuffer` for the operator and `Serial` / `print_value` for debugging. Do not print to serial on hot paths (per-servo writes); it stalls the loop.

## Verification

- Gate for every change: `mise run build` and `mise run test` succeed with no new warnings in `src/`.
- Pure logic is tested off-target: `mise run test` runs the PlatformIO `native` env with Unity (`test/test_*`). Files listed in `[env:native] build_src_filter` must stay free of Arduino and driver headers. Add or update host tests whenever you change that logic.
- Hardware behaviour is verified on the board with the `spooder-hardware-check` skill.
