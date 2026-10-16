# Design

## Context

- The movement core (`body`, `gait`, `kinematics`, `vectors`, `movement_command.h`) is hardware-free C++ with 47 host tests run through PlatformIO's `native` environment. Everything else in `src/` targets the Arduino mbed core on the Nano RP2040 Connect.
- **Servo 2040 facts** (pimoroni-pico `libraries/servo2040/servo2040.hpp`; board schematic):
  - Servo outputs: GPIO0–17.
  - WS2812 LEDs: GPIO18, ×6.
  - I²C0 SDA/SCL on GPIO20/21, which is also UART1 TX/RX. `I2C_INT` is GPIO19.
  - Analog mux address: GPIO22/24/25. The mux output is `SHARED_ADC` on GPIO29. `CURRENT_SENSE` is mux address 7 and `VOLTAGE_SENSE` is address 6.
  - The current shunt is 3 mΩ with gain 69, so about 15.9 A at full scale on a 3.3 V ADC.
  - The only spare GPIOs are 19, 20, 21, 26, 27 and 28, broken out on the unpopulated J6 header together with 3V3, 5V and GND.
- **ServoCluster** (pimoroni-pico `drivers/servo/servo_cluster.hpp`) drives any number of pins from one PIO state machine and one DMA channel, through a shared `DMA_IRQ_0` handler.
  - `pulse(i, us, load=false)` followed by a single `load()` updates all outputs in the same frame.
  - It hard-limits pulses to 400–2600 µs.
- pico-sdk links its optimised ROM/SDK float routines by default (`PICO_DEFAULT_FLOAT_IMPL = pico`), covering `__aeabi_f*`, `sqrtf`, `acosf` and `atan2f`.

## Goals / Non-Goals

**Goals:**
- Build and run the existing movement core unchanged on the Servo 2040.
- Make supply protection a separate, testable layer that sits between commands and movement, and between movement and servo output.
- Keep host tests as the main correctness gate. Add tests for the protection logic.

**Non-Goals:**
- The gamepad bridge, the UART link and telemetry, which belong to `add-gamepad-bridge`. The J6 pin allocation below already reserves GPIO20/21 for the link.
- The I²C sensors, which belong to `add-body-sensing`. GPIO26–28 are reserved for them.
- Multicore use. This change runs on core 0 only.

## Decisions

### D1. Repository layout

```
core/movement/           hardware-free movement core (moved from src/movement, unchanged)
core/protection/          hardware-free power-protection logic (new)
firmware/servo2040/      pico-sdk CMake project
  CMakeLists.txt, pico_sdk_import.cmake, pimoroni_pico_import.cmake
  src/main.cpp           start-up, 20 ms period loop
  src/servo_output.*     ServoCluster adapter (replaces src/movement/servos.*)
  src/power_monitor.*    ADC mux sampling -> protection inputs
  src/status_leds.*      WS2812 mode/fault indication
  src/serial_commands.*  USB stdio bring-up input (ported from control/serial_commands)
  src/config.h           build-time limits (power, pins); #error if unset
external/pico-sdk        git submodule @ 2.2.0
external/pimoroni-pico   git submodule @ v1.29.0-2
test/                    Unity host tests (PlatformIO native env)
```

- **Why submodules:** pimoroni-pico `main` is still changing (`pwm_allocate` was reworked on 2026-10-14). Pinned tags make the build reproducible.
- **Why not pico-boilerplate's "clone alongside" layout:** keeping both libraries inside the repository lets a fresh clone build with `git submodule update --init` and nothing else.
- **Rejected alternative: arduino-pico with a vendored ServoCluster.** It would keep PlatformIO for the firmware, but nobody maintains that port and we would own it.

### D2. Build and tooling

- **mise** adds `cmake`, `ninja` and `gcc-arm-none-eabi`. `PICO_SDK_PATH` and `PIMORONI_PICO_PATH` point at the submodules, and `PICO_BOARD=pimoroni_servo2040`.
- **Tasks:**
  - `mise run build`: configures and builds `firmware/servo2040` into `build/servo2040` and produces a `.uf2`.
  - `mise run upload`: copies the `.uf2` to the RP2040 boot drive, or uses `picotool` if it is installed.
  - `mise run monitor`: opens USB serial.
  - `mise run test`: unchanged; PlatformIO `native`.
- **PlatformIO:** `platformio.ini` keeps only `[env:native]`. Its `build_src_filter` is redirected to `core/movement` and `core/protection` through `src_dir = core`. This keeps the tested Unity setup.
- **Rejected alternative: CMake/ctest for the host tests.** It would mean one toolchain fewer, but rewriting the test harness now adds risk for no behavioural gain.

### D3. Servo output adapter

- `servo_output` owns one `ServoCluster(pio0, 0, servo2040::SERVO_1, 18)`, built and `init()`-ed in `setup` on core 0.
- **Per step:**
  - for every leg the body marked `legUpdated`, write `pulse(output, us, false)` for its three joints;
  - then call `load()` once, so that all changes land in one frame;
  - joint-to-output mapping: output index = 3·leg + joint.
- **Enable/disable:** maps to `enable_all()` / `disable_all()`. A disabled ServoCluster output drives no pulse.
- The adapter keeps an `inhibited` flag, set by protection. While it is set, enabling is refused, including the implicit wake.
- The angle→µs conversion stays in `kinematics::angleToMicroseconds`. `ServoCluster` is used in raw-pulse mode, and its own calibration objects stay at the defaults.

### D4. Power protection as a pure state machine

- `core/protection` has no hardware dependencies. Each tick, `protection::step(state, reading{currentA, voltageV}, nowMs, clearRequested)` returns:
  - `allowOutput`: false while the supply has not yet been valid for 500 ms, or while an output-inhibiting fault is latched;
  - `forceSleep`: true while any fault is latched;
  - the latched fault set, and whether a fault latched on this tick.
- **Trips:**
  - Instant trips (current above `TRIP_CURRENT_A`, voltage below `BROWNOUT_V` or above `MAX_V`) take effect on the tick they are seen.
  - Sustained current uses a time-weighted moving average over `SUSTAINED_WINDOW_MS`.
  - Undervoltage requires the voltage to stay below `MIN_V` for `UNDERVOLT_DEBOUNCE_MS`.
- **Clearing:** a clear succeeds only when the triggering condition is absent on that tick.
- **Wiring in `main`:**

```
reading   = power_monitor::read()          // averaged mux samples
prot      = protection::step(...)
cmd       = input command (serial, later bridge)
if prot.forceSleep: cmd.mode = Sleep        // movement still owns transitions
movementTick(now, cmd)                      // body may want outputs on/off
servo_output.setInhibited(!prot.allowOutput)
servo_output.apply(body state)              // honours inhibit
```

- This keeps the locomotion rule that movement alone changes body mode: protection only rewrites the request. It also keeps output cut-off independent of movement.

### D5. Measurement

- `power_monitor` selects mux address 7 (current) and 6 (voltage) in turn and reads `SHARED_ADC`, using pimoroni-pico `AnalogMux` and `Analog` with `servo2040::CURRENT_GAIN`, `SHUNT_RESISTOR`, `CURRENT_OFFSET` and `VOLTAGE_GAIN`.
- **Sampling:** 8 samples per quantity per tick. ADC reads take about 2 µs, plus a short settle after each mux switch.
- The board's examples average 50 samples over 50 ms. This design trades some of that noise rejection for per-tick evaluation; protection's sustained average provides the remaining filtering.

### D6. Limits configuration

- `firmware/servo2040/src/config.h` defines `TRIP_CURRENT_A`, `SUSTAINED_CURRENT_A`, `SUSTAINED_WINDOW_MS`, `MIN_V`, `UNDERVOLT_DEBOUNCE_MS`, `BROWNOUT_V` and `MAX_V`.
- These start undefined, guarded by `#error "configure power limits for your servos and supply"`. That forces a deliberate choice for the servos and supply actually fitted.
- `config.example.h` gives commented starting points for a regulated 6.0 V servo supply. For example:
  - trip at 9.0 A, under the 10 A terminal rating and well inside the 15.9 A measurable range;
  - 6.0 A sustained over 2 s;
  - a 5.2–6.8 V window, with 4.8 V brown-out.

### D7. Status LEDs

The six WS2812 LEDs (pimoroni `plasma::WS2812` on GPIO18, pio1) show one of:
- waiting for a valid supply: pulsing amber;
- healthy: green, with the mode shown by LED count;
- a latched fault: one colour per fault, flashing red/colour.

They update at no more than 10 Hz.

### D8. Serial bring-up hook

- `serial_commands` is ported to pico stdio over USB (`getchar_timeout_us(0)`).
- It keeps the same keys and adds `!` (clear faults) and `p` (print power readings).
- It is always built in this change, because there is no other input yet. `add-gamepad-bridge` makes it a build option.

### D9. Hardware build notes

- **Where:** the notes go in the `spooder-hardware-check` skill, as part of its rewrite.
- **Servo supply:** into the screw terminals, rated 10 A continuous, reverse-polarity protected, servos up to 11 V.
- **Supplies above 5 V:** cut the "Separate USB and Ext. Power" trace, and power the logic from USB or a separate 5 V BEC on J6 pin 17.
- **Ground:** tie external logic ground to J6 or logic GND, never to the servo-connector ground. The current shunt sits in the servo ground return, so current returned through it is mis-measured.
- **J6 pinout** (schematic sheet 3, unpopulated header; pin → signal): 1 GND, 2 USER_SW, 3 GND, 4 RESET, 5 GND, 6 SWDIO, 7 SWCLK, 8 ADC2 (GPIO28), 9 ADC1 (GPIO27), 10 ADC0 (GPIO26), 11 GND, 12 I2C_INT (GPIO19), 13 I2C_SCL (GPIO21), 14 I2C_SDA (GPIO20), 15 3V3, 16 GND, 17 5V.
- **No pull-ups on the board for GPIO20/21 or GPIO26/27** (the Qw/ST lines show none). The UART RX pin (GPIO21) uses the RP2040's internal pull-up, so an unplugged bridge reads idle-high rather than noise. I²C1 needs pull-ups from the sensor breakouts, or 4.7 kΩ to 3V3.
- **J6 allocation:**
  - GPIO20/21: UART to the bridge;
  - GPIO26/27: I²C1 to the sensors;
  - GPIO28: IMU interrupt;
  - GPIO19: spare.

### D10. Concurrency

- Everything runs on core 0, in the 20 ms period loop. ServoCluster's DMA IRQ is enabled on core 0 because `init()` runs there.
- Nothing is shared across cores yet. `add-body-sensing` may move sensor I/O to core 1 and will define that boundary.

## Risks / Trade-offs

- [The pimoroni-pico v1.29.0-2 tag may not build with pico-sdk 2.2.0; CI on `main` uses 2.2.0] → The first build task verifies the pair. If they don't build together, pin the SDK version that the tag's CI used, and record it.
- [Per-tick current samples are noisier than the 50 ms averaging in the board examples] → The sustained average filters. The trip threshold sits well above normal peaks, and host tests cover debounce and averaging.
- [Total current only: one stalled servo inside a normal total goes undetected] → Documented limitation. Per-servo stall detection would need extra hardware, or inference from the commanded load.
- [The IK and gait geometry is still upstream's V1 geometry] → Out of scope. The new frame's link lengths and offsets are measured during hardware bring-up and changed as constants.
- [Removing the Nano firmware is irreversible in the working tree] → It stays in git history (`f4c07f3` and earlier) and the archived change.

## Migration Plan

1. Land the change, then run `mise run test` and `mise run build`.
2. **Bring-up:** USB only (output must stay off, with the absent supply reported), then a bench supply with current limiting and no servos, then one servo, then a full leg, then all 18, with the robot on a stand.
3. **Rollback:** check out the pre-change commit. The old platform no longer exists physically, so in practice rollback only applies to the code.
