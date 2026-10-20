# Tasks

## 1. Repository and toolchain

- [x] 1.1 Move `src/movement/{body,gait,kinematics}.{h,cpp}`, `vectors.h` and `movement_command.h` to `core/movement/` unchanged. Point the PlatformIO `native` env at `core/` (`src_dir`, `build_src_filter`), and remove the `nanorp2040connect` env and its `lib_deps`. Verify that `mise run test` passes all existing host tests.
- [x] 1.2 Remove the remaining Nano firmware under `src/` (main, control, display, mlc, tof, reporting, globals, movement `servos`/`movement`). Verify with `git grep` that no Arduino, mbed, Bluepad32 or PCA9685 symbols remain outside the archived OpenSpec change.
- [x] 1.3 Add `external/pico-sdk` (tag 2.2.0) and `external/pimoroni-pico` (tag v1.29.0-2) as git submodules. Add `cmake`, `ninja` and `gcc-arm-none-eabi` to `mise.toml`, with `PICO_SDK_PATH`/`PIMORONI_PICO_PATH` env vars. Verify that `mise install` succeeds and `arm-none-eabi-gcc --version` runs.
- [x] 1.4 Create `firmware/servo2040/` (CMakeLists, the SDK and pimoroni import scripts, `PICO_BOARD=pimoroni_servo2040`) with a minimal `main` that blinks the WS2812 LEDs and prints over USB stdio. Add the `build`/`upload`/`monitor` mise tasks. Verify that `mise run build` produces a `.uf2`. If the pinned pair fails to build, pin the SDK version used by the pimoroni tag's CI and record it in `design.md`.
- [x] 1.5 Compile `core/movement` into the firmware target with `-Wall`. Verify the build has no warnings from `core/`, and that a map-file check (`arm-none-eabi-nm`) shows the pico_float wrappers (`__wrap___aeabi_fmul` or equivalent) linked.

## 2. Power protection core (host-tested)

- [x] 2.1 Create `core/protection/` with the reading/limits/state types and `protection::step` (design D4). Verify that host tests cover the power-on gate (500 ms of continuous valid voltage) and that USB-only power (≈0 V) keeps output disallowed.
- [x] 2.2 Implement the overcurrent trip, the sustained-current average over a window, undervoltage with debounce, the brown-out and overvoltage instant trips, and fault latching. Verify that host tests cover:
  - each trip;
  - a short peak and a transient sag not tripping;
  - trips taking effect on the evaluation tick;
  - `forceSleep`/`allowOutput` for each fault type.
- [x] 2.3 Implement clearing: a clear request succeeds only when the condition is absent, and clearing does not change movement on its own. Verify host tests for clearing while the condition persists, clearing after recovery, and forceSleep staying set until cleared.

## 3. Servo 2040 integration

- [x] 3.1 Add `firmware/servo2040/src/config.h` with `#error` unless the power limits are defined, plus `config.example.h` holding the design's starting values for a 6.0 V supply. Verify that the build fails without a config and succeeds with the example copied in.
- [x] 3.2 Implement `servo_output` over `ServoCluster(pio0, 0, SERVO_1, 18)`:
  - batched `pulse(..., false)` plus a single `load()` per step;
  - output index = 3·leg + joint;
  - `enable_all`/`disable_all`;
  - an `inhibited` flag that blocks the implicit wake.
  
  Verify the build, and that the mapping unit (index function) is host-tested against the spec's leg 5 femur → output 14 scenario.
- [x] 3.3 Implement `power_monitor` with `AnalogMux` and `Analog` (current at address 7, voltage at address 6, 8 samples each per tick). Verify the build. On hardware with USB only, the reported voltage must be near 0 V and the current near 0 A (recorded in 4.2).
- [x] 3.4 Implement `status_leds` (WS2812 on GPIO18, pio1) for the waiting, healthy and per-fault indications, at ≤10 Hz. Verify that the build succeeds and that the fault-to-colour table is defined as constants.
- [x] 3.5 Port `serial_commands` to pico stdio (non-blocking), with the existing keys plus `!` (clear faults) and `p` (print readings). Verify the build. A host test of the key-to-command mapping must pass if the mapping is split into `core/`.
- [x] 3.6 Write `main`:
  - set up the cluster, monitor and LEDs on core 0;
  - run a 20 ms period loop doing read → protection → command override → `movementTick` → inhibit → `servo_output.apply`;
  - report readings and any latched fault once per second, and each fault once when it latches.
  
  Verify that `mise run build` passes with no warnings in `firmware/` or `core/`.

## 4. Verification and documentation

- [x] 4.1 Run `mise run test` and `mise run build` from a clean checkout (`git submodule update --init --recursive`). Verify both pass.
- [ ] 4.2 Hardware bring-up per the rewritten `spooder-hardware-check` (requires the user at the hardware), recording results in this change:
  - USB only: output off, absent supply reported;
  - a current-limited bench supply with no servos: supply accepted after 500 ms, healthy LEDs;
  - one servo, then one leg, measuring a 1500 µs pulse on a scope (±2 %, 20 ms frame);
  - the bench supply's current limit lowered below the trip value: overcurrent latches and clears;
  - all 18 servos with the robot on a stand: boot, Stand, Walk, Sleep via the serial hook.
- [x] 4.3 Rewrite `CLAUDE.md` (platform, toolchain, commands, layout), `spooder-firmware-shape` (Servo 2040 layout, core 0 loop, J6 pin allocation, protection layer) and `spooder-hardware-check` (Servo 2040 flashing, bring-up order, wiring and power notes from design D9). Update `spooder-leg-kinematics` (servo mapping and output adapter). Verify that every symbol and path the docs mention exists (`grep`).
