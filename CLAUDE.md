# CLAUDE.md

Spooder is firmware for a hexapod robot, derived from [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). It runs on a **Pimoroni Servo 2040** (RP2040), written in C++ on the pico-sdk with pimoroni-pico. A Bluetooth gamepad bridge on a TTGO T-Display (ESP32) and I²C sensors are planned. See `openspec/changes/`.

## Toolchain

mise (`mise.toml`) provides Python, PlatformIO (host tests only), CMake, Ninja and the Arm GNU toolchain. It also sets `PICO_SDK_PATH`, `PIMORONI_PICO_PATH` and `PICO_BOARD=pimoroni_servo2040`. The SDKs are pinned git submodules in `external/`: pico-sdk 2.2.0 and pimoroni-pico v1.29.0-2.

```sh
mise install          # toolchains
mise run setup        # fetch the pinned submodules (only pico-sdk's tinyusb sub-submodule)
```

## Commands

```sh
mise run build        # CMake build of firmware/servo2040 -> build/servo2040/spooder.uf2
mise run upload       # copy the .uf2 to a Servo 2040 in BOOTSEL mode (RPI-RP2 drive)
mise run monitor      # USB serial console (115200)
mise run test         # host-side Unity tests of core/ (PlatformIO native env)
mise run clean
```

**Power limits are required.** The firmware does not build until `firmware/servo2040/src/power_limits.h` exists, chosen for the servos and supply fitted (start from `power_limits.example.h`). Alternatively, set `SPOODER_POWER_LIMITS=<file>`. To verify a build without real limits:

```sh
SPOODER_POWER_LIMITS=$PWD/firmware/servo2040/src/power_limits.example.h mise run build
```

`mise run test` and the build above are the verification gates. Hardware behaviour can only be checked on the board (see the `spooder-hardware-check` skill).

## Layout

- `core/`: hardware-free C++ shared by the firmware and the host tests. It includes only `<math.h>`/`<stdint.h>`.
  - `movement/`:
    - `body`: the non-blocking state machine (Initialise, Stand, Walk, Sleep, Calibrate) on a fixed 5 ms internal grid.
    - `gait`, `kinematics` (IK, target validation, `servoOutputIndex`) and `vectors`.
    - `movement_command.h`: the request a controller hands to movement.
  - `protection/`: power protection. It turns supply readings into "output allowed" and "force Sleep", with latched faults; `supervisor` decides when the body runs, stops or restarts.
  - `status/led_pattern`: what the six status LEDs show for a given robot state.
- `firmware/servo2040/`: the pico-sdk CMake project.
  - `main.cpp`: the core 0 loop, every 20 ms: power read → protection → command override → `body::step` → inhibit → servo output.
  - `servo_output`: `ServoCluster` on PIO 0, GPIO0–17.
  - `power_monitor`: current and voltage via the analog mux.
  - `status_leds`: WS2812 on PIO 1, drawing `core/status/led_pattern`.
  - `serial_commands`: the USB bring-up input.
  - `config.h` + `power_limits*.h`: build-time limits.
- `test/`: Unity host tests, one directory per area.
- `openspec/`: specifications (`specs/`) and changes (`changes/`). This is the source of truth for intended behaviour.

## Notes

- Never enable stdio over UART. The SDK's default UART pins, GPIO0/1, are servo outputs 1 and 2.
- `ServoCluster::enable()` on a servo with no pulse set sends it to mid-travel, and `pulse()` enables a disabled servo. `servo_output` relies on both facts. Read it before changing output handling.
- Project skills in `.claude/skills/`:
  - `spooder-firmware-shape`: layout, loop, J6 pins, the protection layer.
  - `spooder-leg-kinematics`: the movement reference.
  - `spooder-hardware-check`: flashing, wiring, power and bring-up.
- A CodeGraph index lives in `.codegraph/` (git-ignored). Refresh it with `codegraph sync` after large changes.
