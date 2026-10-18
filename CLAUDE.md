# CLAUDE.md

Spooder is firmware for a hexapod robot, derived from [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). There are two firmwares:
- the robot, on a **Pimoroni Servo 2040** (RP2040), in C++ on the pico-sdk with pimoroni-pico;
- the gamepad bridge, on a **LilyGO TTGO T-Display v1.1** (original ESP32), on ESP-IDF + Arduino + Bluepad32. It sends gamepad commands to the robot over a UART link and shows robot status on its screen.

I²C sensors are planned (`add-body-sensing`, suspended). See `openspec/changes/`.

## Toolchain

mise (`mise.toml`) provides Python, PlatformIO, CMake, Ninja and the Arm GNU toolchain. It also sets `PICO_SDK_PATH`, `PIMORONI_PICO_PATH` and `PICO_BOARD=pimoroni_servo2040`. The SDKs are pinned git submodules in `external/`:
- pico-sdk 2.2.0 and pimoroni-pico v1.29.0-2 (robot);
- the Bluepad32 ESP-IDF + Arduino template (5.0.0-beta0, with its Arduino core and BTstack submodules) and LovyanGFX 1.2.32 (bridge).

The bridge builds with the pioarduino platform's own PlatformIO core in the git-ignored `.pio-core/` (mise's PlatformIO only bootstraps it; the two cores pin different SCons versions). The first `build-bridge` downloads ESP-IDF and the Xtensa toolchain.

```sh
mise install          # toolchains
mise run setup        # fetch the pinned submodules (only the sub-submodules the builds need)
```

## Commands

```sh
mise run build        # CMake build of firmware/servo2040 -> build/servo2040/spooder.uf2
mise run upload       # copy the .uf2 to a Servo 2040 in BOOTSEL mode (RPI-RP2 drive)
mise run monitor      # robot USB serial console (115200)
mise run build-bridge    # PlatformIO build of firmware/bridge (TTGO T-Display)
mise run upload-bridge   # flash the bridge (BRIDGE_PORT, default /dev/ttyUSB0)
mise run monitor-bridge  # the bridge's Bluepad32 console
mise run test         # host-side Unity tests of core/ (PlatformIO native env)
mise run clean
```

**Power limits are required.** The firmware does not build until `firmware/servo2040/src/power_limits.h` exists, chosen for the servos and supply fitted (start from `power_limits.example.h`). Alternatively, set `SPOODER_POWER_LIMITS=<file>`. To verify a build without real limits:

```sh
SPOODER_POWER_LIMITS=$PWD/firmware/servo2040/src/power_limits.example.h mise run build
```

**Command source.** The robot takes commands from the bridge link by default. `mise run build --serial` (or `mise run upload --serial`) builds the USB serial keys instead, for movement bring-up without the bridge and as a rollback.

`mise run test`, `mise run build` (with and without `--serial`) and `mise run build-bridge` are the verification gates. Hardware behaviour can only be checked on the boards (see the `spooder-hardware-check` skill).

## Layout

- `core/`: hardware-free C++ shared by both firmwares and the host tests. It includes only `<math.h>`/`<stdint.h>` (plus `<stdio.h>` for `snprintf` in `status/status_screen`).
  - `movement/`:
    - `body`: the non-blocking state machine (Initialise, Stand, Walk, Sleep, Calibrate) on a fixed 5 ms internal grid.
    - `gait`, `kinematics` (IK, target validation, `servoOutputIndex`) and `vectors`.
    - `movement_command.h`: the request a controller hands to movement.
  - `protection/`: power protection and the emergency stop. It turns supply readings into "output allowed" and "force Sleep", with latched faults; `supervisor` decides when the body runs, stops or restarts.
  - `protocol/`: the link's frames, CRC and incremental decoder (both firmwares), and `link_receiver`, the robot's handling of decoded commands (arrival stamping, clear and mode-request counters, baselines after a link gap).
  - `gamepad_mapping/`: gamepad input to the desired command state (bridge).
  - `status/led_pattern`: what the robot's six status LEDs show; `status/status_screen`: the bridge screen's text.
- `firmware/servo2040/`: the pico-sdk CMake project.
  - `main.cpp`: the core 0 loop, every 20 ms: input (link or serial) → power read → protection → supervisor → `body::step` → servo output → LEDs → telemetry every fifth tick.
  - `link`: UART1 on GPIO20/21 (J6), interrupt-driven RX ring and TX.
  - `servo_output`: `ServoCluster` on PIO 0, GPIO0–17.
  - `power_monitor`: current and voltage via the analog mux.
  - `status_leds`: WS2812 on PIO 1, drawing `core/status/led_pattern`.
  - `serial_commands`: the USB bring-up input (`SPOODER_SERIAL_COMMANDS`).
  - `config.h` + `power_limits*.h`: build-time limits.
- `firmware/bridge/`: the ESP-IDF PlatformIO project for the TTGO T-Display.
  - `main/bridge.cpp`: Bluepad32 callbacks (one controller slot) and `loop()`: gamepad → mapping → link, and the screen.
  - `main/link`: `robot_link`, a FreeRTOS task above `loop()` that sends a command frame every 20 ms, samples the GPIO35 e-stop button and decodes telemetry.
  - `main/display`: LovyanGFX ST7789 driver drawing `status_screen` lines, at most every 250 ms, changed lines only.
- `test/`: Unity host tests, one directory per area.
- `openspec/`: specifications (`specs/`) and changes (`changes/`). This is the source of truth for intended behaviour.

## Notes

- Never enable stdio over UART on the Servo 2040. The SDK's default UART pins, GPIO0/1, are servo outputs 1 and 2. The link uses UART1 on GPIO20/21.
- On ESP-IDF, POSIX `link()` and C++20 `std::lerp` are visible: hence `robot_link` on the bridge and `interpolate` in `core/movement/vectors.h`.
- `ServoCluster::enable()` on a servo with no pulse set sends it to mid-travel, and `pulse()` enables a disabled servo. `servo_output` relies on both facts. Read it before changing output handling.
- Project skills in `.claude/skills/`:
  - `spooder-firmware-shape`: layout, loop, J6 pins, the protection layer, the link and the bridge.
  - `spooder-leg-kinematics`: the movement reference.
  - `spooder-hardware-check`: flashing both boards, wiring, power, bring-up, and the gamepad controls.
- A CodeGraph index lives in `.codegraph/` (git-ignored). Refresh it with `codegraph sync` after large changes.
