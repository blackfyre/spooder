# firmware/servo2040/

The robot's pico-sdk CMake project (`mise run build` → `build/servo2040/spooder.uf2`; `mise run upload`). Structure rules: the `spooder-firmware-shape` skill; flashing and bring-up: `spooder-hardware-check`.

- `src/main.cpp`: the core 0 loop, every 20 ms: read both links' bytes (or the serial keys) and the power reading, run `robot_tick::tick` (`core/robot`), which drives the adapters below. It passes the build identity (`build_identity::ID`, from the generated `build_identity_id.cpp` the `spooder_build_identity` target writes in the build directory) to `robot_tick::init`, and `stdio_usb_connected()` as `Inputs::usbConnected`, so the banner reaches a console opened after boot.
- `src/link`: UART1 on GPIO20/21 (J6), interrupt-driven RX ring and a 256-byte TX queue of whole frames.
- `src/usb_link`: bytes over USB stdio, sharing the port with `printf`; frames are written whole or dropped. It carries the 10 Hz pose and diagnostics frames, as `src/link` (UART1) does.
- `src/servo_output`: `ServoCluster` on PIO 0, GPIO0–17.
- `src/power_monitor`: current and voltage via the analog mux.
- `src/status_leds`: WS2812 on PIO 1, drawing `core/status/led_pattern`.
- `src/serial_commands`: the USB bring-up input (`SPOODER_SERIAL_COMMANDS`). On a tick where `robot_tick::bannerDue` is true, `main.cpp` polls it with `readKeys` false, so no key echo precedes the build banner.
- `src/config.h` + `src/power_limits*.h`: build-time limits.

## Build options

- **Power limits are required.** The firmware does not build without `src/power_limits.h`, chosen for the servos and supply fitted (start from `power_limits.example.h`). Alternatively, set `SPOODER_POWER_LIMITS=<file>`. To verify a build without real limits: `SPOODER_POWER_LIMITS=$PWD/firmware/servo2040/src/power_limits.example.h mise run build`.
- **Servo profile.** `SPOODER_SERVO_PROFILE=<name>` (CMake option, also from the environment through `mise run build` and `mise run virtual-robot`) chooses the fitted servo model from `core/movement/servo_profile.h`; an unknown name fails the build. Add a table entry for a new model.
- **Command source.** By default the robot takes commands from the bridge link or the USB link, whichever reports a connected controller; if both do, it sleeps and reports a source conflict. `mise run build --serial` (or `mise run upload --serial`) builds the USB serial keys instead, for movement bring-up without the bridge and as a rollback; the USB link is then not compiled. Both builds are gates.
- **Warnings are errors.** `spooder_core` compiles with `-Wall -Wextra -Werror`; the `spooder` target has `-Wall`, and its own sources get `-Wextra -Werror` as a source-file `COMPILE_OPTIONS` property, since the SDK's interface libraries compile into the same target and their warnings must not fail the build. Fix a warning in the code, never with `-Wno-…`; one raised inside an SDK header is suppressed at that include with a `#pragma GCC diagnostic` push/ignore/pop naming the header.
- **Image budget.** The last step of `mise run build` runs `scripts/check_image_budget.py` on `build/servo2040/spooder.elf`: `arm-none-eabi-objdump -h`, then `scripts/image_budget.py` sums flash use (allocated, loaded sections whose load address is in flash, so `.data` counts) and static RAM use (allocated sections whose run address is in SRAM, including the minimum heap and the stack) against `FLASH_BUDGET` (256 KiB) and `RAM_BUDGET` (64 KiB). It prints a use line for each and fails the build, and so `mise run upload`, when either is exceeded. Raising a budget is a reviewed one-line change, recorded in the changelog. `mise run test-robot-build` (run by `mise run test`) tests the logic with crafted section tables.

## Rules

- Never enable stdio over UART. The SDK's default UART pins, GPIO0/1, are servo outputs 1 and 2. The link uses UART1 on GPIO20/21.
- Debug text on the USB port must stay printable ASCII (plus tab, CR, LF): the same port carries binary link frames.
- `ServoCluster::enable()` on a servo with no pulse set sends it to mid-travel, and `pulse()` enables a disabled servo. `servo_output` relies on both facts. It also turns off each channel's calibration limits (500–2500 µs by default), keeping the cluster's 400–2600 µs hard limits, so calibrated pulses pass unchanged. Read it before changing output handling.
