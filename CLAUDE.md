# CLAUDE.md

Spooder is firmware for a hexapod robot, derived from [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). There are two firmwares:
- the robot, on a **Pimoroni Servo 2040** (RP2040), in C++ on the pico-sdk with pimoroni-pico;
- the gamepad bridge, on a **LilyGO TTGO T-Display v1.1** (original ESP32), on ESP-IDF + Arduino + Bluepad32. It sends gamepad commands to the robot over a UART link and shows robot status on its screen. Its setup mode (left button held during the boot logo) replaces Bluetooth with a WiFi access point and captive portal, through which a phone calibrates the robot.

A computer can speak the same link protocol on the robot's USB port, alongside the debug text, to calibrate, monitor, control and drive (from the keyboard) the robot without the bridge. Joint calibration can be changed live (in RAM) and exported as the compiled defaults. The desktop calibrator (`desktop/`, Go with a Gio UI) is that computer-side front-end.

I²C sensors are planned (`add-body-sensing`, suspended). See `openspec/changes/`.

## Toolchain

mise (`mise.toml`) provides Python, PlatformIO, CMake, Ninja, the Arm GNU toolchain and Go (the desktop calibrator). It also sets `PICO_SDK_PATH`, `PIMORONI_PICO_PATH` and `PICO_BOARD=pimoroni_servo2040`. The SDKs are pinned git submodules in `external/`:
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
mise run monitor      # robot USB console: timestamped text; --frames decodes frames, --keys for a --serial build
mise run monitor-raw  # the raw console (pio device monitor, 115200): frames show as junk
mise run build-bridge    # PlatformIO build of firmware/bridge (TTGO T-Display)
mise run upload-bridge   # flash the bridge (BRIDGE_PORT, default /dev/ttyUSB0)
mise run monitor-bridge  # the bridge's Bluepad32 console
mise run test         # host-side Unity tests of core/ (PlatformIO native env)
mise run desktop-build   # the desktop calibrator -> build/desktop/spooder-calibrator
mise run desktop-run     # build and start it (run from the checkout: it exports there)
mise run desktop-test    # gofmt, go vet and go test of desktop/
mise run desktop-deps    # name the missing Linux packages for the Gio build
mise run desktop-install # the launcher and icon for this user (Linux); desktop-icons re-renders the icon
mise run virtual-robot   # the virtual robot -> build/virtual-robot/virtual-robot
mise run virtual-robot-run  # run it, USB terminal linked at build/virtual-robot/usb
mise run desktop-sim      # both: the virtual robot in this terminal, the calibrator on its port
mise run e2e             # build it, then the desktop end-to-end tests against it
mise run bridge-screenshots  # render the bridge screen's pages to docs/images/bridge-*.png
mise run clean
```

**Desktop calibrator dependencies.** Gio needs cgo and the Wayland, X11 and EGL development packages on Linux. On Fedora: `sudo dnf install wayland-devel libxkbcommon-devel libxkbcommon-x11-devel libX11-devel libXcursor-devel libXfixes-devel mesa-libEGL-devel vulkan-headers`. `desktop-build` and `desktop-test` run `desktop-deps` first, which prints the `dnf` line for whatever is missing. Opening the robot's port needs membership of the `dialout` group.

**Power limits are required.** The firmware does not build until `firmware/servo2040/src/power_limits.h` exists, chosen for the servos and supply fitted (start from `power_limits.example.h`). Alternatively, set `SPOODER_POWER_LIMITS=<file>`. To verify a build without real limits:

```sh
SPOODER_POWER_LIMITS=$PWD/firmware/servo2040/src/power_limits.example.h mise run build
```

**Servo profile.** `SPOODER_SERVO_PROFILE=<name>` (CMake option, also from the environment through `mise run build` and `mise run virtual-robot`) chooses the fitted servo model from `core/movement/servo_profile.h`; an unknown name fails the build. Add a table entry for a new model.

**Command source.** By default the robot takes commands from the bridge link or the USB link, whichever reports a connected controller; if both do, it sleeps and reports a source conflict. `mise run build --serial` (or `mise run upload --serial`) builds the USB serial keys instead, for movement bring-up without the bridge and as a rollback; the USB link is then not compiled.

**Flash both boards together** when the link protocol changes (`protocol::VERSION`, now 3). A mismatched pair fails safe: each side discards the other's frames, so the link looks lost and the robot sleeps.

`mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build` and `mise run e2e` are the verification gates. Hardware behaviour can only be checked on the boards (see the `spooder-hardware-check` skill).

## Layout

- `core/`: hardware-free C++ shared by both firmwares and the host tests. It includes only `<math.h>`/`<stdint.h>` (plus `<stdio.h>` for `snprintf` in `status/status_screen` and `robot/robot_tick`).
  - `movement/`:
    - `body`: the non-blocking state machine (Initialise, Stand, Walk, Sleep, Calibrate, Perform) on a fixed 5 ms internal grid.
    - `gesture`: the gestures Perform plays (say hi and wiggle so far): keyframe scripts of body shifts and lifted feet, and the player (eased, rate-limited segments; a stopped gesture finishes its keyframe, puts its feet down, then straightens).
    - `gait`, `kinematics` (IK, target validation, `servoOutputIndex`) and `vectors`.
    - `joint_calibration`: the per-joint calibration table (offset, 0° and 180° pulses, direction), its clamps, the 60°/s rate-limited `step` and `toPulse`.
    - `calibration_defaults.cpp`: the compiled defaults, in a fixed format that a front-end's export replaces whole. Its values are free to change; host tests check that it parses, stays within the guards and round-trips through the exporters.
    - `movement_command.h`: the request a controller hands to movement.
    - `servo_profile.h`: the servo models (rated speed, share usable under load) and the build's choice, `SPOODER_SERVO_PROFILE` (default `MG996R`), which sets the joint speed budget. `body` slows its motion clock, in 20 ms pieces, to keep every joint within the budget; `robot_tick`'s output rate backstop never exceeds the rated speed.
  - `protection/`: power protection and the emergency stop. It turns supply readings into "output allowed" and "force Sleep", with latched faults; `supervisor` decides when the body runs, stops or restarts.
  - `protocol/`: the link's frames (version 3: command with the gesture request, telemetry with the gesture playing, calibration set and report, and the pose and diagnostics for the USB host), CRC and incremental decoder (both firmwares), pinned by `test/golden/protocol_v3.txt`. `link_receiver` is the robot's handling of decoded frames: arrival stamping, clear, mode-request and gesture-request counters, baselines after a link gap, the calibration guards and reports, and the arbitration between the bridge and USB links.
  - `robot/robot_tick`: the robot's 20 ms control tick, shared by the firmware and the virtual robot: link decoding and arbitration (or the serial keys) → protection → supervisor → calibration sets → calibration step → `body::step` → servo output → LEDs → telemetry every fifth tick, the pose frame on USB two ticks later and the diagnostics frame (motion share, backstop, tick time, limits) four ticks later, and calibration reports, on both links → the status line. It drives the hardware through its `Hardware` interface.
  - `gamepad_mapping/`: gamepad input to the desired command state (bridge); `rumble`, the bridge's rumble feedback (fault, e-stop, link, refused gesture).
  - `status/led_pattern`: what the robot's six status LEDs show; `status/status_screen`: the bridge screen's content (the drive, power and link pages with the status strip, and the setup-mode text), with `status/power_history` (60 s of voltage and current) and `status/page_button` (the left button as the page switch).
  - `setup/`: the bridge's setup mode. `portal_controller` (portal polls and buttons as the controller), `calibration_sync` (pending edits, one set in flight, confirmation by report), `calibration_export` (a reported table as `calibration_defaults.cpp`), `setup_entry` (the button hold and the boot decision) and `access_point` (SSID, password, QR text).
- `firmware/servo2040/`: the pico-sdk CMake project.
  - `main.cpp`: the core 0 loop, every 20 ms: read both links' bytes (or the serial keys) and the power reading, run `robot_tick::tick`, which drives the adapters below.
  - `link`: UART1 on GPIO20/21 (J6), interrupt-driven RX ring and a 256-byte TX queue of whole frames.
  - `usb_link`: bytes over USB stdio, sharing the port with `printf`; frames are written whole or dropped. Only USB carries the 10 Hz pose and diagnostics frames.
  - `servo_output`: `ServoCluster` on PIO 0, GPIO0–17.
  - `power_monitor`: current and voltage via the analog mux.
  - `status_leds`: WS2812 on PIO 1, drawing `core/status/led_pattern`.
  - `serial_commands`: the USB bring-up input (`SPOODER_SERIAL_COMMANDS`).
  - `config.h` + `power_limits*.h`: build-time limits.
- `firmware/bridge/`: the ESP-IDF PlatformIO project for the TTGO T-Display.
  - `main/main.c`: `app_main`; a restart into setup mode skips BTstack and Bluepad32 there.
  - `main/bridge.cpp`: Bluepad32 callbacks (one controller slot) and `loop()`: gamepad → mapping → link, the screen, and the GPIO0 hold into setup mode during the logo.
  - `main/link`: `robot_link`, a FreeRTOS task above `loop()` that sends a command frame every 20 ms, samples the GPIO35 e-stop button and decodes telemetry and calibration reports. In setup mode its commands come from the portal controller, and it sends calibration sets.
  - `main/display`: LovyanGFX ST7789 driver drawing the `status_screen` pages, at most every 250 ms, changed regions only, each through one reusable sprite; `main/screen_draw` draws each region in the Blackfyre dark palette onto any LovyanGFX surface (shared with the screenshot tool); the smooth fonts in `main/fonts/` come from `scripts/make_fonts.py` (`uv run`, by hand); in setup mode, the setup screen with the joining QR code.
  - `main/setup_mode`: the setup-mode boot path (RTC magic), the WiFi access point, the DNS and HTTP servers and the setup task. `main/portal_page.h` is the portal page, one hand-written HTML file.
- `desktop/`: the desktop calibrator, a Go module (`github.com/blackfyre/spooder/desktop`) kept apart so Gio and cgo never reach the firmware builds.
  - `cmd/spooder-calibrator`: `main`, the wiring.
  - `cmd/spooder-monitor` + `internal/console`: the text console behind `mise run monitor`. It uses the link read-only (no source, so no writer) and never sends a frame; only `--keys` writes, printable ASCII only. No Gio, and cgo only on macOS.
  - `internal/protocol`: the link protocol in Go, byte for byte the C++ one, tested against `test/golden/protocol_v3.txt`.
  - `internal/link`: the serial port (`go.bug.st/serial`), the split of the stream into frames and debug text lines, the 20 ms writer (none for a nil source) and reconnecting by USB serial number.
  - `internal/session`: the single state owner: observe or control, the e-stop, mode and clear counters, driving (held keys and walking settings), telemetry, the latest pose, and the ten-minute history with its CSV export.
  - `internal/legview`: the legs pane's geometry: forward kinematics (tested against `test/golden/leg_pose.txt`), leg mounting, servo limits; and the whole robot in the body frame with the top, front, left and isometric projections (`views.go`) for the four-view window (`internal/ui/views.go`, its own Gio window).
  - `internal/calib`: the joint table shown from reports, pending edits, one change in flight, resend and confirmation by request id.
  - `internal/export`: `calibration_defaults.cpp` in the fixed format (tested against the fixture `test/golden/calibration_defaults.cpp`), the checkout check, the diff and the atomic write.
  - `assets/`: the app icon, `icon.svg`, with its rendered PNGs and `icon.ico`.
  - `internal/virtualrobot`: runs the virtual robot for tests; `e2e/`: end-to-end tests of the real link and session against it (skipped when it is not built).
  - `internal/ui`: the Gio window in the Blackfyre look, with the charts tab (`charts*.go`: ten minutes of telemetry, diagnostics and pose on one time axis; series colours validated per mode); `internal/ui/fonts`: Archivo and JetBrains Mono with their OFL licences. `internal/appearance` and `internal/winreg` are copied from bopen (MIT, `desktop/LICENSE.bopen`).
- `tools/virtual-robot/`: the virtual robot, a native C++17 program (CMake, Ninja) running `core/robot/robot_tick` behind a pseudo-terminal that acts as the robot's USB port (frames and debug text as on the board), optionally a second one for the bridge link, or a simulated bridge. Simulated supply and servo current; commands on standard input (`supply V`, `current A|auto`, `load K`, `bridge off|idle|gamepad|clear`, `estop on|off`, `unplug`, `plug`, `step N`, `status`, `quit`); `--help` lists the options (`--link`, `--speed`, `--stepped`, `--bridge-pty`). The desktop calibrator connects with `--port build/virtual-robot/usb`.
- `tools/bridge-screenshots/`: renders the bridge screen's pages in chosen states to PNGs with the bridge's own `screen_draw` and `core/status` code on the host (LovyanGFX's Linux back end, no panel).
- `test/`: Unity host tests, one directory per area. `test/golden/` holds the golden files other implementations (the desktop calibrator) test against. They are fixed fixtures: never copy a calibration export over `calibration_defaults.cpp` there; regenerate `protocol_v3.txt` and `leg_pose.txt` only for a deliberate protocol or kinematics change (`SPOODER_WRITE_GOLDEN=1 mise run test`).
- `openspec/`: specifications (`specs/`) and changes (`changes/`). This is the source of truth for intended behaviour.

## Notes

- Never enable stdio over UART on the Servo 2040. The SDK's default UART pins, GPIO0/1, are servo outputs 1 and 2. The link uses UART1 on GPIO20/21.
- On ESP-IDF, POSIX `link()` and C++20 `std::lerp` are visible: hence `robot_link` on the bridge and `interpolate` in `core/movement/vectors.h`.
- `ServoCluster::enable()` on a servo with no pulse set sends it to mid-travel, and `pulse()` enables a disabled servo. `servo_output` relies on both facts. It also turns off each channel's calibration limits (500–2500 µs by default), keeping the cluster's 400–2600 µs hard limits, so calibrated pulses pass unchanged. Read it before changing output handling.
- The bridge never runs WiFi and Bluetooth together: setup mode is its own boot path and never starts Bluetooth; normal operation never starts WiFi.
- Values already in the generated `firmware/bridge/sdkconfig.ttgo-t-display` win over `sdkconfig.defaults`. After changing the defaults, delete that file once so they apply (the setup portal needs `CONFIG_ESP_WIFI_IRAM_OPT=n` to link).
- The bridge's partition table is `firmware/bridge/partitions.csv`, selected by `board_build.partitions` in `platformio.ini`: the pioarduino platform ignores the sdkconfig partition setting. The app is about 1.6 MB; the stock single-app tables (1 MB and 1.5 MB) do not fit it. The platform's own size check misses that (without an `ota_0` partition its limit is the whole 4 MB flash), so `firmware/bridge/scripts/check_image_size.py` checks the built image against the built table's factory partition, and that table against `partitions.csv`, on every build and before every upload; it prints the headroom. Its logic is tested by `mise run test-bridge-build`, which `mise run test` runs.
- Debug text on the robot's USB port must stay printable ASCII (plus tab, CR, LF): the same port carries binary link frames.
- Project skills in `.claude/skills/`:
  - `spooder-firmware-shape`: layout, loop, J6 pins, the protection layer, the link and the bridge.
  - `spooder-leg-kinematics`: the movement reference.
  - `spooder-hardware-check`: flashing both boards, wiring, power, bring-up, the gamepad controls, setup mode with the portal calibration workflow, and bench calibration with the desktop calibrator.
- The desktop calibrator never opens a port by itself: it lists ports (the Servo 2040, USB ID 2e8a:000a, preselected; or the `--port` path) and connects on request. Its unit tests use in-memory ports or pseudo-terminals they create; the e2e tests use only the virtual robot's terminal.
- Diagrams in Markdown (readme, skills, OpenSpec) are Mermaid code blocks, never ASCII art; the interactive architecture diagrams under `.archify/` are the exception. File-tree listings stay as text.
- Record notable changes in `CHANGELOG.md` under **Unreleased** (Keep a Changelog sections), in the same commit as the change.
- A CodeGraph index lives in `.codegraph/` (git-ignored). Refresh it with `codegraph sync` after large changes.
