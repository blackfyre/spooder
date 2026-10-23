# CLAUDE.md

Spooder is firmware for a hexapod robot, derived from [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). There are two firmwares:
- the robot, on a **Pimoroni Servo 2040** (RP2040), in C++ on the pico-sdk with pimoroni-pico;
- the gamepad bridge, on a **LilyGO TTGO T-Display v1.1** (original ESP32), on ESP-IDF + Arduino + Bluepad32. It sends gamepad commands to the robot over a UART link and shows robot status on its screen. Its setup mode (left button held during the boot logo) replaces Bluetooth with a WiFi access point and captive portal, through which a phone calibrates the robot.

A computer can speak the same link protocol on the robot's USB port, alongside the debug text, to calibrate, monitor, control and drive (from the keyboard) the robot without the bridge. Joint calibration can be changed live (in RAM) and exported as the compiled defaults. The desktop calibrator (`desktop/`, Go with a Gio UI) is that computer-side front-end.

I²C sensors are planned (`add-body-sensing`, suspended). See `openspec/changes/`.

Each area has its own `CLAUDE.md` with its layout and rules: `core/`, `firmware/servo2040/`, `firmware/bridge/`, `desktop/`, `tools/`, `test/`.

## Toolchain

mise (`mise.toml`) provides Python, PlatformIO, CMake, Ninja, the Arm GNU toolchain and Go. It also sets `PICO_SDK_PATH`, `PIMORONI_PICO_PATH` and `PICO_BOARD=pimoroni_servo2040`. The SDKs are pinned git submodules in `external/`:
- pico-sdk 2.2.0 and pimoroni-pico v1.29.0-2 (robot);
- the Bluepad32 ESP-IDF + Arduino template (5.0.0-beta0, with its Arduino core and BTstack submodules) and LovyanGFX 1.2.32 (bridge).

```sh
mise install          # toolchains
mise run setup        # fetch the pinned submodules (only the sub-submodules the builds need)
```

## Commands

```sh
mise run build        # CMake build of firmware/servo2040 -> build/servo2040/spooder.uf2 (--serial: USB serial keys)
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
mise run worktree-add <name>     # delivery worktree .claude/worktrees/<name> on deliver/<name>, from origin/main
mise run worktree-remove <name>  # remove it and its local branch
mise run clean
```

**Verification gates:** `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`. Hardware behaviour can only be checked on the boards (the `spooder-hardware-check` skill).

**Flash both boards together** when the link protocol changes (`protocol::VERSION`, now 3). A mismatched pair fails safe: each side discards the other's frames, so the link looks lost and the robot sleeps.

## Layout

- `core/`: hardware-free C++ shared by both firmwares, the virtual robot and the host tests.
- `firmware/servo2040/`: the robot's pico-sdk CMake project.
- `firmware/bridge/`: the bridge's ESP-IDF PlatformIO project.
- `desktop/`: the desktop calibrator (Go module).
- `tools/`: the virtual robot, the bridge screenshot renderer, the delivery worktree and risk-tier scripts.
- `test/`: Unity host tests, and `test/golden/`, the fixtures other implementations test against.
- `hardware/aecert-hexapod-v1/`: upstream's printed parts (STLs and the whole-robot 3MF) at `ca5ed27`, the commit the movement code was ported from, copied unchanged; its README records the source and the measured link lengths against the firmware's.
- `openspec/`: specifications (`specs/`) and changes (`changes/`). This is the source of truth for intended behaviour.

## Rules

- Debug text on the robot's USB port must stay printable ASCII (plus tab, CR, LF): the same port carries binary link frames.
- Diagrams in Markdown (readme, skills, OpenSpec) are Mermaid code blocks, never ASCII art; the interactive architecture diagrams under `.archify/` are the exception. File-tree listings stay as text.
- **Dependencies are pinned**: toolchains to exact versions in `mise.toml` (Python to its minor line), SDKs as git submodules at fixed tags, the bridge's PlatformIO platform by release URL, Go modules by the committed `desktop/go.sum`, script dependencies inline (`make_fonts.py`). Never let a build resolve an unpinned version; change pins in a reviewed commit, after reading the upstream changelog, and prefer releases at least 7 days old unless the update is a security fix. Not in place yet: vulnerability scanning (start with `govulncheck` for `desktop/`, adopted through a shrink-only baseline if it finds issues), a monthly update cadence, and SBOMs, which only matter once there are releases.
- Record notable changes in `CHANGELOG.md` under **Unreleased** (Keep a Changelog sections), in the same commit as the change.
- **Reviews** follow `REVIEW.md` (severity, evidence, the nit cap, skipped paths, known bug classes, the security lens, risk tiers). Every finding is answered: fixed, or closed as wrong, intentional or not mattering; pre-existing ones are reported, not fixed in the change. When the same problem is flagged twice, add the rule that prevents it to `REVIEW.md`, a `CLAUDE.md` or a skill.
- **Delivering changes:** the `deliver-changes` workflow (`.claude/workflows/README.md`).
- Project skills in `.claude/skills/`:
  - `spooder-firmware-shape`: layout, loop, J6 pins, the protection layer, the link and the bridge.
  - `spooder-leg-kinematics`: the movement reference.
  - `spooder-hardware-check`: flashing both boards, wiring, power, bring-up, the gamepad controls, setup mode with the portal calibration workflow, and bench calibration with the desktop calibrator.
- A CodeGraph index lives in `.codegraph/` (git-ignored). Refresh it with `codegraph sync` after large changes.
