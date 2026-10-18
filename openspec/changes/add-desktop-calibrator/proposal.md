# Proposal

## Why

Calibration and diagnostics happen mostly on the bench, next to a computer. A desktop app on the robot's USB port avoids the bridge, WiFi and a phone screen, and it can write the calibrated defaults straight into the checkout instead of going through copy and paste. The robot side, the wire contract and the defaults format come from `add-live-joint-calibration`; this change is a second front-end for them, next to the bridge's setup portal.

## What Changes

- **A Go desktop app, `spooder-calibrator`, under `desktop/`**, with a Gio user interface in the Blackfyre look of blackfyre.ninja: its palette, Archivo and JetBrains Mono, and tactile buttons; light and dark follow the system.
- **Connection:** it lists serial ports, preselects the Servo 2040 by its USB ID, reconnects after an unplug, and shows the robot's debug text in a log view next to the decoded frames.
- **Observe by default, control on request.** On connecting, the app only watches. A "Take control" toggle makes it report a connected controller, and only then are the Stand, Calibrate and Sleep buttons live. Translate and rotate are always zero. Releasing control, disconnecting or closing the window removes the controller, and the robot sleeps.
- **An e-stop button** (also the Space key) that works whether or not the app is in control.
- **Telemetry:** body mode, gait, output, supply voltage and current, faults by name, link statistics, and which source controls the robot (bridge, USB or conflict).
- **Calibration editor:** the 18 joints with offset, pulse end points and direction, fine and coarse nudges (buttons and keys). Values shown are the robot's reports; a pending marker shows changes awaiting confirmation.
- **Export:** writes `core/movement/calibration_defaults.cpp` into a spooder checkout the user picks, after showing a diff, or copies the text.
- **mise tasks** to build, run and test the app.

## Capabilities

### New Capabilities

- `desktop-calibrator`: The desktop app: connection and log, observe and control, the e-stop, telemetry, the calibration editor, export into a checkout, and the look.

### Modified Capabilities

None.

## Impact

- **Depends on** `add-live-joint-calibration`: protocol version 2, the USB link, source arbitration, `test/golden/protocol_v2.txt` and `test/golden/calibration_defaults.cpp`.
- **New code:** `desktop/` (Go module), including a Go implementation of the link protocol, tested against the golden vectors.
- **Toolchain:** Go pinned in `mise.toml`; Gio needs cgo and the Wayland, X11 and EGL development packages on Linux.
- **Reference:** `~/Projects/blackfyre/bopen` (a Gio app with theme, appearance and release scaffolding) is copied and adapted, since its packages are `internal/`. The look is taken from `~/Projects/blackfyre/legendary-winner` and its Claude Design project "Blackfyre", which is read only.
- **Out of scope:** walking the robot from the app, release packaging and installers, macOS, and UI code shared with the setup portal.
