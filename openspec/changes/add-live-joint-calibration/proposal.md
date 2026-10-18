# Proposal

## Why

Calibrating the new robot means adjusting 18 joints: their angle offsets and, for MG996R clones, the real pulse end points. Today each try is an edit, a rebuild and a flash. The robot should accept live calibration changes, guard them, report what it uses, and let the result become its compiled defaults. Two front-ends will drive this: the bridge's WiFi setup portal for the field (`add-bridge-setup-portal`) and a desktop calibrator over USB for the bench (`add-desktop-calibrator`). This change is the robot side and the shared contract they both rely on.

## What Changes

- **Joint calibration table on the robot.**
  - Each of the 18 joints has an angle offset, pulses for 0° and 180°, and a direction.
  - Compiled defaults are loaded at start-up, seeded with today's `LEG_OFFSETS`. Live changes stay in RAM until the robot restarts.
  - Changes are accepted only in Calibrate with output allowed, clamped (offset within 30° of the default; pulses 400–2600 µs, at least 1000 µs apart) and applied at no more than 60°/s.
  - The robot reports the table in effect after each change and at 1 Hz in Calibrate.
- **IK and pulse conversion use the table** instead of fixed per-leg offsets and a fixed 500–2500 µs range.
- **Protocol version 2.** Calibration set and calibration report frames are added. Telemetry gains a source-conflict flag.
  - **BREAKING (link):** both firmwares must be flashed together.
- **The USB serial port becomes a second link port.** It carries the same frames, interleaved with debug text, so a computer can calibrate, monitor and control the robot without the bridge.
  - Source arbitration: whichever link reports a connected controller is in control. If both claim control, the robot sleeps and reports a conflict until one stops.
  - An e-stop from either link latches.
- **The compiled-defaults file** has a fixed format, with a golden copy that every exporter's tests check.
- **Golden frame vectors** for protocol version 2 are generated from the C++ encoder and committed, so another implementation (the Go desktop app) can be tested against them.

## Capabilities

### New Capabilities

- `joint-calibration`: The per-joint calibration table: defaults, live changes and their guards, the rate limit, reports and confirmation, and the compiled-defaults file and export contract.
- `usb-link`: Link frames over the robot's USB serial port alongside debug text, operator source arbitration between the bridge and USB, and USB staleness.

### Modified Capabilities

- `leg-actuation`: "Foot target to joint angles" and "Joint angle to servo pulse" use each joint's calibration (offset, pulse range, direction).

## Impact

- **Depends on** `port-movement-to-servo2040` and `add-gamepad-bridge`, which must be archived first; the `leg-actuation` delta is written against the specification after the port. `controller-link` (from `add-gamepad-bridge`) is extended by protocol version 2; the frame details live in this change's design.
- **Code:**
  - `core/movement`: `joint_calibration`, `calibration_defaults.cpp`; `LEG_OFFSETS` removed.
  - `core/protocol`: version 2, new frame types, golden vectors; `link_receiver` gains the calibration guards and source arbitration.
  - `firmware/servo2040`: the runtime table, reports, a whole-frame TX queue, and the USB link alongside stdio.
  - `firmware/bridge`: protocol version 2 (decode the new frames; normal operation sends no calibration).
- **Out of scope:** the front-ends themselves, and storing calibration on the robot.
