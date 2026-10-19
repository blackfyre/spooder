# Changelog

All notable changes to Spooder are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). The project has no releases yet; changes collect under **Unreleased**.

## [Unreleased]

### Added

- **Robot on the Pimoroni Servo 2040.** A pico-sdk firmware running a 20 ms control tick: link input, power protection, supervisor, calibration, body state machine, servo output on GPIO0–17, the status LEDs and telemetry.
- **Movement core.** The upstream inverse kinematics, Bézier foot paths and gait table (tri, wave, ripple, bi, quad), ported to a non-blocking, hardware-free state machine (Initialise, Stand, Walk, Sleep, Calibrate) on a fixed 5 ms grid, with host tests.
- **Power protection.** Overcurrent, sustained overload, brown-out, low and high voltage, and the emergency stop. Faults latch and cut servo output at once. Clearing a fault never restarts motion; a new mode request must follow. Limits are set at build time for the fitted servos and supply.
- **Status LEDs.** The six WS2812 LEDs show the operating state and faults.
- **Gamepad bridge on the TTGO T-Display.**
  - Bluepad32 pairs one gamepad, mapped to modes, gaits, height, speed and stride.
  - Commands go to the robot over UART every 20 ms.
  - The screen shows the robot's telemetry, after a boot logo and start-up log.
  - The right button toggles the emergency stop.
- **Link protocol, version 2.** CRC-16 framed command, telemetry, calibration set and calibration report frames, and a pose frame for the USB host at 10 Hz. Golden vectors pin the format for every implementation.
- **USB link and source arbitration.** A computer can speak the link protocol on the robot's USB port alongside the debug text. The robot follows whichever link reports a connected controller, and sleeps on a conflict. `mise run build --serial` keeps the USB serial keys for bring-up without the bridge.
- **Live joint calibration.**
  - Per joint: an offset, the pulses for 0° and 180°, and the direction.
  - Changes are accepted only in Calibrate with output allowed, clamped (offset within ±30° of the default, pulses 400–2600 µs at least 1000 µs apart) and applied at no more than 60°/s.
  - They are held in RAM, confirmed by the robot's reports, and exported as the complete `calibration_defaults.cpp`.
- **Bridge setup mode.** Holding the left button during the boot logo starts a WiFi access point, joined with a QR code shown on the screen, and a captive portal in the Blackfyre look. Through it, a phone controls the robot's mode and calibrates its joints. Bluetooth and WiFi never run together.
- **Desktop calibrator** (Go, Gio).
  - It observes or controls the robot over USB, with an e-stop on Space.
  - It shows telemetry, the joint editor with fine and coarse nudges, the debug log and decoded frames.
  - It exports the calibration into a checkout, with a diff and an atomic write.
  - It reconnects to the board by USB serial number.
- **Legs pane.** Forward kinematics of the reported pose, each joint's command, servo angle and pulse, rings at the servo limits, and a top view of all six legs with the front marked.
- **Readable robot console.** `mise run monitor` shows the robot's debug text with timestamps, decodes frames with `--frames`, and sends keys only with `--keys`.
- **Virtual robot.**
  - It runs the robot's own control tick natively, behind a pseudo-terminal that acts as the USB port, optionally with a bridge link or a simulated bridge.
  - Supply, current, load, e-stop and unplugging are simulated and can be driven from standard input.
  - `mise run desktop-sim` runs it together with the calibrator, and `mise run e2e` tests the calibrator against it.
- **Bridge image-size check.** Every build and upload checks the app against the factory partition of the custom partition table, and reports the headroom.
- **Toolchain.** mise provides every toolchain, and the SDKs are pinned git submodules. OpenSpec holds the specifications.
- **Project files.** An MIT licence, this changelog and an expanded readme.

### Changed

- The robot moved from the Arduino Nano RP2040 Connect (PCA9685 servo driver, Bluepad32 on its NINA module) to the Servo 2040, and the gamepad moved from the robot to the bridge.
- Coxa offsets are shown from the servo centre (stored value minus 90°) in the portal and the calibrator. The stored values, the link and the export are unchanged.
- Compiled defaults must let every servo reach the calibration pose: the IK angle there, plus the offset, within 0–180°.

- The desktop calibrator's top bar puts the ports, Refresh, Connect, the control toggle and the e-stop on one aligned row of equal-height buttons, with the link state beneath.
- While in control, the desktop calibrator shows its keyboard controls under the mode buttons.
- The desktop calibrator's mode controls are on keys too: 1 Stand, 2 Calibrate, 3 Sleep, F Clear fault, only while in control.

### Removed

- The Nano RP2040 Connect firmware (it remains in the history).

### Suspended

- I²C body sensing (VL6180X range, LSM6DSO IMU), until the sensor boards are found.

## 2024-02-09: prototype

The first iteration on Aecert's Hexapod V1: an Arduino build for the Nano RP2040 Connect with the upstream movement code, a Bluepad32 gamepad, an SH1107 display, IMU (IKS01A3) and range (VL6180X) sensor experiments, and a first walk.
