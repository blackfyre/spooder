# Changelog

All notable changes to Spooder are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). The project has no releases yet; changes collect under **Unreleased**.

## [Unreleased]

### Added

- **Bridge screen pages.** The bridge's status screen is graphical, in the Blackfyre dark palette with Archivo and JetBrains Mono, and has three pages switched by a short press of the left button: drive (mode chip, voltage and current with bars, height and speed, stride, stick and output), power (the last 60 s of voltage and current as 10 Hz traces, with the window's lowest voltage and highest current) and link (frames per second each side accepts and rejects, telemetry age, source, gamepad and its battery). The e-stop, every latched fault and a lost link show in a status strip on every page. Only the bridge needs flashing.
- **Held extremes.** The bridge's drive page marks the recent peak current and lowest voltage on their bars, and the desktop calibrator's current and supply charts draw them as named lines: held 3 s after they were reached, then easing back to the reading at 2 A/s or 0.5 V/s. On the bridge, a current beating the held peak by more than 0.1 A, or a voltage sagging 0.05 V below the held low, turns its value amber for a second.
- **Bridge screenshots.** `mise run bridge-screenshots` renders the bridge screen's pages and states to `docs/images/bridge-*.png` with the bridge's own drawing code on the host; the readme shows them.
- **Robot on the Pimoroni Servo 2040.** A pico-sdk firmware running a 20 ms control tick: link input, power protection, supervisor, calibration, body state machine, servo output on GPIO0–17, the status LEDs and telemetry.
- **Movement core.** The upstream inverse kinematics, Bézier foot paths and gait table (tri, wave, ripple, bi, quad, hop), ported to a non-blocking, hardware-free state machine (Initialise, Stand, Walk, Sleep, Calibrate) on a fixed 5 ms grid, with host tests.
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
- **Calibrator icon.** A hexapod from above in the Blackfyre colours (`desktop/assets`), and `mise run desktop-install`, which installs a launcher and the icon for the current user on Linux.
- **Gestures.** A Perform body mode plays say hi and wiggle from Stand: keyframes of body shifts over planted feet and lifted legs, eased and rate-limited to 300 mm/s per foot, interrupted at once by Sleep, a stale command, the e-stop or a fault, and ended after the current keyframe by any other request. Triggered by L2 + a face button on the gamepad and by buttons or keys 4–8 in the desktop calibrator; the bridge screen, the calibrator and the console show the gesture playing.
- **Gamepad rumble.** The bridge rumbles the gamepad when a protection fault latches, its e-stop switches on or off, the robot link is lost or regained, or a gesture request does not play; one pattern per event, by priority, never repeating within a second.
- **Servo profiles and the joint speed budget.**
  - The build names the fitted servo model (`SPOODER_SERVO_PROFILE`, default `MG996R`). Its rated speed and the share usable under load (MG996R: 429°/s, 0.6) set a joint speed budget.
  - Movement slows its own clock, in 20 ms pieces, wherever full speed would turn a joint faster than the budget. Feet keep their paths, and motion stays independent of the tick length.
  - An output rate backstop never moves a joint faster than the servo's rated speed. It reports on the USB console when it engages.
- **Project files.** An MIT licence, this changelog and an expanded readme.

### Changed

- The robot moved from the Arduino Nano RP2040 Connect (PCA9685 servo driver, Bluepad32 on its NINA module) to the Servo 2040, and the gamepad moved from the robot to the bridge.
- Coxa offsets are shown from the servo centre (stored value minus 90°) in the portal and the calibrator. The stored values, the link and the export are unchanged.
- Compiled defaults must let every servo reach the calibration pose: the IK angle there, plus the offset, within 0–180°.
- The overcurrent trip has two levels. Above the trip current (9 A) it latches only once the current has stayed there for `SPOODER_TRIP_PERSIST_MS` (60 ms), so the surge of all servos starting together passes; above the new ceiling, `SPOODER_CEILING_CURRENT_A` (13 A), one reading still cuts output at once. Limits files need both new constants.
- Servo output switches on two legs per tick (legs 1–2, then 3–4, then 5–6), so the servos' start surges no longer coincide: the 11.8 A measured with all 18 starting together drops to about a third. Switching off stays immediate.

- The desktop calibrator's top bar puts the ports, Refresh, Connect, the control toggle and the e-stop on one aligned row of equal-height buttons, with the link state beneath.
- While in control, the desktop calibrator shows its keyboard controls under the mode buttons.
- **Four-view window in the desktop calibrator.** "4 views" in the top bar opens a second window that draws the whole robot from the pose: top, front, left and isometric views. The body is a schematic plate, and the window shows the ground at the lowest foot, the selected leg, the servo limits and the leg numbers. The three orthographic views share one scale.
- **Charts in the desktop calibrator.** A charts tab keeps ten minutes of supply voltage, current, link frame rates, a mode/output/fault timeline, joint speeds against the servo budget, the selected leg's servo angles, the budget's motion share, the control tick's time and backstop hits. It has 30 s, 2 min and 10 min windows, a hover readout on every chart, and a CSV export.
- **Diagnostics frame.** The robot sends its internals to the USB host at 10 Hz: the budget's motion share, backstop count, tick time, averaged current, its power limits and the servo profile's speeds. It is a new frame type of protocol version 3, so the bridge is unaffected.
- The desktop calibrator drives the robot from the keyboard while in control. W/S walk, A/D steer (as the bridge's car layout: on the spot without walking, mirrored when reversing) and Q/E step sideways. Z/X set the speed dial, C/V the height, G the gait and T the stride mode. Losing the window focus stops the input.
- The desktop calibrator's mode controls are on keys too: 1 Stand, 2 Calibrate, 3 Sleep, F Clear fault, only while in control.

- Walking and turning are superposed instead of shared: turning while walking keeps the full forward stride and follows an arc, with one common scale when a stroke would get too long or unreachable.
- The bridge steers like a car by default: left stick Y walks, left stick X steers, mirrored when reversing; `b` switches to the crab layout. The right stick turns the body the way it points (it was inverted). The screen shows the stick mode.

- The link protocol is version 3: the command frame carries a gesture request and its counter (14-byte payload), telemetry the gesture playing. Flash both boards together.

- Walking at higher speed dial settings is slower: as fast as the servos can follow, no faster.
- Feet are fitted to the servos' 0–180° travel outside Calibrate: a target that would take a joint past its end (with the calibration offsets) is moved just enough, continuously, so the joints agree with each other instead of one being clamped at its pulse limit.
- The speed dial and the stride mode are eased like translate and height, so changing either no longer moves every foot at once.
- The movement code costs much less on the RP2040: the stride's reach is solved exactly instead of searched, trigonometry and powers are gone from the foot paths, and phase tracking computes the stride only on a phase change.
- A leg entering its cycle part-way through a phase (entering Walk, a gait change) starts from where its foot is, instead of jumping onto the path (up to 45° in one tick in Bi and Ripple).
- Foot targets are valid only within the femur and tibia's reach from the femur joint, less 4 mm (295 mm), not 340 mm from the coxa pivot. Strides that would not reach shrink smoothly, for single motions too. At body height 100 they are shorter than before: 50–78% at the default dial.

- The desktop calibrator and the setup portal follow the Blackfyre design system's tokens: its exact colours (with `bf-ok` for the status green and the dark-panel shades in dark mode), 10 px buttons on raised edges (3 px under dark buttons, 2 px with a highlight under light and orange ones), 22 px panels and 14 px cards, the label, kicker and micro letter-spacing, and display titles ending in an orange full stop. The portal also gets 44 px touch targets, numbered section kickers, hairline-outlined panels with the panel shadow, and the accent focus ring. Text on the orange stays ink: the system's white reaches only 3.78:1.
- The desktop calibrator's debug log and frames pane are selectable across lines with the mouse (Shift+click extends) and copied with Ctrl+C (Cmd+C on macOS), which no longer lowers the height. They follow new lines only while their end is in view; scrolled up or holding a selection, the view stays on the same lines, also as the oldest are dropped.

- Telemetry on every link also reports the bridge link: up or down, whether a gamepad is on it, and its frame counts (reserved bytes, no version bump). The desktop calibrator shows it in a Bridge cell, and `mise run monitor --frames` on each telemetry line.

### Removed

- The Nano RP2040 Connect firmware (it remains in the history).

### Suspended

- I²C body sensing (VL6180X range, LSM6DSO IMU), until the sensor boards are found.

## 2024-02-09: prototype

The first iteration on Aecert's Hexapod V1: an Arduino build for the Nano RP2040 Connect with the upstream movement code, a Bluepad32 gamepad, an SH1107 display, IMU (IKS01A3) and range (VL6180X) sensor experiments, and a first walk.
