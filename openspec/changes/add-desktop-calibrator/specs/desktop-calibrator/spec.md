# Spec Delta

## Purpose

A desktop app on the robot's USB port for bench calibration and diagnostics: it watches the robot, can take control for calibration, and writes calibrated defaults into the source tree.

## ADDED Requirements

### Requirement: Connection and log
The app SHALL list the computer's serial ports and preselect a Raspberry Pi RP2040 USB device (USB ID 2e8a:000a) when present. When connected, it SHALL decode link frames from the stream and show all other text as the robot's debug log, line by line. After an unplug it SHALL reconnect to the same device when it reappears, and SHALL show the connection state at all times.

#### Scenario: Robot plugged in
- **WHEN** the app starts with the Servo 2040 on USB
- **THEN** the Servo 2040's port is preselected, and after connecting, the debug log and live telemetry appear

#### Scenario: Unplug and replug
- **WHEN** the USB cable is pulled and plugged back in
- **THEN** the app shows the link as disconnected, then reconnects without user action

### Requirement: Observe and control
After connecting, the app SHALL send command frames every 20 ms that report no connected controller, so it only observes. A "Take control" toggle SHALL make its frames report a connected controller; only then SHALL its Stand, Calibrate and Sleep controls be enabled, each counting as a mode request. Translate and rotate SHALL always be zero. Releasing control, losing the connection or closing the app SHALL stop reporting a controller.

#### Scenario: Observe only
- **WHEN** the app is connected and control is not taken
- **THEN** its command frames report no controller, and its mode controls are disabled

#### Scenario: Close while in control
- **WHEN** the app is closed while in control of the robot
- **THEN** the robot goes to Sleep within about 1 second

#### Scenario: Conflict shown
- **WHEN** the app takes control while the bridge also reports a connected gamepad
- **THEN** the app shows the source conflict reported in telemetry

### Requirement: Emergency stop
The app SHALL offer an emergency-stop control, also on the Space key, that toggles the emergency-stop flag in its command frames whether or not it is in control. The active state SHALL be shown prominently. Releasing it SHALL NOT clear the robot's fault; clearing SHALL be a separate control, enabled only while the app is in control.

#### Scenario: Stop while observing
- **WHEN** the gamepad drives the robot and the operator presses Space in the app
- **THEN** the robot's servo output switches off and the app shows the emergency stop

### Requirement: Telemetry view
The app SHALL show, from the latest telemetry: body mode and gait, whether servo output is on, supply voltage and current, every latched fault by name, the link's valid and discarded frame counts, and which source controls the robot (bridge, USB, none or conflict). Values SHALL be marked as not current when no telemetry has arrived for 1 second.

#### Scenario: Fault on the robot
- **WHEN** telemetry reports an undervoltage fault
- **THEN** the app shows the fault by name and the measured voltage

### Requirement: Calibration editor
The app SHALL show the 18 joints by leg, each with its offset, its pulses for 0° and 180°, and its direction, and SHALL offer fine and coarse nudges for each value by pointer and keyboard. It SHALL follow the joint-calibration rules for confirmation and display: it shows reported values, marks changes awaiting confirmation, and resends them until confirmed. Editing SHALL be enabled only while the app is in control and the robot reports Calibrate.

#### Scenario: Nudge a joint
- **WHEN** the operator selects leg 3's tibia and nudges its 0° pulse by +10 µs in Calibrate
- **THEN** the value is marked pending until the robot's report confirms it, and then shows the reported value

#### Scenario: Editing outside Calibrate
- **WHEN** the robot is standing
- **THEN** the editor's controls are disabled

### Requirement: Export into a checkout
The app SHALL export the reported table as the complete text of `core/movement/calibration_defaults.cpp`. It SHALL write the file only into a directory the user has chosen and that contains a spooder checkout, only after showing the difference from the existing file, and only on confirmation. It SHALL also offer copying the text.

#### Scenario: Write the defaults
- **WHEN** the operator exports to their checkout and confirms the shown difference
- **THEN** `core/movement/calibration_defaults.cpp` in that checkout holds the reported table, and nothing else in the checkout changes

#### Scenario: Not a checkout
- **WHEN** the chosen directory does not contain `core/movement/calibration_defaults.cpp`
- **THEN** the app refuses to write and says why

### Requirement: Appearance
The app SHALL use the Blackfyre design of blackfyre.ninja: its light (paper) and dark palettes, the orange accent, Archivo for text and JetBrains Mono for labels and values, and tactile buttons. It SHALL follow the system's light or dark setting.

#### Scenario: System in dark mode
- **WHEN** the system is set to a dark appearance
- **THEN** the app uses the Blackfyre dark palette
