# desktop-calibrator Specification

## Purpose
The desktop calibrator: a Go application that speaks the link protocol on the robot's USB port to observe, control and calibrate the robot without the bridge, in the Blackfyre look.

## Requirements

### Requirement: Explicit port
The app SHALL accept a serial port path on its command line (`--port PATH`). That port SHALL be listed and preselected whenever the path exists, even when the system's port enumeration does not report it (a pseudo-terminal, or a symbolic link to one), and SHALL be opened without setting modem control lines. Reconnection SHALL follow the path, so a new terminal behind the same link is found again.

#### Scenario: Virtual robot
- **WHEN** the app is started with `--port build/virtual-robot/usb` while the virtual robot runs
- **THEN** that port is preselected, and connecting shows the virtual robot's log and telemetry

#### Scenario: Path absent
- **WHEN** the given path does not exist
- **THEN** it is not listed, and a connection to it waits until it appears

### Requirement: Connection and log
The app SHALL list the computer's serial ports and preselect a Raspberry Pi RP2040 USB device (USB ID 2e8a:000a) when present. When connected, it SHALL decode link frames from the stream and show all other text as the robot's debug log, line by line. After an unplug it SHALL reconnect to the same device when it reappears, and SHALL show the connection state at all times.

#### Scenario: Robot plugged in
- **WHEN** the app starts with the Servo 2040 on USB
- **THEN** the Servo 2040's port is preselected, and after connecting, the debug log and live telemetry appear

#### Scenario: Unplug and replug
- **WHEN** the USB cable is pulled and plugged back in
- **THEN** the app shows the link as disconnected, then reconnects without user action

### Requirement: Observe and control
After connecting, the app SHALL send command frames every 20 ms that report no connected controller, so it only observes. A "Take control" toggle SHALL make its frames report a connected controller. Only then SHALL its Stand, Calibrate and Sleep controls be enabled, each counting as a mode request, and its drive keys act. Translate and rotate SHALL be zero except while the drive keys drive the robot in Walk. Releasing control, losing the connection or closing the app SHALL stop reporting a controller.

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

### Requirement: Keyboard controls shown
The mode controls SHALL also be on keys, 1 for Stand, 2 for Calibrate, 3 for Sleep and F for Clear fault, acting like the controls themselves: only while the app is in control. Each control SHALL name its key. While the app is in control, it SHALL show its keyboard controls beside the mode controls: Space for the emergency stop, the mode keys, the drive keys and the walking settings keys, the arrow keys to select a joint and value, the nudge keys with their coarse modifier, and the direction key. The editor's heading SHALL say that editing needs Calibrate whenever the robot is elsewhere. The list SHALL stay readable at the minimum window size.

#### Scenario: Taking control
- **WHEN** the operator takes control
- **THEN** the app shows the keyboard controls, and hides them again on release

#### Scenario: Mode key while observing
- **WHEN** the operator presses 2 while the app only observes
- **THEN** no mode request is sent; once in control, 2 requests Calibrate as the button does

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

### Requirement: Bridge link shown
The app SHALL show the bridge link's state from telemetry: down, or up with or without a gamepad, and its valid and discarded frame counts in every state.

#### Scenario: Confirming the bridge link from the computer
- **WHEN** the bridge runs without a gamepad and the app is connected to the robot
- **THEN** the app shows the bridge link up, with no gamepad, and counts that rise

### Requirement: Driving from the keyboard
While the app is in control, held keys SHALL drive the robot as the bridge's gamepad does in its car layout:
- W and S SHALL walk forward and back at full input.
- A and D SHALL steer left and right at full input. Without W or S they SHALL turn the robot on the spot, and while S alone is held they SHALL be mirrored, as a car's steering is.
- Q and E SHALL step left and right.

Opposite keys held together SHALL cancel. Pressing a drive key while the app requests Stand SHALL request Walk once. Releasing every drive key SHALL leave Walk with zero input. In Sleep and Calibrate the drive keys SHALL have no effect.

Z and X SHALL lower and raise the speed dial by 10, C and V the body height by 10, each within 0–100. G SHALL select the next gait, and T SHALL switch between the dynamic and fixed stride. They SHALL start at speed 44, height 0, Tri and the dynamic stride, and the app SHALL send them in every command frame while in control.

Losing the window's focus, releasing control or losing the connection SHALL drop every held drive key. While in control, the app SHALL show the gait, speed, height and stride it sends.

#### Scenario: Walking forward
- **WHEN** the app is in control with Stand requested and the operator holds W
- **THEN** the app requests Walk and its command frames carry translate Y 100, and releasing W returns them to zero while Walk stays requested

#### Scenario: Steering and turning on the spot
- **WHEN** the operator holds W and D
- **THEN** the frames carry translate Y 100 and a negative rotate (a clockwise turn), and with D alone the same negative rotate and no translate

#### Scenario: Reversing mirrors the steering
- **WHEN** the operator holds S and D
- **THEN** the frames carry translate Y −100 and a positive rotate

#### Scenario: Settings
- **WHEN** the operator presses X twice, V once, G once and T once while in control
- **THEN** the frames carry speed 64, height 10, the Wave gait and the fixed stride

#### Scenario: Focus lost while driving
- **WHEN** the window loses focus while W is held
- **THEN** the frames return to zero translate and rotate

#### Scenario: Drive keys while observing
- **WHEN** the operator holds W while the app only observes
- **THEN** nothing changes in its command frames

### Requirement: Charts
The app SHALL keep the last ten minutes of telemetry, diagnostics and pose on the current connection, and SHALL show them in a charts tab as stacked charts sharing one time axis:
- supply voltage, with the minimum, maximum and brown-out voltages;
- current and averaged current, with the trip and sustained currents;
- accepted frames per second on the robot's link, the bridge's link and the host link;
- rejected frames per second on the same three links;
- a timeline of the body mode, servo output and faults;
- the fastest joint speed of each joint kind (coxa, femur, tibia), over all legs, with the joint speed budget and the rated speed;
- the selected leg's servo angles, with the 0° and 180° limits;
- the budget's motion share;
- the control loop's longest and mean working time, with the 20 ms period;
- backstop hits per second.

Each chart SHALL have one value axis. Charts of two or more series SHALL have a legend, and reference lines SHALL be named. A window selector SHALL choose the last 30 seconds, 2 minutes or 10 minutes. Hovering a chart SHALL show the time and the values at that moment on every chart. Charts whose data the robot does not send (a robot without diagnostics, or no pose frames) SHALL say so instead of drawing nothing.

#### Scenario: Supply sag while walking
- **WHEN** the robot walks and the supply voltage dips
- **THEN** the voltage chart shows the dip against the minimum voltage, and the timeline shows Walk at that time

#### Scenario: Window
- **WHEN** the operator selects 30 s
- **THEN** every chart shows the last 30 seconds, and selecting 10 min shows up to ten minutes

#### Scenario: Hover
- **WHEN** the operator hovers a moment on any chart
- **THEN** every chart marks that moment and shows its values there

#### Scenario: New connection
- **WHEN** the app connects to a robot
- **THEN** the history starts empty

### Requirement: CSV export of the history
The charts tab SHALL offer to save the kept history as a CSV file: one row per telemetry sample, with its time, the telemetry values, the latest diagnostics and the joint speeds at that time, and a header row naming the columns and their units. The app SHALL write the file whole (written to a temporary file, then renamed), in the user's downloads directory (the home directory when there is none), and SHALL name the file it wrote.

#### Scenario: Save
- **WHEN** the operator saves the history after a minute of telemetry
- **THEN** a CSV file with a header row and about 600 rows is written, and the app shows its path

### Requirement: Selectable text panes
The debug log and the frames pane SHALL let the user select their text with the pointer, within a line and across lines, by dragging, and extend a selection with Shift and a click. The selection SHALL be drawn in the accent colour. The panes SHALL stay read-only. Ctrl+C, and Cmd+C on macOS, SHALL copy the selected text of the pane shown to the clipboard: the lines as shown, joined by newlines. That shortcut SHALL NOT change the walking settings. Selection SHALL NOT lay out more than the lines in view.

#### Scenario: Copy across lines
- **WHEN** the user drags from the middle of one log line to the middle of a later one and presses Ctrl+C
- **THEN** the clipboard holds the selected part of the first line, the lines between, and the selected part of the last line, separated by newlines

#### Scenario: Copy shortcut in control
- **WHEN** the app is in control in Stand and the user presses Ctrl+C
- **THEN** the body height is unchanged

### Requirement: Text panes follow the newest line
The debug log and the frames pane SHALL follow new lines only while their bottom is visible. While the user has scrolled up, or holds a selection, new lines SHALL move neither the view nor the selection, also when the oldest lines are dropped. While all the lines fit in the pane, they SHALL stay aligned to its bottom. Scrolling back to the bottom, or clearing the selection there, SHALL resume following.

#### Scenario: Reading back
- **WHEN** the user scrolls up in a full log while the robot keeps printing
- **THEN** the lines in view stay in place until the user scrolls back to the bottom, after which the log follows again

#### Scenario: Selecting at the bottom
- **WHEN** the user selects a line at the bottom of the log while new lines arrive
- **THEN** the view and the selection stay in place until the user scrolls to the bottom again
