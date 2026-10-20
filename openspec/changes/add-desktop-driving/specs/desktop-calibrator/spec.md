# Spec Delta

## ADDED Requirements

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

## MODIFIED Requirements

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

### Requirement: Keyboard controls shown
The mode controls SHALL also be on keys, 1 for Stand, 2 for Calibrate, 3 for Sleep and F for Clear fault, acting like the controls themselves: only while the app is in control. Each control SHALL name its key. While the app is in control, it SHALL show its keyboard controls beside the mode controls: Space for the emergency stop, the mode keys, the drive keys and the walking settings keys, the arrow keys to select a joint and value, the nudge keys with their coarse modifier, and the direction key. The editor's heading SHALL say that editing needs Calibrate whenever the robot is elsewhere. The list SHALL stay readable at the minimum window size.

#### Scenario: Taking control
- **WHEN** the operator takes control
- **THEN** the app shows the keyboard controls, and hides them again on release

#### Scenario: Mode key while observing
- **WHEN** the operator presses 2 while the app only observes
- **THEN** no mode request is sent; once in control, 2 requests Calibrate as the button does
