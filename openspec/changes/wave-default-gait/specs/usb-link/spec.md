## MODIFIED Requirements

### Requirement: Operator source arbitration
The robot SHALL take movement commands from at most one operator source at a time:
- from the bridge link while it reports a connected controller;
- otherwise from the USB link while it reports a connected controller;
- otherwise no source, which requests Sleep with the starting gait, Wave.

The robot SHALL start in Wave, the gait every controller starts in, so that telemetry reports Wave before a source is in control and while the body is stopped.

If both links report a connected controller, the robot SHALL treat the situation as no source, request Sleep, and report the conflict in telemetry until one of them stops. Calibration changes SHALL be accepted from either link, subject to the joint calibration guards. An emergency stop signalled by either link SHALL latch the emergency-stop fault, and a fault clear SHALL be accepted only from the source in control.

#### Scenario: Desktop on the bench, no bridge
- **WHEN** no bridge is connected and a host on USB reports a connected controller and requests Calibrate
- **THEN** the robot follows the USB link's commands

#### Scenario: Wave before a source is in control
- **WHEN** the robot starts with no servo supply and no source in control
- **THEN** telemetry reports the Wave gait

#### Scenario: Both claim control
- **WHEN** the bridge reports a connected gamepad and the USB host also reports a connected controller
- **THEN** the robot requests Sleep and telemetry reports a source conflict

#### Scenario: Emergency stop from the idle source
- **WHEN** the gamepad controls the robot and the USB host signals emergency stop
- **THEN** servo output switches off within 20 ms and the emergency-stop fault latches
