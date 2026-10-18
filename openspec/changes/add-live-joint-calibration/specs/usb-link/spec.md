# Spec Delta

## Purpose

Lets a computer on the robot's USB port use the same link protocol as the gamepad bridge, for calibration, telemetry and bench control, and decides which operator source may control the robot.

## ADDED Requirements

### Requirement: Link frames over USB
The robot SHALL accept and send link protocol frames on its USB serial port, in both directions, with the same framing, versioning and validation as the bridge link. Debug text SHALL continue on the same port. Debug text SHALL contain only printable ASCII, tab, carriage return and line feed, and SHALL never be written inside a frame. Telemetry and calibration reports SHALL be sent on the USB port as well as the bridge link.

#### Scenario: Frames and text interleaved
- **WHEN** the robot prints a debug line between two telemetry frames on USB
- **THEN** a host reading the port decodes both frames and the complete text line

#### Scenario: Host not reading
- **WHEN** a host holds the USB port open without reading
- **THEN** the control loop keeps its period, and frames that could not be sent are dropped whole

### Requirement: Operator source arbitration
The robot SHALL take movement commands from at most one operator source at a time:
- from the bridge link while it reports a connected controller;
- otherwise from the USB link while it reports a connected controller;
- otherwise no source, which requests Sleep.

If both links report a connected controller, the robot SHALL treat the situation as no source, request Sleep, and report the conflict in telemetry until one of them stops. Calibration changes SHALL be accepted from either link, subject to the joint calibration guards. An emergency stop signalled by either link SHALL latch the emergency-stop fault, and a fault clear SHALL be accepted only from the source in control.

#### Scenario: Desktop on the bench, no bridge
- **WHEN** no bridge is connected and a host on USB reports a connected controller and requests Calibrate
- **THEN** the robot follows the USB link's commands

#### Scenario: Both claim control
- **WHEN** the bridge reports a connected gamepad and the USB host also reports a connected controller
- **THEN** the robot requests Sleep and telemetry reports a source conflict

#### Scenario: Emergency stop from the idle source
- **WHEN** the gamepad controls the robot and the USB host signals emergency stop
- **THEN** servo output switches off within 20 ms and the emergency-stop fault latches

### Requirement: USB link staleness
The USB link SHALL follow the same rules as the bridge link: commands are stamped on arrival, a command older than 1 second is stale, and after a gap of 200 ms or more the first frame only sets the counter baselines.

#### Scenario: Host unplugged
- **WHEN** the USB host controlling the robot is unplugged
- **THEN** the robot goes to Sleep within about 1 second
