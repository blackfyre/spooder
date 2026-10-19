# pose-telemetry Specification

## Purpose
Lets a computer on the robot's USB port see what the robot commands each joint to do: the angle the body asks for, the servo angle after calibration, and the pulse set for the output.

## Requirements

### Requirement: Pose frame contents
The robot SHALL report its pose in a link protocol frame that carries, for each of the 18 joints in servo output order:
- the commanded angle, as the inverse kinematics solved it before the joint's calibration offset;
- the servo angle, the commanded angle plus the calibration offset in use, before the pulse conversion limits it to 0–180°;
- the pulse width last set for the joint's output, or 0 if the output has not been set since power-up.

Angles SHALL be in hundredths of a degree. The frame SHALL also state whether servo output is on.

#### Scenario: Calibration pose
- **WHEN** every leg holds the calibration pose with the default offsets
- **THEN** the pose frame reports commanded angles of 0°, 90° and 90° for each leg's coxa, femur and tibia, and servo angles equal to those plus each joint's offset

#### Scenario: Output off
- **WHEN** the body has folded the legs in Sleep and servo output is off
- **THEN** the pose frame reports output off, with the last pulses that were set

### Requirement: Pose frame on USB only
The robot SHALL send the pose frame on its USB link only, about ten times per second, whole or not at all, under the same rules as other USB frames. It SHALL NOT send pose frames on the bridge link.

#### Scenario: Bridge link unaffected
- **WHEN** the robot runs with the bridge connected and a host on USB
- **THEN** the host receives pose frames at about 10 Hz and the bridge receives none

#### Scenario: Host not reading
- **WHEN** a host holds the USB port open without reading
- **THEN** pose frames that do not fit are dropped whole and the control loop keeps its period

### Requirement: Pose frame in the link protocol
The pose frame SHALL be an additional frame type of link protocol version 2, without changing the protocol version or any other frame. Golden vectors for it SHALL be part of the protocol's golden frame file, and every implementation SHALL decode and re-encode them byte for byte. A receiver that does not expect pose frames SHALL ignore them.

#### Scenario: Bridge firmware unchanged in behaviour
- **WHEN** a bridge built before this change is connected to a robot with it
- **THEN** the link works as before, because the protocol version is unchanged and no pose frame is sent on the bridge link
