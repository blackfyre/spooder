# Spec Delta

## MODIFIED Requirements

### Requirement: Pose frame contents
The robot SHALL report its pose in a link protocol frame that carries, for each of the 18 joints in servo output order:
- the commanded angle, as the inverse kinematics solved it before the joint's calibration offset;
- the servo angle, the commanded angle plus the calibration offset in use, before the pulse conversion limits it to 0–180°;
- the pulse width last set for the joint's output, or 0 if the output has not been set since power-up.

Angles SHALL be in hundredths of a degree. The frame SHALL also state whether servo output is on. In bench simulation, the pulse widths and the output state SHALL be the servo model's.

#### Scenario: Calibration pose
- **WHEN** every leg holds the calibration pose with the default offsets
- **THEN** the pose frame reports commanded angles of 0°, 90° and 90° for each leg's coxa, femur and tibia, and servo angles equal to those plus each joint's offset

#### Scenario: Output off
- **WHEN** the body has folded the legs in Sleep and servo output is off
- **THEN** the pose frame reports output off, with the last pulses that were set

#### Scenario: Pose in bench simulation
- **WHEN** the robot stands in bench simulation
- **THEN** the pose frame reports the modelled pulses and output on, while no servo output produces a pulse
