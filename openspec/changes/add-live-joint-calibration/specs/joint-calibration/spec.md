# Spec Delta

## Purpose

Lets the operator adjust each joint's calibration live on the robot, without rebuilding, from a calibration front-end (the bridge's setup portal or the desktop calibrator), and then carry the result into the firmware's compiled defaults.

## ADDED Requirements

### Requirement: Calibration table
The robot SHALL hold a calibration for each of the 18 joints: an angle offset in degrees, the pulse widths for 0° and 180°, and a direction (normal or reversed). At start-up the table SHALL be loaded from compiled defaults. Initially these are the offsets compiled in today (`kinematics::LEG_OFFSETS`), 500 µs and 2500 µs, and normal direction. Live changes SHALL be kept in RAM only, until the robot restarts. A restart of the bridge SHALL NOT change the robot's table.

#### Scenario: Restart restores the defaults
- **WHEN** joint values were changed live and the robot restarts
- **THEN** the table holds the compiled defaults again

#### Scenario: Front-end restart keeps live values
- **WHEN** the bridge restarts from setup mode into normal operation, or the desktop calibrator disconnects
- **THEN** the robot keeps the live values, and walking uses them

### Requirement: Calibration changes and their guards
The robot SHALL accept a calibration change only while the body is in Calibrate and servo output is allowed; otherwise it SHALL reject the change. It SHALL clamp each value to its safe range:
- offset within 30° of the joint's compiled default (the defaults include mounting offsets of up to about 90°);
- each pulse 400–2600 µs, with the 180° pulse at least 1000 µs away from the 0° pulse, in either order.

A change to a joint's offset or pulse range SHALL take effect at a bounded rate: the output for that joint SHALL move by no more than 60° per second, as converted through its current calibration.

#### Scenario: Change outside Calibrate
- **WHEN** a calibration change arrives while the body is standing
- **THEN** the change is rejected and the table is unchanged

#### Scenario: Offset clamped
- **WHEN** an offset 45° above the joint's compiled default is requested
- **THEN** the joint's offset becomes its default plus 30°

#### Scenario: Gradual change
- **WHEN** a joint's offset is changed by 20° in Calibrate
- **THEN** the joint takes at least a third of a second to reach the new angle

### Requirement: Calibration report and confirmation
The robot SHALL report the complete table in effect after every accepted or rejected change, and at least once per second while in Calibrate, on every link it has. A front-end SHALL treat a change as done only when a report confirms it, and SHALL resend a change that is not confirmed within 200 ms. A front-end SHALL show the values from the latest report, not the values requested. It SHALL show a coxa's offset from the servo centre, the stored offset minus 90° (which puts the coxa's IK angle of 0° at mid-travel), labelled as such; the table, the link and the export SHALL carry the stored offset.

#### Scenario: Lost change frame
- **WHEN** a calibration change frame is lost on the link
- **THEN** the front-end resends it, and shows the new value only once the robot reports it

#### Scenario: Clamped value shown
- **WHEN** an offset 45° above the default is requested and the robot clamps it
- **THEN** the front-end shows the clamped value, the default plus 30°

#### Scenario: Coxa offset shown from centre
- **WHEN** the robot reports a coxa offset of 93°
- **THEN** the front-end shows +3.00° from centre, and its export writes 93.00

### Requirement: Compiled defaults and export
The compiled defaults SHALL live in one source file with a fixed, documented format. A front-end's export SHALL produce the current reported table as the complete text of that file, so that replacing the file with it and rebuilding gives a robot that starts with exactly that table. Every exporter's tests SHALL check, byte for byte, the export of a fixed fixture table against a golden fixture file that does not change with calibration. The committed defaults file SHALL be free to hold any calibration; host tests SHALL check that it parses as 18 joints in output order, within the guards' hard limits, with every joint's servo angle at the calibration pose (its IK angle there plus its offset) within 0–180°, and that exporting what it holds reproduces it exactly.

#### Scenario: Round trip
- **WHEN** the exported text replaces the compiled defaults and the firmware is rebuilt and flashed
- **THEN** after start-up the robot reports the same table as before the export

#### Scenario: Calibration export keeps the tests passing
- **WHEN** a changed table is exported over a copy of the compiled defaults
- **THEN** the copy passes the defaults checks, and the format checks against the golden fixture still pass unchanged
