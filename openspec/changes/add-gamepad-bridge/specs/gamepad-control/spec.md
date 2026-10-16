# Spec Delta

## Purpose

Defines how a Bluetooth gamepad connected to the bridge controls the robot: which inputs request which modes and settings, and what happens when the gamepad connects or disconnects.

## ADDED Requirements

### Requirement: Supported controllers
The bridge SHALL accept one gamepad at a time through Bluepad32, including Bluetooth Classic gamepads such as DualShock 4, DualSense, Switch Pro and 8BitDo pads. A second gamepad SHALL be refused while one is connected.

#### Scenario: Second gamepad
- **WHEN** a gamepad is connected and another tries to connect
- **THEN** the second gamepad is refused and the first keeps control

### Requirement: Input mapping
The bridge SHALL map gamepad input to the desired movement state as follows. Button names are Bluepad32's.

| Input | Effect |
|---|---|
| Left stick | Translate vector, scaled to −100..100 per axis, magnitude clamped to 100 |
| Right stick X | Rotate amount, −100..100; right stick Y unused |
| `miscHome` (press) | Toggle requested mode between Stand and Walk |
| `y` (press) | Toggle Sleep. Leaving Sleep returns to the mode requested before it |
| `miscBack` + `miscHome` held 1 s | Toggle Calibrate. Leaving it returns to Stand |
| `r1` / `l1` (press) | Next / previous gait, wrapping around |
| D-pad up / down (press) | Body height +10 / −10, within 0..100 |
| D-pad right / left (press) | Speed +10 / −10, within 0..100 |
| `x` (press) | Toggle stride mode between dynamic and fixed |
| `thumbL` + `thumbR` held 1 s | Fault-clear request |

Toggles and steps SHALL act once per press. Holding a button SHALL NOT repeat it.

#### Scenario: Gait selection wraps
- **WHEN** the last gait is selected and `r1` is pressed
- **THEN** the first gait is selected

#### Scenario: Calibrate needs a deliberate hold
- **WHEN** `miscBack` and `miscHome` are pressed together and released after 0.5 s
- **THEN** Calibrate is not toggled

### Requirement: Stick dead zone
Stick deflections within a radial dead zone of 8 % of full scale SHALL be reported as zero. Outside the dead zone, output SHALL rise continuously from zero, with no jump at the edge.

#### Scenario: Drifting stick
- **WHEN** the left stick rests at 5 % deflection
- **THEN** the translate vector is zero

#### Scenario: Continuous beyond the dead zone
- **WHEN** the left stick moves from 7 % to 10 % deflection
- **THEN** the translate magnitude rises from zero without a step

### Requirement: Controller connection
While no gamepad is connected, the bridge SHALL request Sleep with zero translate and rotate, and report that no controller is connected. When a gamepad connects, the requested mode SHALL start as Stand, and the height, speed, gait and stride settings SHALL be kept from before.

#### Scenario: Gamepad disconnects while walking
- **WHEN** the gamepad disconnects while the robot is walking
- **THEN** the bridge's next command frame requests Sleep with zero input

#### Scenario: Reconnect
- **WHEN** a gamepad reconnects
- **THEN** the requested mode is Stand and the previous height, speed and gait are kept

### Requirement: Bridge emergency-stop button
The bridge's GPIO35 button SHALL toggle emergency stop. While emergency stop is active, the bridge SHALL signal it in every command frame and show it on the screen.

#### Scenario: Emergency stop from the bridge
- **WHEN** the GPIO35 button is pressed
- **THEN** every subsequent command frame signals emergency stop until the button is pressed again
