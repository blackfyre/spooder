# Spec Delta

## Purpose

Shows the operator the state of the controller, the link and the robot on the bridge's screen, so that faults and unsafe conditions are visible without a computer.

## ADDED Requirements

### Requirement: Status content
The bridge screen SHALL show:
- whether a gamepad is connected;
- the link state (receiving telemetry, or lost);
- the robot's body mode and gait;
- whether servo output is on;
- supply voltage and current;
- the requested height, speed and stride mode;
- every latched fault by name;
- whether emergency stop is active.

Sensor readings SHALL be shown once telemetry reports them as available.

#### Scenario: Fault on the robot
- **WHEN** telemetry reports a latched undervoltage fault
- **THEN** the screen shows the undervoltage fault and the measured voltage

### Requirement: Link loss shown
If no valid telemetry frame has arrived for 1 second, the screen SHALL show that the robot link is lost, and SHALL stop showing robot values as current.

#### Scenario: Robot unplugged
- **WHEN** telemetry stops arriving
- **THEN** within about 1 second the screen shows the robot link as lost

### Requirement: Display does not delay control
Screen updates SHALL NOT delay command frames beyond their 20 ms interval. The screen SHALL refresh at least twice a second, and redraw only what changed.

#### Scenario: Busy screen
- **WHEN** many status values change at once
- **THEN** command frames continue to leave at least every 20 ms
