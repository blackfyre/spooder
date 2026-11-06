# Spec Delta

## ADDED Requirements

### Requirement: Simulation shown
While the latest telemetry has the simulation flag set, the app SHALL show a banner saying that the robot is in bench simulation and that no servo moves, and the servo-output value SHALL be marked as simulated. The app SHALL name the simulation supply fault like every other fault. Without the flag, the view SHALL be as before.

#### Scenario: Robot in simulation
- **WHEN** telemetry with the simulation flag set and output on arrives
- **THEN** the app shows the simulation banner and the output as simulated

#### Scenario: Supply appears in simulation
- **WHEN** telemetry reports the simulation supply fault
- **THEN** the app names the fault by name, not by bit number
