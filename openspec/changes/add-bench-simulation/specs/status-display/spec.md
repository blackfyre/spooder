# Spec Delta

## ADDED Requirements

### Requirement: Simulation shown
While fresh telemetry has the simulation flag set, the bridge screen SHALL name simulation (`SIM`) in the status strip on every page, in the warning tone, and the drive page's output field SHALL mark the output as simulated. The status strip SHALL name the simulation supply fault when telemetry reports it. Without the flag, the pages SHALL be as before.

#### Scenario: Robot in simulation
- **WHEN** fresh telemetry has the simulation flag set and reports output on
- **THEN** the status strip shows SIM on whichever page is shown, and the drive page shows the output as simulated rather than plainly on

#### Scenario: Supply appears in simulation
- **WHEN** telemetry reports the simulation supply fault
- **THEN** the status strip names the fault in the fault tone, beside SIM
