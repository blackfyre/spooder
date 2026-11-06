# Spec Delta

## MODIFIED Requirements

### Requirement: Telemetry frames
The robot SHALL send a telemetry frame at least every 100 ms. It SHALL contain:
- body mode and gait;
- whether servo output is on (in bench simulation, the modelled output, with the simulation flag set);
- latched faults;
- supply voltage and current (in bench simulation, the modelled readings);
- link statistics (valid and discarded frames received);
- reserved fields for sensor readings, each marked as unavailable until a sensor fills it.

#### Scenario: Telemetry reflects a fault
- **WHEN** an overcurrent fault latches on the robot
- **THEN** the next telemetry frame reports the overcurrent fault and the measured current

#### Scenario: Telemetry in bench simulation
- **WHEN** the robot walks in bench simulation
- **THEN** each telemetry frame sets the simulation flag and reports the modelled output as on and the modelled voltage and current
