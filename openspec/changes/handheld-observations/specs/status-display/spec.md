# Spec Delta

## ADDED Requirements

### Requirement: Supply bands
The bridge SHALL show the supply voltage and current values on the drive page, and the newest values and the window's lowest voltage and highest current on the power page, in a colour for their band, from the protection limits the robot reports in its newest diagnostics frame: the fault colour at or past the limit at which protection faults (at or below the undervoltage voltage or at or above the overvoltage voltage; at or above the trip current), the warning colour within the warning band (within the gap between the brown-out and undervoltage voltages of either voltage limit; at or above the sustained current), and the ok colour otherwise. The value SHALL stay shown as the label. While no limits have been reported, or they are out of order, the values SHALL keep the colours they have without bands. On the drive page, a value in the ok band SHALL be shown in the warning colour while its held extreme warns (held extremes on the drive bars). The band rule SHALL be the handheld's, kept equal by a check that both implementations pass over one set of cases.

#### Scenario: Voltage in the warning band
- **WHEN** the newest diagnostics report a brown-out of 4.8 V, an undervoltage of 5.2 V and an overvoltage of 6.8 V, and telemetry reports 5.4 V
- **THEN** the drive page shows 5.40 V in the warning colour

#### Scenario: Current at the trip level
- **WHEN** the newest diagnostics report a sustained current of 6 A and a trip current of 9 A, and telemetry reports 9.0 A
- **THEN** the drive page shows the current in the fault colour

#### Scenario: No limits yet
- **WHEN** telemetry reports 6.0 V and no diagnostics frame has arrived since the bridge started
- **THEN** the drive page shows 6.00 V in the colour it has without bands

#### Scenario: Band rule shared
- **WHEN** the band rule changes in the bridge's code or the handheld's without the other
- **THEN** a host test fails
