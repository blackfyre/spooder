## ADDED Requirements

### Requirement: Held extremes on the charts
The current chart SHALL mark the highest current reported recently, and the supply chart the lowest voltage, each as a named line in its series' colour with the held value. A held extreme SHALL move at once to a reading at or beyond it, stay for 3 seconds after it was last reached, then move back towards the reported value at 2 A per second for the current and 0.5 V per second for the voltage, until it meets it, as on the bridge's drive page. With no telemetry on the connection, or none for longer than the gap between samples, no held extreme SHALL be shown.

#### Scenario: Stand surge
- **WHEN** the current reads 4.3 A once and 0.4 A from then on
- **THEN** the current chart's held line shows 4.3 A for 3 seconds, 2.3 A one second later, and then follows the current at 0.4 A

#### Scenario: Supply sag
- **WHEN** the voltage dips to 5.5 V and returns to 6.0 V
- **THEN** the supply chart's held line shows 5.5 V for 3 seconds, then rises to 6.0 V within a further second
