## MODIFIED Requirements

### Requirement: Start-up sequence
At power-up the screen SHALL show the Spooder logo, then a start-up log of the bridge's set-up steps, for no more than 3 seconds in total, and then the status screen. The start-up log SHALL begin with the bridge's build identity (see the build-identity capability). Command frames SHALL be sent throughout. If emergency stop becomes active during the sequence, the status screen SHALL be shown at once.

#### Scenario: Normal power-up
- **WHEN** the bridge powers up
- **THEN** the logo is shown, then the start-up log, and the status screen appears within 3 seconds

#### Scenario: Build shown at start-up
- **WHEN** the start-up log is shown
- **THEN** its first line names the bridge's build identity

#### Scenario: Emergency stop during start-up
- **WHEN** the GPIO35 button is pressed while the logo is shown
- **THEN** the status screen is shown at once, with emergency stop active
