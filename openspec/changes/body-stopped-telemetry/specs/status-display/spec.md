## ADDED Requirements

### Requirement: Stopped body shown
While fresh telemetry reports the body stopped, the bridge screen SHALL show `STOPPED` wherever it shows the body mode (the drive page's header and the setup screen), in place of the body mode, gait and gesture. Once telemetry reports the body running, it SHALL show the body mode again.

#### Scenario: Waiting after a fault
- **WHEN** a fault stopped the body while it walked, the fault has been cleared, and the robot waits for a new mode request
- **THEN** the drive page's header shows `STOPPED`, not `WALK`

#### Scenario: Setup screen
- **WHEN** the bridge is in setup mode and telemetry reports the body stopped
- **THEN** the setup screen's body mode cell shows `STOPPED`

#### Scenario: Body restarts
- **WHEN** a new mode request restarts the body
- **THEN** the header shows the body mode and gait again from the first telemetry frame that reports the body running
