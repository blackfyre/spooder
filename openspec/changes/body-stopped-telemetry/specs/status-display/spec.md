## ADDED Requirements

### Requirement: Stopped body shown
While fresh telemetry reports the body stopped, the bridge screen SHALL show the body as stopped wherever it shows the body mode, in place of the body mode, gait and gesture: `STOPPED` in the drive page's header, and `Stopped` in the setup screen's body mode cell, in the case of its other mode names. Once telemetry reports the body running, it SHALL show the body mode again.

#### Scenario: Waiting after a fault
- **WHEN** a fault stopped the body while it walked, the fault has been cleared, and the robot waits for a new mode request
- **THEN** the drive page's header shows `STOPPED`, not `WALK`

#### Scenario: Setup screen
- **WHEN** the bridge is in setup mode and telemetry reports the body stopped
- **THEN** the setup screen's body mode cell shows `Stopped`, not the last body mode

#### Scenario: Body restarts
- **WHEN** a new mode request restarts the body
- **THEN** the header shows the body mode and gait again from the first telemetry frame that reports the body running
