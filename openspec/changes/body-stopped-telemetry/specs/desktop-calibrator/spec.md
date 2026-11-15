## ADDED Requirements

### Requirement: Stopped body shown
While the latest telemetry reports the body stopped, the app SHALL show the body as `Stopped` wherever it shows the body mode (the header's mode cell, the charts' mode lane and the decoded telemetry line in the log), in place of the last body mode and gesture. Once telemetry reports the body running, it SHALL show the body mode again.

#### Scenario: Waiting after a fault
- **WHEN** an emergency stop stopped the body while it walked, and it has been cleared without a new mode request
- **THEN** the header's mode cell shows `Stopped`, not `Walk`

#### Scenario: Body restarts
- **WHEN** a new mode request restarts the body
- **THEN** the header's mode cell shows the body mode again
