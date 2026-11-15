## ADDED Requirements

### Requirement: Stopped body shown
While the latest telemetry reports the body stopped, the app SHALL show the body as `Stopped` wherever it shows the body mode (the header's mode cell and the charts' timeline of the body mode), in place of the last body mode and gesture. Once telemetry reports the body running, it SHALL show the body mode again. The frames pane, a raw field dump, SHALL keep showing the frame's fields as received.

#### Scenario: Waiting after a fault
- **WHEN** an emergency stop stopped the body while it walked, and it has been cleared without a new mode request
- **THEN** the header's mode cell shows `Stopped`, not `Walk`

#### Scenario: Body restarts
- **WHEN** a new mode request restarts the body
- **THEN** the header's mode cell shows the body mode again

#### Scenario: Charts while stopped
- **WHEN** telemetry samples with body mode Walk report the body stopped
- **THEN** the charts' timeline of the body mode shows those samples as `Stopped`, not `Walk`
