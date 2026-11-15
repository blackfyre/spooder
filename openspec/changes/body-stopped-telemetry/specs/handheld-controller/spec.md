## ADDED Requirements

### Requirement: Stopped body on the handheld
While fresh telemetry reports the body stopped, the handheld's screen SHALL show the body as `Stopped` in place of its last body mode, and SHALL treat the body as neither standing nor performing. When it waits to leave safely, a telemetry frame that reports the body stopped SHALL count towards the confirmation as one that reports the body in Sleep or servo output off does.

#### Scenario: Stopped body shown
- **WHEN** a fault stopped the body while it walked and the fault has been cleared without a new mode request
- **THEN** the handheld's screen shows the body as `Stopped`, not `Walk`

#### Scenario: Leaving while stopped
- **WHEN** Menu is pressed and the next two newer telemetry frames report the body stopped
- **THEN** the leave is confirmed and the handheld exits without waiting the full 3 s

#### Scenario: No gestures while stopped
- **WHEN** fresh telemetry reports the body stopped with Stand or Perform as its last mode
- **THEN** the handheld treats gestures as unavailable, as for any body neither standing nor performing
