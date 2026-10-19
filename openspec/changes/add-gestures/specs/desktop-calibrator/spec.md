## ADDED Requirements

### Requirement: Gesture controls
While the app is in control, it SHALL offer a button for each gesture, also on keys 4 to 8 in the gesture order, sending one gesture request per press. The legs pane SHALL show the gesture as it plays, so that each gesture can be rehearsed on the virtual robot.

#### Scenario: Rehearsal on the virtual robot
- **WHEN** the app controls the virtual robot in Stand and the operator presses the wiggle button
- **THEN** the legs pane shows the wiggle and the telemetry reports it until the body is back in Stand
