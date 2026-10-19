## ADDED Requirements

### Requirement: Gesture requests
Command frames SHALL carry a gesture id and a gesture-request counter, and telemetry SHALL report the gesture being played. Each change of the counter with a gesture id SHALL be one request; an unchanged counter SHALL request nothing, so that a held button or a lost frame neither repeats nor loses a request. After a link gap, the first frame SHALL only set the counter's baseline. The link protocol version SHALL become 3.

#### Scenario: Held button plays once
- **WHEN** the bridge sends the same gesture id and counter in consecutive frames
- **THEN** the robot plays the gesture once

#### Scenario: Version mismatch fails safe
- **WHEN** one board speaks protocol 2 and the other protocol 3
- **THEN** each side discards the other's frames and the robot sleeps
