## MODIFIED Requirements

### Requirement: Link page
The link page SHALL show, while the link is up: the frames the robot's decoder accepted and rejected per second on the bridge link, the frames the bridge's decoder accepted and rejected per second, the age of the newest telemetry, and which source the robot follows (the bridge, the USB link in control, or a conflict). It SHALL also show whether a gamepad is connected and its battery level when reported. The rates SHALL be computed over about one second.

#### Scenario: Healthy link
- **WHEN** the robot accepts every command frame and sends telemetry, pose and diagnostics at 10 Hz each on the bridge link
- **THEN** the link page shows about 50 frames per second accepted by the robot and about 30 accepted by the bridge, none rejected
