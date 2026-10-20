## ADDED Requirements

### Requirement: Bridge link shown
The app SHALL show the bridge link's state from telemetry: down, or up with or without a gamepad, and its valid and discarded frame counts in every state.

#### Scenario: Confirming the bridge link from the computer
- **WHEN** the bridge runs without a gamepad and the app is connected to the robot
- **THEN** the app shows the bridge link up, with no gamepad, and counts that rise
