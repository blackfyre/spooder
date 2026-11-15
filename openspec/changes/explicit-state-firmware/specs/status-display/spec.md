## ADDED Requirements

### Requirement: The emergency stop shown is the one the command frames carry

The emergency stop the bridge screen shows SHALL be the emergency stop the bridge's command frames carry, computed by the same rule: in Bluetooth drive and setup mode the GPIO35 toggle; in WiFi drive the GPIO35 toggle or the handheld's emergency-stop flag. The screen SHALL take it from the same snapshot of the link state as the telemetry it shows with it, never by combining values read at different times.

#### Scenario: Handheld emergency stop in WiFi drive

- **WHEN** in WiFi drive the handheld's accepted frames carry the emergency-stop flag while the GPIO35 toggle is off
- **THEN** the command frames carry the emergency stop and the status strip shows the emergency stop

#### Scenario: Handheld flag kept through a lapse

- **WHEN** in WiFi drive the handheld's liveness lapses after a frame that carried the emergency-stop flag
- **THEN** the command frames keep carrying the emergency stop and the status strip keeps showing it until an accepted frame clears the flag

#### Scenario: GPIO35 toggle in Bluetooth drive

- **WHEN** in Bluetooth drive the GPIO35 button is pressed once
- **THEN** the command frames carry the emergency stop and the status strip shows it, as before this change
