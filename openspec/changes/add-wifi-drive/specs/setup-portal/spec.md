# Spec Delta

## MODIFIED Requirements

### Requirement: Entering and leaving setup mode
The bridge SHALL enter setup mode when the left button (GPIO0) is held for at least 1 second while the boot logo is shown, by restarting into setup mode, in Bluetooth drive and in WiFi drive alike. It SHALL leave setup mode only by restarting into the drive mode that the drive mode switch selects, either from the portal's exit control or from a reset. A normal power-up SHALL always start in the drive mode that the switch selects.

#### Scenario: Enter setup mode
- **WHEN** the left button is held for 1 second during the boot logo
- **THEN** the bridge restarts and comes up in setup mode

#### Scenario: Enter setup mode from WiFi drive
- **WHEN** the drive mode switch is closed and the left button is held for 1 second during the boot logo
- **THEN** the bridge restarts and comes up in setup mode, with the setup network and not the WiFi drive network

#### Scenario: Power-up after setup mode
- **WHEN** the bridge is reset or powered up after being in setup mode
- **THEN** it starts in the drive mode that the switch selects

### Requirement: One radio at a time
The bridge SHALL NOT run WiFi and Bluetooth together. In setup mode and in WiFi drive it SHALL NOT start Bluetooth. In Bluetooth drive it SHALL NOT start WiFi.

#### Scenario: No gamepad in setup mode
- **WHEN** the bridge is in setup mode
- **THEN** no gamepad can connect, and the WiFi access point is up

#### Scenario: No gamepad in WiFi drive
- **WHEN** the bridge is in WiFi drive
- **THEN** no gamepad can connect, and the WiFi drive access point is up

#### Scenario: No WiFi in Bluetooth drive
- **WHEN** the bridge is in Bluetooth drive
- **THEN** it runs no access point
