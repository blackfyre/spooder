## ADDED Requirements

### Requirement: Every bridge boot mode has exactly one boot path

The bridge firmware SHALL represent its boot mode with one type, the drive mode decided at boot (Bluetooth drive, WiFi drive, setup mode), from that decision through the robot link. Every branch on the boot mode in the bridge's sources SHALL name each mode explicitly, with no fallback that maps an unhandled or unknown mode to another one. A C mirror of the boot mode, where C code needs one, SHALL pin each of its values to the corresponding drive mode at compile time. `mise run build-bridge` SHALL fail when a drive mode is added without being handled in every such branch, or when a mirrored value no longer equals its drive mode.

#### Scenario: Drive mode without a boot path

- **WHEN** a drive mode is added to the boot decision and the robot link or `app_main` does not handle it
- **THEN** `mise run build-bridge` fails and names the unhandled mode

#### Scenario: Mirror value out of step

- **WHEN** a value of the C boot-mode mirror is changed so that it no longer equals its drive mode
- **THEN** `mise run build-bridge` fails

#### Scenario: Boot paths unchanged

- **WHEN** the bridge starts with the setup request, with the drive mode switch closed, or with it open
- **THEN** it enters setup mode, WiFi drive or Bluetooth drive respectively, as before this change
