## ADDED Requirements

### Requirement: Diagnostics frame on both links
The robot SHALL send the diagnostics frame on its USB link and on the bridge link, once per telemetry period (about ten times per second) on each, whole or not at all, under the same rules as other frames on that link, and on a different tick from the telemetry and pose frames. Both links SHALL carry the same values for a window. It SHALL send them on the bridge link whatever mode the bridge is in. A frame refused by one link SHALL NOT stop it being sent on the other.

#### Scenario: Both links
- **WHEN** the robot runs with the bridge connected and a host on USB
- **THEN** the host and the bridge each receive diagnostics frames at about 10 Hz, on the same ticks, with the same values for each window

#### Scenario: One window per period
- **WHEN** the robot sends a diagnostics frame on both links
- **THEN** the window restarts once, after both, and each frame reports the five ticks before it

## MODIFIED Requirements

### Requirement: Diagnostics frame in the link protocol
The diagnostics frame SHALL be an additional frame type of link protocol version 3, without changing the protocol version or any other frame. Golden vectors for it SHALL be part of the protocol's golden frame file, and every implementation SHALL decode and re-encode them byte for byte. A receiver that does not expect diagnostics frames SHALL ignore them.

#### Scenario: Bridge firmware unchanged
- **WHEN** a bridge, built before or after this change, is connected to a robot that sends diagnostics frames on the bridge link
- **THEN** the bridge counts them as accepted frames, rejects none of them, and drives and shows status as before, in Bluetooth drive and in setup mode

## REMOVED Requirements

### Requirement: Diagnostics frame on USB only
**Reason**: Replaced by "Diagnostics frame on both links": the frame now goes to the bridge link as well.
**Migration**: None; receivers that do not use the frame already ignore it.
