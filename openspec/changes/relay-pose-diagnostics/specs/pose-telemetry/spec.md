## ADDED Requirements

### Requirement: Pose frame on both links
The robot SHALL send the pose frame on its USB link and on the bridge link, about ten times per second on each, whole or not at all, under the same rules as other frames on that link, and on a different tick from the telemetry and diagnostics frames. It SHALL send them on the bridge link whatever mode the bridge is in. A frame refused by one link SHALL NOT stop it being sent on the other.

#### Scenario: Both links
- **WHEN** the robot runs with the bridge connected and a host on USB
- **THEN** the host and the bridge each receive pose frames at about 10 Hz, on the same ticks, and neither link receives a pose frame on a telemetry tick

#### Scenario: Host not reading
- **WHEN** a host holds the USB port open without reading
- **THEN** pose frames that do not fit are dropped whole, the bridge link still receives every pose frame, and the control loop keeps its period

#### Scenario: Bridge link full
- **WHEN** the bridge link's transmit queue cannot hold a pose frame
- **THEN** that pose frame is dropped whole on the bridge link, the USB link still receives it, and the control loop keeps its period

## MODIFIED Requirements

### Requirement: Pose frame in the link protocol
The pose frame SHALL be an additional frame type of link protocol version 2, without changing the protocol version or any other frame. Golden vectors for it SHALL be part of the protocol's golden frame file, and every implementation SHALL decode and re-encode them byte for byte. A receiver that does not expect pose frames SHALL ignore them.

#### Scenario: Bridge firmware unchanged in behaviour
- **WHEN** a bridge, built before or after this change, is connected to a robot that sends pose frames on the bridge link
- **THEN** the bridge counts them as accepted frames, rejects none of them, and drives and shows status as before, in Bluetooth drive and in setup mode

## REMOVED Requirements

### Requirement: Pose frame on USB only
**Reason**: Replaced by "Pose frame on both links": the frame now goes to the bridge link as well.
**Migration**: None; receivers that do not use the frame already ignore it.
