## ADDED Requirements

### Requirement: Bridge link state in telemetry
Every telemetry frame, on both links, SHALL carry the bridge link's state: whether a valid bridge command frame (the bridge's 20 ms heartbeat, in both of its modes) arrived within the link-gap time, whether the newest bridge frame reports a connected gamepad, and the bridge link's valid and discarded frame counts. The fields SHALL use bytes reserved in protocol version 3, so that front-ends and robots of either age interoperate.

#### Scenario: Bridge without a gamepad
- **WHEN** the bridge is running with no gamepad paired and the robot is read on USB
- **THEN** telemetry reports the bridge link up, no gamepad, and rising valid counts

#### Scenario: Bridge unplugged
- **WHEN** the bridge is unplugged
- **THEN** within the link-gap time telemetry reports the bridge link down, and the counts stop rising
