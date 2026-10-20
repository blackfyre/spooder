# Spec Delta

## Purpose

Carries movement requests from the gamepad bridge to the robot and status back from the robot to the bridge over a wired serial link. It also guarantees that link faults lead to safe behaviour.

## ADDED Requirements

### Requirement: Frame integrity
Every frame on the link SHALL carry a start marker, a protocol version, a frame type, a sequence number, a payload length and a CRC over the rest of the frame. A receiver SHALL discard any frame with a bad CRC, an unknown version or type, or an inconsistent length, and SHALL resynchronise on the next start marker. Discarded frames SHALL be counted.

#### Scenario: Corrupted frame
- **WHEN** a command frame arrives with one corrupted byte
- **THEN** the robot discards it, increments its bad-frame count, and accepts the next valid frame

#### Scenario: Garbage between frames
- **WHEN** random bytes arrive between two valid frames
- **THEN** both valid frames are accepted

### Requirement: Command frames
The bridge SHALL send a command frame at least every 20 ms while it runs. Each frame SHALL carry the complete desired state: requested mode, translate vector, rotate amount, body height, speed, gait, stride mode, whether a controller is connected, the emergency-stop state, a fault-clear counter and a mode-request counter that the bridge increments on every explicit mode request. It SHALL carry no events. The robot SHALL apply only the newest valid frame, and SHALL stamp it with its own arrival time as the command's issue time.

#### Scenario: Lost frame loses nothing
- **WHEN** one command frame is lost and the next arrives
- **THEN** the robot ends up in the same state as if both had arrived

#### Scenario: Bridge goes silent
- **WHEN** no valid command frame arrives for more than 1 second
- **THEN** the robot treats its last command as stale and goes to Sleep

### Requirement: Fault clearing over the link
A fault-clear request SHALL be signalled by the bridge incrementing the fault-clear counter. The robot SHALL issue one clear request for each change of the counter between consecutive valid frames. A repeated value SHALL NOT clear anything. The first valid frame after power-up, or after 200 ms or more without a valid frame, SHALL only set the baseline for the fault-clear and mode-request counters: a restarted bridge, whose counters start again, SHALL NOT clear faults or count as a new mode request.

#### Scenario: Repeated frames carry the same counter
- **WHEN** many frames arrive with an unchanged counter
- **THEN** no clear request is issued

#### Scenario: Counter increments
- **WHEN** a frame arrives with a counter value one higher than the last one seen
- **THEN** exactly one clear request is issued

#### Scenario: Bridge restarts
- **WHEN** the bridge restarts after an emergency stop, and its first frames after the gap carry reset counters and no emergency stop
- **THEN** no clear request is issued, the emergency-stop fault stays latched and the body stays asleep

### Requirement: Resuming after a fault
After a fault is cleared, the robot SHALL keep the body asleep until a command frame arrives with a mode-request counter different from the one in effect when the fault latched. A frame that only repeats the previous desired mode SHALL NOT restart motion.

#### Scenario: Clear while the bridge still requests Walk
- **WHEN** a fault latched while walking is cleared, and the bridge keeps sending Walk with an unchanged mode-request counter
- **THEN** the body stays asleep

#### Scenario: New request after clearing
- **WHEN** the operator toggles the mode after clearing, so the mode-request counter changes
- **THEN** the body wakes through Stand and follows the new request

### Requirement: Emergency stop
When a command frame signals emergency stop, the robot SHALL inhibit servo output within one 20 ms period and latch an emergency-stop fault. The fault SHALL clear only through a fault-clear request while emergency stop is no longer signalled.

#### Scenario: Emergency stop pressed
- **WHEN** the bridge signals emergency stop
- **THEN** servo output is switched off within 20 ms and the emergency-stop fault is latched

#### Scenario: Release does not restart
- **WHEN** emergency stop is released and no fault-clear request is made
- **THEN** servo output stays off

### Requirement: Telemetry frames
The robot SHALL send a telemetry frame at least every 100 ms. It SHALL contain:
- body mode and gait;
- whether servo output is on;
- latched faults;
- supply voltage and current;
- link statistics (valid and discarded frames received);
- reserved fields for sensor readings, each marked as unavailable until a sensor fills it.

#### Scenario: Telemetry reflects a fault
- **WHEN** an overcurrent fault latches on the robot
- **THEN** the next telemetry frame reports the overcurrent fault and the measured current

### Requirement: Physical link
The link SHALL be a 3.3 V UART at 8 data bits, no parity and 1 stop bit, at a fixed baud rate shared by both firmwares. On the robot it SHALL use UART1 on GPIO20 (TX) and GPIO21 (RX), with the RX pull-up enabled. On the bridge it SHALL use GPIO27 (TX) and GPIO26 (RX). The two boards SHALL share a ground through the robot's logic ground.

#### Scenario: Bridge unplugged
- **WHEN** the bridge is disconnected from the robot
- **THEN** the robot's RX line idles high, no frames are accepted, and the robot goes to Sleep after 1 second
