# diagnostics-telemetry Specification

## Purpose
Lets a computer on the robot's USB port follow the robot's internals over time: how much the joint speed budget slows the motion, the output rate backstop, the control loop's working time, the averaged current and the configured limits.

## Requirements

### Requirement: Diagnostics frame contents
The robot SHALL report, for each window of five control ticks (100 ms):
- the smallest motion share the joint speed budget allowed in that window, in thousandths (1000 = full rate);
- the output rate backstop's count of ticks that held a joint back, since start, wrapping at 16 bits;
- the longest and the mean working time of a control tick in the window, in microseconds;
- the protection's averaged current over its sustained window, in milliamperes.

It SHALL also report the configured limits: the trip and sustained currents in milliamperes; the minimum, maximum and brown-out voltages in millivolts; and the servo profile's joint speed budget and rated speed in degrees per second.

#### Scenario: Governed walking
- **WHEN** the robot walks fast enough that the budget slows its motion
- **THEN** the diagnostics report a motion share below 1000, and the budget and rated speed of the build's servo profile

#### Scenario: Idle robot
- **WHEN** the robot stands still
- **THEN** the diagnostics report a motion share of 1000, and the control tick's working time

### Requirement: Diagnostics frame on USB only
The robot SHALL send the diagnostics frame on its USB link only, once per telemetry period (about ten times per second), whole or not at all, under the same rules as other USB frames, and on a different tick from the telemetry and pose frames. It SHALL NOT send diagnostics frames on the bridge link.

#### Scenario: Bridge link unaffected
- **WHEN** the robot runs with the bridge connected and a host on USB
- **THEN** the host receives diagnostics frames at about 10 Hz and the bridge receives none

### Requirement: Diagnostics frame in the link protocol
The diagnostics frame SHALL be an additional frame type of link protocol version 3, without changing the protocol version or any other frame. Golden vectors for it SHALL be part of the protocol's golden frame file, and every implementation SHALL decode and re-encode them byte for byte. A receiver that does not expect diagnostics frames SHALL ignore them.

#### Scenario: Bridge firmware unchanged
- **WHEN** a bridge built before this change is connected to a robot with it
- **THEN** the link works as before
