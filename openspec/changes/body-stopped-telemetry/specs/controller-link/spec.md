## ADDED Requirements

### Requirement: Body run state in telemetry
Every telemetry frame, on both links, SHALL carry a body-stopped flag (bit 6 of the telemetry flags), set while the robot's body is not running and clear while it runs. The body is not running from power-up until it first starts, and from the tick a fault that inhibits servo output stops it until it restarts through the boot sequence. While the flag is set, the reported body mode and gait are the body's last ones and SHALL NOT be taken as live, and the reported gesture SHALL be none. A fault that only forces Sleep SHALL leave the flag clear, since the body keeps running. The flag SHALL NOT change the telemetry frame's layout. Link protocol version 4 SHALL be the first version with the flag; every frame SHALL carry version 4, and a receiver SHALL discard frames of any other version, so a robot and a front-end of different versions fail safe as a lost link.

#### Scenario: Before the first start
- **WHEN** the robot has powered up and no request has started the body yet
- **THEN** every telemetry frame sets the body-stopped flag

#### Scenario: Fault stops the body
- **WHEN** the body is walking and an emergency stop inhibits servo output
- **THEN** the next telemetry frame sets the body-stopped flag and reports no gesture, and the flag stays set after the fault is cleared while no new mode request has been made

#### Scenario: Restart clears the flag
- **WHEN** a stopped body restarts after the fault is cleared and a new mode request is made
- **THEN** the first telemetry frame after the restart has the body-stopped flag clear

#### Scenario: Sleep-only fault
- **WHEN** a fault that only forces Sleep latches while servo output stays allowed
- **THEN** telemetry keeps the body-stopped flag clear and reports the body's Sleep transition

#### Scenario: Mismatched versions
- **WHEN** a robot with protocol version 4 is connected to a bridge with protocol version 3
- **THEN** each side discards the other's frames, the bridge shows the link as lost, and the robot goes to Sleep after 1 second
