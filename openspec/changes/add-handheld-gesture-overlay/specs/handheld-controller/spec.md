# Spec Delta

## MODIFIED Requirements

### Requirement: Drive screen
The handheld SHALL show one live screen, with two pages switched by Select + X. The panels SHALL show, from the newest telemetry: the robot's body mode, gait, whether servo output is on, the faults by name, the gesture being played, the supply voltage and the current; from its own state: the requested mode, height, speed, gait, stride mode, stick mode and emergency stop. The driving view SHALL show the robot from a camera fixed behind it, looking forward and 45° down, in perspective, over a ground grid at the lowest foot, its legs placed from the newest pose frame (a leg never set since the robot started marked at its pivot, joints near or at their servo limits marked), with the body mode, gait, output, supply voltage and faults from the newest telemetry. Both pages SHALL show the emergency stop and the link's state, and, while the newest telemetry reports bench simulation (bench-simulation: the simulation flag), that the robot is simulated, with its output shown as simulated rather than on. It SHALL show each frame type's values from the frame of that type with the newest sequence number, not replacing them with an older one, except that a type whose last shown frame was taken more than 500 ms ago SHALL be taken again from its next frame. It SHALL redraw at least 10 times a second. When no telemetry has arrived for 500 ms it SHALL mark the robot's values as stale, and when no pose frame has arrived for 500 ms the driving view as stale. While L2 is held beyond half travel, the handheld SHALL show a gesture overlay over either page, and SHALL close it when L2 is released; the overlay SHALL change nothing that is sent. It SHALL hold one tile for each gesture of the L2 layer, placed as their buttons are on the handheld (A, B, X and Y as a diamond, R2 beside it), each with the gesture's name and its button. A tile of a gesture the robot has a script for SHALL loop an animation of the robot playing that gesture, drawn from the joint angles the robot's own script produces; a tile of a gesture without a script SHALL show the standing robot muted and marked as not available yet. A gesture request SHALL light its tile for a moment; while the newest telemetry, not stale, reports a gesture playing, that gesture's tile SHALL stay lit. While the robot cannot start a gesture, because its newest telemetry is missing or stale or reports a body mode other than Stand or Perform, the overlay SHALL say that the robot must stand first. Which gestures have a script, and their animations, SHALL be kept equal to the robot's scripts by a check that fails when either changes without the other.

#### Scenario: Bench simulation shown
- **WHEN** telemetry arrives with the bench simulation flag set and output on
- **THEN** both pages show that the robot is simulated, and the output reads as simulated; after telemetry without the flag, neither does

#### Scenario: Fault by name
- **WHEN** telemetry reports the overcurrent fault
- **THEN** the screen names the overcurrent fault

#### Scenario: Driving view from the pose
- **WHEN** a pose frame arrives with leg 1's joints set and the other legs never set, and the driving view is shown
- **THEN** leg 1 is drawn from its pose and the others are marked at their pivots; 500 ms later, with no further pose frame, the driving view is marked stale

#### Scenario: Older telemetry after a newer frame
- **WHEN** telemetry 10 is shown, then diagnostics 14, telemetry 13 and telemetry 9 arrive
- **THEN** the screen shows telemetry 13's values, not telemetry 9's

#### Scenario: One type lost while the others arrive
- **WHEN** telemetry 10 is shown, then no telemetry arrives for 600 ms while pose and diagnostics keep arriving, and telemetry 140 then arrives
- **THEN** the screen shows telemetry 140's values

#### Scenario: Link lost
- **WHEN** no robot frame has arrived for 500 ms
- **THEN** the robot's values are marked stale and the time since the last frame keeps rising

#### Scenario: Overlay held open
- **WHEN** L2 is pressed beyond half travel on either page, then released
- **THEN** the overlay shows the five gesture tiles with their names and buttons while L2 is held, closes on release, and the command frames sent are the same as without the overlay

#### Scenario: Gesture without a script
- **WHEN** the overlay is open and the robot has scripts for say hi and wiggle only
- **THEN** the say-hi and wiggle tiles animate, and the dance, attack and ball tiles show the still standing robot marked as not available yet

#### Scenario: Request lights its tile
- **WHEN** the overlay is open and A is pressed
- **THEN** the say-hi tile is lit for a moment, and the command frame carries the say-hi request as it does without the overlay

#### Scenario: Playing gesture stays lit
- **WHEN** the overlay is open and fresh telemetry reports wiggle playing
- **THEN** the wiggle tile stays lit until telemetry no longer reports it or turns stale

#### Scenario: Stand first
- **WHEN** the overlay is open and the newest telemetry reports Sleep or Walk, or no telemetry has arrived for 500 ms
- **THEN** the overlay says that the robot must stand first; with fresh telemetry reporting Stand or Perform, it does not

#### Scenario: Scripts and animations in step
- **WHEN** a gesture's script changes in the robot's movement code, or one gains a script, without the handheld's gesture data being regenerated
- **THEN** a host test fails
