# Spec Delta

## MODIFIED Requirements

### Requirement: Drive screen
The handheld SHALL show one live screen, with two pages switched by Select + X. The panels SHALL show, from the newest telemetry: the robot's body mode, gait, whether servo output is on, the faults by name, the gesture being played, the supply voltage and the current; from its own state: the requested mode, height, speed, gait, stride mode, stick mode and emergency stop. Beside each requested value's label it SHALL name the controls that change it, and each SHALL show as pressed while its control is held and the press reaches that value (not while Select or L2 turns it to another function), as the controls along the bottom show their presses. The supply voltage and the current SHALL be shown in a colour for their band, from the protection limits the robot reports in its newest diagnostics frame: the fault colour at or past the limit at which protection faults (at or below the undervoltage voltage or at or above the overvoltage voltage; at or above the trip current), the warning colour within the warning band (within the gap between the brown-out and undervoltage voltages of either voltage limit; at or above the sustained current), and the ok colour otherwise; the value SHALL stay shown as the label, the colour SHALL be the screen's normal value colour while no limits have been reported or they are out of order, and muted while the values are stale. The driving view SHALL show the robot from a camera fixed behind it, looking forward and 45° down, in perspective, over a ground grid at the lowest foot, its legs placed from the newest pose frame (a leg never set since the robot started marked at its pivot, joints near or at their servo limits marked), with the body mode, the gesture being played, gait, output, supply voltage and faults from the newest telemetry. Both pages SHALL show the emergency stop and the link's state, and, while the newest telemetry reports bench simulation (bench-simulation: the simulation flag), that the robot is simulated, with its output shown as simulated rather than on. It SHALL show each frame type's values from the frame of that type with the newest sequence number, not replacing them with an older one, except that a type whose last shown frame was taken more than 500 ms ago SHALL be taken again from its next frame. It SHALL redraw at least 10 times a second. When no telemetry has arrived for 500 ms it SHALL mark the robot's values as stale, and when no pose frame has arrived for 500 ms the driving view as stale. While L2 is held beyond half travel, the handheld SHALL show a gesture overlay over either page, and SHALL close it when L2 is released; the overlay SHALL change nothing that is sent. It SHALL hold one tile for each gesture of the L2 layer, placed as their buttons are on the handheld (A, B, X and Y as a diamond, R2 beside it), each with the gesture's name and its button. A tile of a gesture the robot has a script for SHALL loop an animation of the robot playing that gesture, drawn from the joint angles the robot's own script produces; a tile of a gesture without a script SHALL show the standing robot muted and marked as not available yet. A gesture request SHALL light its tile for a moment; while the newest telemetry, not stale, reports a gesture playing, that gesture's tile SHALL stay lit. While the robot cannot start a gesture, because its newest telemetry is missing or stale or reports a body mode other than Stand or Perform, the overlay SHALL say that the robot must stand first. Which gestures have a script, and their animations, SHALL be kept equal to the robot's scripts by a check that fails when either changes without the other.

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

#### Scenario: Driving view names the gesture
- **WHEN** the driving view is shown and fresh telemetry reports Perform with say hi playing
- **THEN** the driving view names say hi beside the body mode; with telemetry reporting no gesture, it reads none

#### Scenario: Hint key pressed
- **WHEN** the panels page is shown and the up button is held
- **THEN** the up key beside the requested height's label shows as pressed, and no other requested value's key does; with Select and Y held, the Y key beside the requested mode does not show as pressed

#### Scenario: Supply in its band
- **WHEN** the newest diagnostics report a brown-out of 4.8 V, an undervoltage of 5.2 V, an overvoltage of 6.8 V, a sustained current of 6 A and a trip current of 9 A, and fresh telemetry reports 5.4 V and 2 A
- **THEN** the voltage reads 5.40 V in the warning colour and the current 2.00 A in the ok colour; at 5.1 V and 9.5 A both read in the fault colour

#### Scenario: Supply without limits
- **WHEN** fresh telemetry reports 6.0 V and no diagnostics frame has arrived
- **THEN** the voltage reads 6.00 V in the normal value colour
