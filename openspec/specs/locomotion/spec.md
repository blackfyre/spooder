# locomotion Specification

## Purpose
Defines how the hexapod body moves in response to movement commands: its modes, the transitions between them, gaits, smoothing and the safety behaviour when commands stop arriving.

## Requirements

### Requirement: Movement command interface
Movement SHALL accept commands as a single value containing a requested mode (Stand, Walk, Sleep or Calibrate), a translate vector, a rotate amount, a body height, a speed scale, a gait, a stride mode and the time the command was issued. Movement SHALL be the only component that changes body mode, gait or leg state; a command only expresses a request.

#### Scenario: Command expresses a request, movement decides
- **WHEN** a command requests Walk while a Stand transition is still in progress
- **THEN** movement completes the Stand transition before entering Walk

#### Scenario: Out-of-range parameters are clamped
- **WHEN** a command carries a translate or rotate magnitude above 100, a body height outside the supported range, or a speed scale outside 0–100
- **THEN** movement clamps each value to its range and does not reject the command

#### Scenario: Unknown gait is ignored
- **WHEN** a command names a gait that does not exist
- **THEN** movement keeps the current gait

### Requirement: Non-blocking, time-based movement
Movement SHALL advance by elapsed time in one bounded step per main-loop iteration and SHALL return control to the caller after each step. No mode or transition SHALL hold the main loop for longer than one step. The speed of every transition, gait cycle and smoothing filter SHALL depend on elapsed time, not on how often movement is stepped.

#### Scenario: Stand transition does not block
- **WHEN** the body enters Stand from another mode
- **THEN** the transition progresses over successive steps and the display, distance sensor and IMU continue to be serviced during it

#### Scenario: Tick-rate independence
- **WHEN** the same command sequence is applied with steps of 20 ms and with steps of 50 ms over the same total time
- **THEN** the resulting leg positions agree within 5 mm

#### Scenario: Large elapsed time is bounded
- **WHEN** a step reports an elapsed time longer than the maximum step interval
- **THEN** movement advances by the maximum step interval only

### Requirement: Boot sequence
On boot, movement SHALL perform the Initialise sequence and then hold Stand until a command requests otherwise. Until a controller provides commands, movement SHALL behave as if it received a Stand command.

#### Scenario: Boot without a controller
- **WHEN** the firmware starts and no controller supplies commands
- **THEN** the legs move through the Initialise poses and the body settles in Stand

### Requirement: Stand mode
In Stand the body SHALL bring every foot to the standing position defined by the current body height and stance width. When entering from Walk, the body SHALL move at most three legs at a time, highest first. When entering from Initialise, Sleep or Calibrate, it SHALL move all legs together. A change of body height while standing SHALL move the feet to the new height.

#### Scenario: Stand from Walk
- **WHEN** the body is in Walk and a command requests Stand
- **THEN** no more than three legs are lifted at any moment until every foot reaches the standing position

#### Scenario: Height change while standing
- **WHEN** the body is standing and the commanded height changes
- **THEN** the feet move to the new height without lifting

### Requirement: Walk mode and gaits
In Walk the body SHALL move each leg through a repeating cycle of a ground phase and a swing phase, according to the selected gait's phase offsets, ground fraction, stride, lift height and speed limits. The supported gaits SHALL be Tri, Wave, Ripple, Bi, Quad and Hop, with their current parameters. Changing gait SHALL re-phase the legs to the new gait's offsets. Translate and rotate inputs SHALL be superposed in each leg's path: the foot's stroke SHALL be the sum of the translation stroke and the rotation stroke, so that turning while walking follows an arc without shortening the forward stride. A single-motion stroke whose ends would be unreachable on any leg SHALL scale down by the same factor on every leg, by no more than needed to reach. The combined stroke's limit SHALL be the larger of the gait's stride limit and the longer of the two single-motion strokes, as each motion alone would take them, so that adding a turn never shortens the forward stroke the walk alone would take. When any leg's combined stroke would exceed that limit, or a combined stroke end would be unreachable, both parts SHALL scale down by the same factor on every leg. Shortening for reach SHALL change smoothly as the input or the body height changes. A combined swing SHALL lift to the higher of the two motions' swings. A positive rotate amount SHALL turn the body anticlockwise, seen from above.

#### Scenario: Gait change re-phases legs
- **WHEN** the body is walking in Tri and a command selects Wave
- **THEN** each leg's cycle position is reset to Wave's phase offset and walking continues

#### Scenario: Zero input while walking
- **WHEN** the body is in Walk and both the translate vector and the rotate amount are zero
- **THEN** gait progress stops, every foot holds its current position and every computed position is finite

#### Scenario: Rotate-only input
- **WHEN** the translate vector is zero and the rotate amount is non-zero
- **THEN** the legs follow the rotation path only and every computed position is finite

#### Scenario: Translate-only input
- **WHEN** the rotate amount is zero and the translate vector is non-zero
- **THEN** the legs follow the straight path only, as they did before steering was added

#### Scenario: Turning while walking keeps the forward stride
- **WHEN** the body walks forward at translate 60 and the rotate amount changes from 0 to 30
- **THEN** each foot's forward component of its ground stroke stays the same, and a rotation component is added

#### Scenario: Combined stroke within the stride limit
- **WHEN** translate and rotate are both 100 in every gait, at every body height
- **THEN** no leg's ground stroke exceeds the combined limit, the forward and rotation parts keep their ratio and share one scale on every leg wherever neither single stroke is shortened for reach, and every target is reachable wherever each motion alone is

#### Scenario: Long strides shrink to reach
- **WHEN** the body walks forward with translate 100 at the speed dial 100 and body height 100
- **THEN** every stroke end is reachable, and the stroke is shorter than at body height 0

### Requirement: Stride mode
Walk SHALL support two stride modes. In dynamic mode, stride and rotation length SHALL scale with input magnitude. In fixed mode, stride and rotation length SHALL be constant and the input SHALL set only direction.

#### Scenario: Fixed stride ignores magnitude
- **WHEN** stride mode is fixed and the translate magnitude changes from 30 to 90 in the same direction
- **THEN** the stride length stays the same

### Requirement: Command smoothing
Movement SHALL ease the translate vector, rotate amount, body height, speed dial and stride mode towards their commanded values instead of applying them at once. Translate, rotate, the speed dial and the stride mode SHALL settle within about one second, body height more slowly. A change of the speed dial or of the stride mode SHALL lengthen or shorten the stride gradually, without moving a foot further in one step than walking does.

#### Scenario: Step input is eased
- **WHEN** the commanded translate magnitude jumps from 0 to 100
- **THEN** the applied magnitude rises over successive steps and does not reach 100 in the first step

#### Scenario: Speed dial and stride mode are eased
- **WHEN** the body walks at dial 44 and the command changes to dial 100 and the fixed stride mode
- **THEN** the applied dial and stride mode change over successive steps, and no foot moves more in the first step than about the most it moved per step before

#### Scenario: Smoothing is reset on Sleep
- **WHEN** the body enters Sleep
- **THEN** the applied translate and rotate values are reset to zero

### Requirement: Sleep mode
In Sleep the body SHALL move all feet to the rest pose and then switch off all servo outputs. Leaving Sleep SHALL re-enable the outputs and pass through Stand before any other mode.

#### Scenario: Enter sleep
- **WHEN** a command requests Sleep
- **THEN** the feet move to the rest pose and, once every foot is there, all servo outputs are switched off

#### Scenario: Wake from sleep
- **WHEN** the body is asleep and a command requests Walk
- **THEN** servo outputs are switched on, the body performs the Stand transition, and then enters Walk

### Requirement: Stale command safety
Movement SHALL treat a command as stale when it is older than the command timeout of 1 second. On a stale command the body SHALL enter Sleep, interrupting any transition in progress, and SHALL stay asleep until a fresh command requests another mode.

#### Scenario: Controller stops sending
- **WHEN** the body is walking and no fresh command arrives for more than 1 second
- **THEN** the body enters Sleep

#### Scenario: Default command is not stale
- **WHEN** no controller is present and movement uses its built-in Stand command
- **THEN** the stale-command rule does not trigger

### Requirement: Calibrate mode
In Calibrate the body SHALL first raise any foot that is below the intermediate height, leaving feet that are already higher where they are. It SHALL then move every foot towards the calibration pose at a bounded speed, approaching from either side without overshooting or jumping. Leaving Calibrate SHALL pass through Stand.

#### Scenario: Only low legs are raised
- **WHEN** Calibrate is entered with some feet below and some above the intermediate height
- **THEN** only the feet below it move upwards in the first phase

#### Scenario: Approach from above
- **WHEN** a foot coordinate is greater than its calibration target
- **THEN** it decreases towards the target over successive steps instead of jumping to it

### Requirement: Joint speed budget
Movement SHALL keep every joint's angular speed within the joint speed budget of the selected servo profile, in every body mode except Initialise.

Under a budget, movement SHALL advance in pieces of 20 ms of elapsed time, counted from its first step, and SHALL carry elapsed time short of a whole piece to the next step. A step SHALL advance at most three pieces; elapsed time beyond that (after an overrun) SHALL be dropped, as for a long stall. When advancing a piece at the full rate would move any joint faster than the budget, movement SHALL advance that piece's motion by less time, so that the fastest joint moves at about the budget. The motion SHALL follow the same foot paths as it would without the budget, only more slowly. The budget SHALL slow translate, rotate and height smoothing, gait progress, Stand, Sleep and Calibrate transitions, and gesture playback alike.

Stale-command detection SHALL keep using elapsed wall time. Movement SHALL always make some progress: a piece SHALL advance by at least a fiftieth of its time, leaving any excess to the output rate backstop. Movement SHALL estimate each piece from at most three runs. Where no run keeps within the budget, it SHALL keep the run that came closest.

#### Scenario: Walking within the budget
- **WHEN** the body walks in every gait with translate 100, rotate 0 or 100, the speed dial at 0, 44 or 100, and the body height at 0, 50 or 100
- **THEN** in at least 99.5 % of consecutive 20 ms steps no joint's angle changes faster than the budget, with 5 % tolerance, and in no step faster than 1.6 times the budget

#### Scenario: Paths are kept
- **WHEN** the body walks in Tri with translate 100 and the speed dial at 100, once under a budget it exceeds and once without a budget
- **THEN** each foot position reached under the budget lies within 1 mm of the path the foot follows without it, or within 2 mm where a joint is held at its servo limit

#### Scenario: Slow motion is unaffected
- **WHEN** the body walks in Tri with translate 30 and the speed dial at 0
- **THEN** movement advances by the full elapsed time on every step

#### Scenario: Transitions and gestures within the budget
- **WHEN** the body goes from Sleep to Stand, changes height from 0 to 100, walks then stands, goes to Sleep, enters Calibrate, or plays each gesture
- **THEN** no joint's angle changes faster than the budget, with 5 % tolerance, between consecutive 20 ms steps

#### Scenario: Tick-rate independence under the budget
- **WHEN** the same command sequence is applied under the budget with steps of 20 ms and with steps of 50 ms over the same total time
- **THEN** the resulting leg positions agree within 5 mm

#### Scenario: Stale commands still time out on wall time
- **WHEN** the body is walking under the budget and commands stop arriving
- **THEN** the body goes to Sleep after the same wall-clock timeout as without the budget

#### Scenario: Initialise is not slowed
- **WHEN** the firmware starts
- **THEN** the Initialise poses are set and held for their usual durations

### Requirement: Continuous foot paths on entering a phase
A leg that enters its cycle part-way through its ground phase or its swing (on entering Walk, or when a gait change re-phases it) SHALL start that path segment from where its foot is and SHALL complete it, at the segment's usual end point, by the end of that phase. A leg SHALL NOT jump to the position its cycle position would have on the full segment.

#### Scenario: Entering Walk mid-swing
- **WHEN** the body enters Walk in Bi and a leg's phase offset lies in the middle of its swing
- **THEN** that leg's first target is its current foot position, and its later targets move it in small steps to the swing's end
