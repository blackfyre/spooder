# Spec Delta

## ADDED Requirements

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

## MODIFIED Requirements

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
