## MODIFIED Requirements

### Requirement: Walk mode and gaits
In Walk the body SHALL move each leg through a repeating cycle of a ground phase and a swing phase, according to the selected gait's phase offsets, ground fraction, stride, lift height and speed limits. The supported gaits SHALL be Tri, Wave, Ripple, Bi, Quad and Hop, with their current parameters. Changing gait SHALL re-phase the legs to the new gait's offsets. Translate and rotate inputs SHALL be superposed in each leg's path: the foot's stroke SHALL be the sum of the translation stroke and the rotation stroke, so that turning while walking follows an arc without shortening the forward stride. The combined stroke's limit SHALL be the larger of the gait's stride limit and the longer of the two single-motion strokes, so that adding a turn never shortens the forward stroke the walk alone would take. When any leg's combined stroke would exceed that limit, or a combined stroke end would be unreachable, both parts SHALL scale down by the same factor on every leg. A combined swing SHALL lift to the higher of the two motions' swings. A positive rotate amount SHALL turn the body anticlockwise, seen from above.

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
- **THEN** no leg's ground stroke exceeds the combined limit, the forward and rotation parts keep their ratio and share one scale on every leg, and every target is reachable wherever each motion alone is
