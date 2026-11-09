## MODIFIED Requirements

### Requirement: Input mapping
The bridge SHALL map gamepad input to the desired movement state as follows. Button names are Bluepad32's.

| Input | Effect |
|---|---|
| Left stick | Translate vector, scaled to −100..100 per axis, magnitude clamped to 100 |
| Right stick X | Rotate amount, −100..100; right stick Y unused |
| `miscHome` (press) | Toggle requested mode between Stand and Walk |
| `y` (press) | Toggle Sleep. Leaving Sleep returns to the mode requested before it |
| `miscBack` + `miscHome` held 1 s | Toggle Calibrate. Leaving it returns to Stand |
| `r1` / `l1` (press) | Next / previous gait, wrapping around |
| D-pad up / down (press) | Body height +10 / −10, within 0..100 |
| D-pad right / left (press) | Speed +10 / −10, within 0..100 |
| `x` (press) | Toggle stride mode between dynamic and fixed |
| `thumbL` + `thumbR` held 1 s | Fault-clear request |

Toggles and steps SHALL act once per press. Holding a button SHALL NOT repeat it.

At start-up the selected gait SHALL be Wave. The gaits SHALL be selected in the order Tri, Wave, Ripple, Bi, Quad, Hop.

#### Scenario: Gait selection wraps
- **WHEN** the last gait is selected and `r1` is pressed
- **THEN** the first gait is selected

#### Scenario: Wave at start-up
- **WHEN** the bridge starts and a gamepad connects
- **THEN** the gait sent is Wave, and one `l1` press selects Tri

#### Scenario: Calibrate needs a deliberate hold
- **WHEN** `miscBack` and `miscHome` are pressed together and released after 0.5 s
- **THEN** Calibrate is not toggled
