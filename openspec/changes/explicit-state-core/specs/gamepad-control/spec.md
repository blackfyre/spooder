## MODIFIED Requirements

### Requirement: Input mapping
The bridge SHALL map gamepad input to the desired movement state as follows. Button names are Bluepad32's.

| Input | Effect |
|---|---|
| Left stick | Translate vector, scaled to −100..100 per axis, magnitude clamped to 100 |
| Right stick X | Rotate amount, −100..100; right stick Y unused |
| `miscHome` (press) | Toggle requested mode between Stand and Walk. While Sleep is requested, switch the mode Sleep returns to and keep Sleep requested. While Calibrate is requested, leave it for Stand |
| `y` (press) | Toggle Sleep. Leaving Sleep returns to Stand or Walk, whichever Sleep was entered from; Sleep entered from Calibrate returns to Stand |
| `miscBack` + `miscHome` held 1 s | Toggle Calibrate, also from Sleep. Leaving it returns to Stand |
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

#### Scenario: Sleep returns to Walk
- **WHEN** Walk is requested, `y` is pressed and then pressed again
- **THEN** Sleep is requested after the first press and Walk after the second, and the mode-request counter has advanced by two

#### Scenario: Sleep from Calibrate returns to Stand
- **WHEN** Calibrate is requested, `y` is pressed and then pressed again
- **THEN** Sleep is requested after the first press and Stand after the second

#### Scenario: miscHome while sleeping
- **WHEN** Sleep was entered from Stand, `miscHome` is pressed and then `y` is pressed
- **THEN** Sleep is still requested after `miscHome`, with the mode-request counter advanced by one, and Walk is requested after `y`

#### Scenario: Calibrate hold completes with y pressed
- **WHEN** Stand is requested and `y` is pressed at the moment the `miscBack` + `miscHome` hold reaches 1 s
- **THEN** Calibrate is entered and then left for Sleep, so Sleep is requested, the mode-request counter has advanced by two, and a further `y` press requests Stand
