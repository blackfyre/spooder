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
In Walk the body SHALL move each leg through a repeating cycle of a ground phase and a swing phase, according to the selected gait's phase offsets, ground fraction, stride, lift height and speed limits. The supported gaits SHALL be Tri, Wave, Ripple, Bi, Quad and Hop, with their current parameters. Changing gait SHALL re-phase the legs to the new gait's offsets. Translate and rotate inputs SHALL be blended into each leg's path in proportion to their magnitudes.

#### Scenario: Gait change re-phases legs
- **WHEN** the body is walking in Tri and a command selects Wave
- **THEN** each leg's cycle position is reset to Wave's phase offset and walking continues

#### Scenario: Zero input while walking
- **WHEN** the body is in Walk and both the translate vector and the rotate amount are zero
- **THEN** gait progress stops, every foot holds its current position and every computed position is finite

#### Scenario: Rotate-only input
- **WHEN** the translate vector is zero and the rotate amount is non-zero
- **THEN** the legs follow the rotation path only and every computed position is finite

### Requirement: Stride mode
Walk SHALL support two stride modes. In dynamic mode, stride and rotation length SHALL scale with input magnitude. In fixed mode, stride and rotation length SHALL be constant and the input SHALL set only direction.

#### Scenario: Fixed stride ignores magnitude
- **WHEN** stride mode is fixed and the translate magnitude changes from 30 to 90 in the same direction
- **THEN** the stride length stays the same

### Requirement: Command smoothing
Movement SHALL ease the translate vector, rotate amount and body height towards their commanded values instead of applying them at once. Translate and rotate SHALL settle within about one second, body height more slowly.

#### Scenario: Step input is eased
- **WHEN** the commanded translate magnitude jumps from 0 to 100
- **THEN** the applied magnitude rises over successive steps and does not reach 100 in the first step

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
