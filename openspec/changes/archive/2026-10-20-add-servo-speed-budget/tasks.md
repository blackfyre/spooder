# Tasks

## 1. Servo profiles

- [x] 1.1 Add the header-only `core/movement/servo_profile.h` with the profile table (MG996R: 0.14 s/60° at 6 V, load share 0.6), the derived budget and a `constexpr` lookup by name. Verify with host tests in `test/test_servo_profile`: the MG996R budget, and a lookup that fails for an unknown name.
- [x] 1.2 Add the `SPOODER_SERVO_PROFILE` build option (default `MG996R`, also from the environment) to `firmware/servo2040/CMakeLists.txt`, `tools/virtual-robot/` and `mise.toml` (host tests use the header default), with a `static_assert` naming the known profiles. Verify that `mise run build` passes, and that a build with `SPOODER_SERVO_PROFILE=NOPE` fails with the message.
- [x] 1.3 Print the profile name and budget on the first `robot_tick::tick` (init has no hardware to log to) (printable ASCII). Verify with a `test_robot_tick` case on the logged line.

## 2. Joint speed budget in movement

- [x] 2.1 Move the sub-step duration in `body` from integer milliseconds to microseconds, keeping integer bookkeeping in Initialise and Perform. Verify that the existing `mise run test` suites pass unchanged.
- [x] 2.2 Separate the motion clock from wall time in `body::step`, and add the governed step in 20 ms pieces with carry: run 1 with a mid-piece measurement, interpolated re-run, last run with a margin, best run kept, at most three runs, floor 0.02, Initialise excluded. Verify with new `test_body` cases for the "Walking within the budget", "Slow motion is unaffected", "Stale commands still time out on wall time" and "Initialise is not slowed" scenarios, and with the existing tick-rate independence tests under the budget.
- [x] 2.3 Make a leg entering a phase part-way start from its foot (`LegCycle::segmentStart`). Verify with `test_entering_mid_swing_starts_from_the_foot` and the existing gait tests (probing the nominal path from the phase start).
- [x] 2.4 Check reach from the femur joint (`MAX_KNEE_SPAN`, 4 mm margin), shrink single-motion strokes for reach too, base the combined limit on reach-shortened single strokes, find the factor by bisection, and compute the shared stride once per input. Verify with the kinematics tests ("Nearly straight knee"), the gait tests (combined stroke limit and reach), and the gesture reach test.
- [x] 2.5 Add the path-preservation test ("Paths are kept") and the transitions-and-gestures sweep (Sleep→Stand, height 0→100, Walk→Stand, Sleep, Calibrate, every gesture), and a "Long strides shrink to reach" gait test. Also cover a tick without a whole piece (no movement; a gesture request carried to the next piece). Verify that they pass in `mise run test`.

- [x] 2.6 Ease the speed dial and the stride mode (`BodyState::speed`, `fixedStride` as a share blended in the gait). Verify with `test_speed_dial_and_stride_mode_are_eased`, which fails with instant easing.
- [x] 2.7 Advance at most three pieces per step. Verify with `test_an_overrun_advances_at_most_three_pieces`.
- [x] 2.8 Bring the board cost down: reach solved per stroke end, cached placement trigonometry, `trackPhase` stride only on a phase change, de Casteljau, one square root per reach check, cached smoothing factors, float `distanceTo`. Verify with every host test unchanged and a temporary benchmark build on the Servo 2040.

- [x] 2.9 Fit feet to the servo travel (`kinematics::solveLegInRange`; the body uses it outside Calibrate). Verify with the kinematics tests (in travel unchanged; femur lift fitted continuously; tibia and coxa fitted) and `test_every_pose_stays_within_servo_travel` (every gait, dial, height and steering, transitions, Sleep).

## 3. Output rate backstop

- [x] 3.1 Add the backstop to `core/robot/robot_tick`: the last written angle per joint, the limit of the rated no-load speed × elapsed (Initialise unlimited), held-back legs kept updated, and a reset when outputs switch off. Verify with `test_robot_tick` cases for the "Fast command is held back" and "Output switched on" scenarios.
- [x] 3.2 Count engagements and print the separate console line when the count rises, leaving the status line unchanged. Verify with a `test_robot_tick` case for the line, a case showing that walking at dials 0 and 44 in every gait at every height, steering, then standing and sleeping, leaves the count at zero, and passing `mise run desktop-test` (console transcript).

## 4. Integration and documentation

- [x] 4.1 Run every verification gate: `mise run test`, `mise run build` with and without `--serial` (example power limits), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`. Verify that all of them pass.
- [x] 4.2 Update `CHANGELOG.md` (Unreleased), `CLAUDE.md` (layout and build option), the `spooder-leg-kinematics` and `spooder-hardware-check` skills (profiles, budget, the backstop line, slower walking at high dial settings) and the readme where it lists build options. Verify that the docs name `SPOODER_SERVO_PROFILE` and its default.
- [x] 4.3 On the board, time `body::step` ungoverned and governed (full-steer Hop at dial 100, height 100) with a temporary benchmark build. Measured: at most 2.8 ms ungoverned and 8.4 ms governed, about 10.4 ms of the 20 ms tick with the rest of the tick (48 % headroom, against 50 % planned).
- [ ] 4.4 Walk Tri at dials 44 and 100 on a stand and then on the ground. Verify no backstop line at dial 44, full foot lift on every swing (the femur reaches its travel limit with the old default calibration), and the supply current.
