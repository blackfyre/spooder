# Design

## Context

`body::step` advances the state machine in 5 ms sub-steps. It then evaluates the walk's foot paths once and solves the joint angles once per step. All motion rates come from elapsed time: gait progress (`gait::progressRate`, scaled by the speed dial), the smoothing time constants, `STAND_RATE`, the sleep time constant, `CALIBRATION_RATE` and the gesture player's timing. `robot_tick` runs `body::step`, then hands the body state to `Hardware::applyServos`. On the board, `servo_output::apply` turns angles into pulses with `joint_calibration::toPulse`. The only rate limit is `joint_calibration::MAX_RATE_DEG_PER_S` (60°/s), and it applies only to calibration changes.

Measured on the host, with full stick forward in Tri, the femur peaks at about 1370°/s at dial 44 and 3050°/s at dial 100. The peaks come in the swing's lift. The stance stays near the median (about 180°/s at dial 44). Stand, Sleep and height changes peak at 90–490°/s. Initialise is a step change, by design.

Build-time hardware choices follow the power-limits pattern: a CMake cache variable, also settable from the environment, and a compile-time check.

## Goals / Non-Goals

**Goals:**
- Joint speed bounded by the fitted servo, with the foot paths unchanged.
- Changing servo model means changing one build option and, for a new model, adding one table entry.
- A shared core implementation, so the board and the virtual robot behave alike.
- Foot paths without step changes, which no slowing of the clock could bound.

**Non-Goals:**
- Retuning the gait table, lift heights or the speed dial's scale. The budget caps them. Retuning for better use of the budget is separate tuning work.
- Ramping Initialise, or starting the servos in turn at power-on.
- Per-joint or per-leg budgets, voltage-dependent speed, and torque or stall-current data in the profile. The power limits remain the current protection.
- Servo travel or pulse range in the profile. Per-joint calibration (0° and 180° pulses) already covers a different pulse mapping.
- Choosing the profile at run time, or over the link.
- The folded-knee side of target validation.

## Decisions

### Profiles are a compiled table in core, chosen by name at build time

`core/movement/servo_profile.h` (header-only) holds `struct ServoProfile { name, noLoadDegPerS, loadShare }` with `budgetDegPerS()`. The first entry is the MG996R: 0.14 s/60° at 6 V (about 429°/s) and a load share of 0.6, giving about 257°/s.

CMake's `SPOODER_SERVO_PROFILE` (default `MG996R`) becomes a string compile definition. A `constexpr` lookup with a `static_assert` makes an unknown name fail the build, and the message lists the known names. The firmware (on `spooder_core`, public) and the virtual robot both read the option. `mise.toml` passes the environment variable through, as it already does for `SPOODER_POWER_LIMITS`. Without the definition (the host tests), the header defaults to the MG996R.

*Alternative:* a user header like `power_limits.h`. Rejected: a profile is a fact about a servo model, not a site-specific choice. Datasheet values belong in the repository, reviewed and tested. *Alternative:* a run-time setting over the link. Rejected: a servo swap is a physical rebuild, and a wrong value sent at run time could overdrive the servos.

### The budget slows the body's motion clock, in fixed 20 ms pieces

`body::step` keeps wall time for staleness (`isStale`, `lastStepMs`). Under a budget it adds the elapsed time to `motionPendingUs` and advances in whole 20 ms pieces, carrying the remainder. A step without a whole piece moves nothing, so a command change cannot move a joint unmeasured. A new gesture request waits for the next piece instead of being dropped. Only the very first step acts at once, for Initialise's first pose. Every tick length governs the same pieces, so motion stays independent of the tick length. On the board's 20 ms ticks each step is exactly one piece.

An earlier version cut pieces at tick boundaries, and the governor's approximations then built up as phase drift: 9 mm after 3 s between 20 ms and 50 ms ticks.

Each piece:
1. **Run 1.** Copy the state, run the sub-steps for the full piece and finish (walk evaluation, angle solve). Also keep the state finished at mid-piece. Measure the largest joint change against the piece's start.
2. **Run 2.** If the change exceeds `budget × 20 ms`, choose the next motion share by interpolating through three points: no change at no time, the mid-piece change and the full change. Aim at 0.9 of the budget. This also fits pieces whose fast part comes early, such as a swing's landing, where change grows concavely with time.
3. **Run 3.** If still over, use a proportional estimate with a 0.7 margin, since it is the last run.
4. Accept the first run within the budget. If none is within, keep the run that came closest. Near awkward poses less time can mean more change, so the last run is not always the best.

The motion share has a floor of 0.02, so motion always progresses. Fast gaits at dial 100 need shares down to about 0.06; the floor is reached only near awkward poses. Initialise is never scaled: its poses are step changes by design. Sub-step lengths are in microseconds. Initialise and Perform keep their integer-millisecond bookkeeping by accumulating the sub-millisecond remainder.

The trial, mid-piece and best states are static `BodyState`s (924 bytes each), because the board's main stack is 2 KB.

Measured over the whole sweep (every gait, dials 0/44/100, heights 0/50/100, rotate 0/100; 16,200 steps): 0.24% of steps exceed the budget by more than 5%, and the worst is 1.42–1.51× the budget, all at dial 100. Most governed pieces take two runs.

*Alternative:* cap each gait's progress rate offline from its geometry. Rejected: it covers only Walk, has to be redone for every gait, height and steering mix, and it slows the whole cycle, including the stance. The clock slows only where the peaks are. *Alternative:* slow from the previous step's measurement only, with no re-run. Rejected: the lift starts sharply, so a one-step lag would exceed the budget at every swing onset. *Alternative:* limit joint rates alone. Rejected for the reasons in the proposal: it distorts the foot paths and desynchronises the legs. *Alternative:* more runs. Rejected: four runs left about as many misses as three, and the run count is the board's cost.

### A leg entering a phase part-way starts from its foot

`LegCycle` gains `segmentStart`, the cycle position where the current segment began. It is 0 or the ground fraction for normal phase changes, and the entry position for a leg leaving `Reset` or `Standing` mid-phase. The segment's path parameter maps from `segmentStart` to the phase end. The first target is therefore the current foot, and the segment still ends where it always did. Walking that passes through phase changes normally is unchanged. Only entry (entering Walk, gait change) differs.

### Reach is checked from the femur joint, and strokes shrink smoothly to it

`kinematics::isValidTarget` accepts a finite target whose distance from the femur joint is at most 299 − 4 = 295 mm (`MAX_KNEE_SPAN`). At that distance the knee is at about 158°, and the joint rate is about 2.4°/mm of foot movement. Closer to straight it is unbounded. The 340 mm from the coxa pivot used before accepted nearly straight and unreachable knees, which the IK clamped. A folded-knee limit was tried and dropped. It shortened diagonal strides at every height, and the folded side's rate stays near 1°/mm until very close to the limit.

`strideFor`'s reach shrink now applies to single-motion strokes as well as combined ones. The combined-stroke limit uses each single stroke as that motion alone would take it, that is, shortened for reach. The factor is found by bisection: 12 halvings between 13% of the stroke and the full stroke. It used to be 5% steps, which made every foot jump by 5% of the stroke whenever the step count changed.

The stride's leg-independent part is computed once per distinct input, memoised on the exact `GaitParams` and `GaitInput`, rather than once per leg. This keeps the extra reach checks within the board's budget.

### The backstop lives in robot_tick, on servo angles

`robot_tick` keeps the last written servo angle per joint and limits the angles it hands to `Hardware::applyServos` to `rated no-load speed × elapsed` per tick. That is the servo's own datasheet speed, 1.67 times the budget for the MG996R. A first choice of 1.25 times the budget engaged in ordinary walking at height 100, where the governor misses by up to 1.34×. Initialise passes unlimited, as its poses are step changes by design. It passes a body state copy with the limited angles. A held-back leg keeps `legUpdated` set until it reaches its target, so `servo_output` keeps writing. The limiter resets when outputs switch off (`servosOn()` false), so the first pulse after switching on goes straight to the command.

The engagement count and the most-held-back joint are reported on a separate console line at the status period, only when the count has risen. The existing status line is parsed by the `desktop/internal/console` tests and stays as it is. Up to the default dial it never engages, in any gait or height, steering, standing or sleeping. At dial 100 it engages rarely, in Hop's infeasible inward poses at height 0.

*Alternative:* limit in `servo_output` on the board. Rejected: the virtual robot would not share it, and the e2e tests could not see it.

### The speed dial and the stride mode are eased

`cmd.speed` and `cmd.stride` set the stride directly, so a dial step or a mode toggle moved every foot at once, which no clock slowing can bound. The body now eases both, like translate (τ 0.25 s): `BodyState::speed`, and `fixedStride` as a 0–1 share. `GaitInput::fixedStride` becomes a float, and the gait blends the dynamic and fixed strides by it.

### Feet are fitted to the servo travel in the IK

With the compiled defaults (femur offsets 75–89°, carried over from the old robot), walking, the walk-to-stand transition, Sleep and Initialise all asked the femur for more than 180°, by up to 72°. The tibia went below 0° by up to 5° at a straight knee. The pulse clamp hid this, and the other joints then disagreed with the clamped femur.

Two searches were tried first and reverted:
- moving the foot to the furthest valid point on the way from its last position;
- lowering it to the highest valid height.

Both made feet jump, because the valid heights are not an interval when a swing lifts far above the coxa (Hop's lift is 325 mm).

The limits have a closed form about the femur joint, so `kinematics::solveLegInRange` fits the target directly, in three steps:
1. **Coxa:** the azimuth is clamped to [−offset, 180° − offset].
2. **Tibia:** its angle depends only on the femur joint to foot distance h, so its travel bounds h (the bounds are cached per offset). h is clamped along the same elevation.
3. **Femur:** at that h, its angle is φ1(h) + elevation + offset, so the elevation is clamped to [−offset − φ1, 180° − offset − φ1].

Each clamp is continuous in the target, so a swing slides along the femur's limit instead of jumping. A target in travel is solved exactly as `solveLeg` solves it. The limits are inset by 0.01°, so rounding never leaves the travel. The body uses the fit in every mode but Calibrate, where each joint must show its calibration unaltered, and records the fitted foot. The cost is that of `solveLeg`, plus a cache lookup.

### Board cost

Measured on the Servo 2040 (125 MHz, ROM soft float: `sqrtf` 2.3 µs, `sinf` 6.2 µs, `solveLeg` 56 µs) with a temporary benchmark build, for governed full-steer Hop at dial 100 and height 100. The first version took 56 ms per step at most (44 ms mean), and even ungoverned 32 ms. The changes:
- **The reach factor is solved, not searched.** A stroke end is the stance point plus the factor times a direction, and the reach at the ground's height is a circle about the coxa pivot (radius coxa + √(295² − z²), less 1.5 mm for the rotation's integer truncation). So each end's limit is the root of a quadratic. Alone, the straight stroke needs only legs 0–2 (3–5 mirror them), and the rotation stroke one leg.
- **Placement trigonometry is cached.** `placed()` uses precomputed cosine and sine, with the same truncation as `Vector3::rotate`.
- **The stride is computed only where it is needed.** `trackPhase` computes it only when a leg changes phase, and `strideFor` memoises it per distinct input.
- **Cheaper arithmetic.** `bezier` uses de Casteljau instead of `powf`, `isValidTarget` takes one square root, `smoothing` caches its `expf` factor per time constant, and `Vector3::distanceTo` is single precision.
- **At most three pieces per step**, so an overrun cannot snowball.

Result: at most 8.4 ms (6.4 ms mean) governed and 2.8 ms ungoverned. With the rest of the tick (about 2 ms), the worst case uses about 10.4 ms of the 20 ms period. The governor needs its three runs: two left 243 misses and a worst case of 2.6×.

### The pose frame stays as it is

`commandedCentideg` remains the IK result. `pulseUs` is the written pulse, now possibly limited. No protocol change is needed.

## Risks / Trade-offs

- [Several body evaluations per tick on the RP2040, which has no FPU: up to three runs plus the mid-piece finish] → Measure `body::step` on the board, governed and ungoverned, with full-stick Hop at dial 100 at height 100 as the worst case. The 20 ms loop must keep at least 50% headroom. If it does not, drop the third run, and the backstop takes the residual.
- [Ticks that are not a multiple of 20 ms get pieces in bursts: 10 ms ticks alternate between none and one, 25 ms ticks sometimes carry two] → Accepted for exact tick-rate independence. The board ticks at 20 ms, and the servos take a new pulse once per 20 ms frame. An overrun gives one double piece, and the backstop smooths it.
- [Smoothing slows with the motion clock, so stick response lags while governed] → Accepted. The response then follows what the servos can do.
- [The 0.6 load share is an estimate] → It is a profile field: tune it on the bench by watching current and foot lift, and record the value in the table.
- [Walking at high dial settings is slower, and strides at body height 100 are shorter (50–78% at dial 44, 25–51% at dial 100)] → Expected and intended. It is noted in the changelog and the hardware-check skill.
- [Rare governor misses (0.24% of steps, at most about 1.5×)] → The servos are still within their rated speed. The backstop bounds anything beyond it, and its console line makes that visible.
- [Static scratch states make `body::step` non-reentrant] → One body steps at a time on the board, in the virtual robot and in the tests.

## Migration Plan

This is a firmware-only change with no protocol bump, so the bridge does not need reflashing. A build without the option uses the MG996R profile. To roll back, revert the change. There is no persistent state.
