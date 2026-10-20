# Proposal

## Why

The movement code asks the joints for speeds the fitted servos cannot reach. A host run of the body at the 20 ms tick, full stick forward in Tri, peaks at about 1370°/s on the femur at the default speed dial (44) and about 3050°/s at dial 100. An MG996R manages about 430°/s at 6 V without load. The legs lag their commands, the swing does not lift as planned, and the servos run near stall. Nothing between the inverse kinematics and the servo output limits joint speed. The MG996Rs may later be replaced by other standard-size servos, so the limit must come from the servo actually fitted, not from a constant tuned for one model.

Bounding joint speed also exposed two places where the planned foot path itself asks for unbounded joint speed:
- A leg that enters its cycle part-way through a phase jumps onto the path. Entering Walk in Bi or Ripple moves a foot to mid-swing in one tick, up to 45°.
- Target validation accepts feet up to 340 mm from the coxa pivot, but the femur and tibia reach only 299 mm from the femur joint. Near that limit, which body height 100 reaches with a full stride, the knee is nearly straight and the joint angles change without bound.

## What Changes

- **Servo profiles**: a table of servo models with their rated speed and the share of it the body may use under load. One profile is chosen at build time, defaulting to the MG996R. An unknown profile name fails the build. The robot names the profile on its USB console at start-up. The virtual robot uses the same profile.
- **Joint speed budget**: the profile gives a budget in degrees per second. In every body mode except Initialise, movement advances in 20 ms pieces of elapsed time. Each piece's motion is slowed when moving at full rate would turn any joint faster than the budget. Feet follow the same paths as at full rate, only more slowly, so gaits, transitions and gestures keep their shape, and motion stays independent of the tick length. The governor estimates each piece from at most three runs, so it keeps within 5% of the budget in nearly every step. Near awkward poses at the highest dial settings it can miss by a bounded amount. Stale-command timing stays on wall time.
- **Output rate backstop**: the servo output never moves a joint's commanded angle faster than the profile's rated no-load speed (429°/s for the MG996R, 1.67 times the budget). It does not engage up to the default speed dial. At dial 100 it takes the governor's rare larger misses. When it engages, the robot counts it and reports it on the USB console.
- **Continuous foot paths**: a leg that enters a phase part-way (entering Walk, a gait change) starts that segment from where its foot is and completes it over the rest of the phase.
- **Leg reach**: a target is valid only within the femur and tibia's reach from the femur joint, less a 4 mm margin (295 mm). Strokes whose ends would not reach now shrink for single motions too, not only for combined ones, and they shrink smoothly rather than in 5% steps. At body height 100 this shortens strides: to 50–78% of the stroke at the default dial and 25–51% at dial 100. Heights 0 and 50 are barely affected.
- **Eased speed dial and stride mode**: a change of either lengthens or shortens the stride gradually, like translate and height, instead of moving every foot at once.
- **Board cost**: the stride's reach is solved exactly per stroke end instead of searched, placement trigonometry is cached, Bézier curves avoid powers, and a step advances at most three pieces after an overrun. In the worst case (governed full-steer Hop, dial 100, height 100) `body::step` takes at most 8.4 ms on the RP2040.
- **Feet fitted to the servo travel**: outside Calibrate, a foot target that would take a joint outside 0–180° (offset included) is moved just enough, continuously, to keep every joint within travel. With the compiled defaults (femur offsets 75–89°), walking, the transitions, Initialise and Sleep all asked the femur for more than 180°, up to 72° more.
- The speed dial keeps its meaning as a request. Dial settings that would exceed the budget give the fastest motion the servos can follow, not faster.
- No link protocol change. The pose frame's pulse field already shows what was written.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `locomotion`: the joint speed budget; continuous paths on entering a phase part-way; strokes shrink to stay reachable for single motions too.
- `leg-actuation`: servo profiles chosen at build time, the output rate backstop, and target validation against the femur–tibia reach.

## Impact

- `core/movement/`:
  - `servo_profile.h`: the profile table.
  - `body`: the governed motion clock.
  - `gait`: path continuity, the general and smooth reach shrink, and the shared stride computed once per input.
  - `kinematics`: the reach check.
  - Host tests sweep every gait, dial, height, steering mix, transition and gesture against the budget.
- `core/robot/robot_tick`: the output rate backstop before the hardware writes pulses, the engagement count and its console line, and the profile name on the first tick.
- `firmware/servo2040/CMakeLists.txt`, `tools/virtual-robot/CMakeLists.txt` and `mise.toml`: the `SPOODER_SERVO_PROFILE` build option.
- Behaviour on the robot: walking at higher dial settings is slower than it is now, and strides at body height 100 are shorter. The existing status line format, the link protocol and the bridge are unchanged.
