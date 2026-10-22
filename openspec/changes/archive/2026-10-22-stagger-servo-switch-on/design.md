# Design

## Context

`robot_tick` hands each movement step to `Hardware::applyServos`. The adapter (`servo_output` on the board, `sim::Servos` in the virtual robot) switches output on when the body wants it and protection allows it, and on switch-on pulses every output that has a pulse: all 18 in one `cluster.load()`. `robot_tick::limitOutput` already treats the first pulse of each leg after switch-on as a step change (`outputKnown`), because the servo's position is unknown.

## Goals / Non-Goals

**Goals:**
- Spread the servos' start at switch-on over just enough ticks that their start surges no longer coincide.

**Non-Goals:**
- Holding the body's motion during the stagger. The stand's path moves a few degrees in 40 ms, and a leg's first pulse is a step change anyway.
- Staggering movement while output is on; the speed budget and the output rate backstop cover that.
- Changing how output switches off.

## Decisions

- **Two legs per tick, three ticks.** A servo's start surge lasts well under a tick, so one tick between groups separates them. Six servos per group cut the measured 11.8 A to about 4 A, under half the 9 A trip, for 40 ms of delay. One leg per tick (100 ms) would cut more than needed. The group size is a named constant in `robot_tick`.
- **The decision lives in `core/robot/robot_tick`.** It knows whether output was on at the start of the tick (`servosOn()`), so it counts the released legs: none while output is off, then two more on each tick from the tick that switches output on, up to six. It passes the count to `applyServos`, and the host tests cover it.
- **The adapter enables only released legs.** A leg below the count gets its pulses as before; when it is first released, every output of it that has a pulse is pulsed. A leg not yet released stays disabled even if its angles change. Switching off clears the released state.
- **The backstop follows the release.** A leg not yet released counts as unknown in `limitOutput`, so its first pulse goes straight to the command, as for any first pulse after switch-on.

## Risks / Trade-offs

- [The last legs start 40 ms later than the first] → In a stand from Sleep or Initialise the late legs join a path that has barely moved; acceptable.
- [A loaded stand may draw more than the start surge] → Measure a loaded stand from Sleep on the bench; this change only removes the coincident start.
