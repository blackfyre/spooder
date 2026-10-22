# Proposal

## Why

When servo output switches on, all 18 servos get their first pulse in the same frame. Their positions are unknown while output is off, so each runs flat out towards its target at once. On the bench, with the servos unloaded, that read 11.8 A for one sample: the surge is the servos starting together, not the robot's weight. The two-level overcurrent trip now tolerates such a surge, but the bench supply sags about 0.125 Ω, so the surge still pulls the servo supply towards the brown-out voltage.

## What Changes

- **Staggered switch-on.** When servo output switches on, the legs receive their first pulses in groups of two, one group per 20 ms tick, in leg order: legs 1–2, then 3–4, then 5–6. The surge is cut to about a third (about 4 A for the measured 11.8 A), and the last legs start 40 ms after the first.
- Only the switch-on is staggered. Once a leg is on, its pulses follow the body as before; the body's motion is not held, and a leg that comes on later goes straight to its current target, as any first pulse after switch-on does.
- Switching off stays immediate for all outputs together.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `leg-actuation`: servo output switches on in groups of two legs per tick.

## Impact

- `core/robot/robot_tick`: counts the legs released since output switched on and hands that count to the hardware with each movement step.
- `Hardware::applyServos` gains the count; `firmware/servo2040/src/servo_output`, the virtual robot's servo model and the host-test fake honour it.
- No change to the link protocol, telemetry or the desktop calibrator. `outputs_on` reports output on from the first group.
