# Proposal

## Why

The robot should steer like a car: turning while walking forward, along an arc, at full walking speed. Today the gait blends the translate and rotate inputs in proportion to their magnitudes, normalised so that the weights sum to one. Adding a turn therefore takes stride away from walking: full forward and full turn give half a forward stride and half a turn, and a small turn noticeably slows the walk. On the gamepad, walking is on the left stick and turning on the right stick X, so a car-like "forward and steer" needs both thumbs, and the left stick's X axis strafes.

## What Changes

- **Superposed motion.** In Walk, each foot's ground stroke is the sum of the translation stroke and the rotation stroke about the body centre, instead of their normalised blend. Turning while walking keeps the full forward stride and adds curvature: the body follows an arc whose radius is set by the ratio of turn to forward input. When a leg's combined stroke would exceed its gait's stride limit, both parts scale down together, so the arc keeps its shape.
- **Walk only, rotate only and the zero-input guard behave as today:** with one input at zero, the paths are the current ones.
- **Car steering on the gamepad.** A new stick mode, steer, on by default: left stick Y walks forward or back, left stick X steers. Reversing while steering left swings the rear to the right, as a car does. The current layout (left stick translates and strafes, right stick X rotates) stays available as crab mode. `b` toggles between the two; the setting survives a controller reconnect like the other dials. In both modes the right stick X adds rotation.
- The bridge's screen shows the stick mode.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `locomotion`: the Walk requirement's input blending becomes superposition with a common stride limit.
- `gamepad-control` (from `add-gamepad-bridge`): the steer and crab stick modes and the `b` toggle.

## Impact

- `core/movement/gait.cpp` (stroke superposition and the stride limit), its host tests and golden checks of walking paths.
- `core/gamepad_mapping/` (stick mode, steering, reverse convention) and its tests; `core/status/status_screen` (stick mode).
- No protocol change: the command frame's translate and rotate fields carry the result.
- Both firmwares are rebuilt; flashing order does not matter.
- The `spooder-leg-kinematics` and `spooder-hardware-check` skills, the readme's controls table and the changelog.
