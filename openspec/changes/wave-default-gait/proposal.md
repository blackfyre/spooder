# Proposal

## Why

The weighed robot (2929 g with the battery) is heavier than its MG996R servos can carry in a tripod gait. A static estimate with a 20 % mass margin puts the tripod's femur torque at 151–175 % of stall at 6 V with the stance at 190 mm. Wave keeps five or six feet down and spreads the load most evenly of the six gaits. Its femur torque is 135 % of stall at the current stance and 99 % at a 150 mm stance. Starting in Wave means the first walk does not overload the femurs. Tri stays selectable.

## What Changes

- Every controller starts with Wave selected instead of Tri: the bridge's gamepad mapping (`core/gamepad_mapping`), the handheld's Go port of it (`desktop/handheld/internal/padmap`) and the desktop calibrator's keyboard driving (`desktop/internal/session`).
- The gait order and the next/previous wrap are unchanged: Tri, Wave, Ripple, Bi, Quad, Hop.
- The robot's own initial gait (`body`) is unchanged. Every command frame carries the gait, so the robot walks in whatever the controller sends.
- No protocol change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gamepad-control`: the input mapping states the gait selected at start-up, Wave.
- `desktop-calibrator`: keyboard driving starts at Wave instead of Tri.

The handheld follows `gamepad-control` through its mapping requirement (`handheld-controller`, added by `add-handheld-controller`), which needs no text change.

## Impact

- `core/gamepad_mapping/gamepad_mapping.h`, `desktop/handheld/internal/padmap/mapping.go`, `desktop/internal/session` (the initial gait), and the tests and fixtures that start from Tri: `test/golden/gamepad_mapping.txt` (hand-written: every expected gait moves one step on) and the session tests.
- `CHANGELOG.md`, `docs/controls.md` if it names the starting gait.
- Flashing the bridge and installing the pak pick up the new default; the robot needs no flashing.
