# Proposal

## Why

The robot's state is only visible on the bridge screen, the robot's LEDs and the desktop. While driving, eyes are on the robot: a latched fault, an emergency stop, a lost link or a refused gesture can go unnoticed. The gamepad's rumble motors (driven through Bluepad32) can tell the operator without looking.

## What Changes

- The bridge rumbles the connected gamepad on these events, each with its own pattern:
  - a protection fault newly latched on the robot: a long, strong rumble;
  - the bridge's emergency stop switched on: a short strong rumble; switched off: a light tap;
  - the robot link lost while a gamepad is connected: a medium rumble; regained: a light tap;
  - a gesture request the robot did not act on within 0.6 s: a light double-length tap.
- Events are detected from what the bridge already knows (telemetry, its e-stop toggle, its own gesture requests). On connecting, the current state is the baseline: no rumble for faults that were already latched.
- One rumble at a time; a more important event replaces a lighter one in progress, a lighter one waits for a stronger one to end, and the same event never repeats within a second.
- Gamepads without rumble ignore the request.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gamepad-control`: rumble feedback.

## Impact

- New hardware-free `core/gamepad_mapping/rumble.{h,cpp}` with host tests; `firmware/bridge/main/bridge.cpp` plays the rumble through Bluepad32.
- No protocol or robot change.
