# Proposal

## Why

The weighed robot (2929 g with the battery) is heavier than its MG996R servos can carry in a tripod gait. A static estimate with a 20 % mass margin puts the tripod's femur torque at 151–175 % of stall at 6 V with the stance at 190 mm. Wave keeps five or six feet down and spreads the load most evenly of the six gaits. Its femur torque is 135 % of stall at the current stance and 99 % at a 150 mm stance. Starting in Wave means the first walk does not overload the femurs. Tri stays selectable.

## What Changes

- Every controller starts with Wave selected instead of Tri: the bridge's gamepad mapping (`core/gamepad_mapping`) and its first frames, the bridge's WiFi drive without a live handheld, the handheld's Go port of the mapping (`desktop/handheld/internal/padmap`), the desktop calibrator's keyboard driving (`desktop/internal/session`), `spooder-wifi-probe` and the `--serial` build's keys.
- The gait order and the next/previous wrap are unchanged: Tri, Wave, Ripple, Bi, Quad, Hop.
- The robot starts in Wave too, and runs in it with no source in control (`gait::START_GAIT`, which `body`, `MovementCommand` and the mapping take): otherwise telemetry reports Tri while the body is stopped (no servo supply) or before a controller is in control, and the bridge screen, the pak and the calibrator show it.
- No protocol change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gamepad-control`: the input mapping states the gait selected at start-up, Wave.
- `desktop-calibrator`: keyboard driving starts at Wave instead of Tri.
- `usb-link`: with no source in control the robot runs the starting gait, Wave, and it starts in it.

The handheld follows `gamepad-control` through its mapping requirement (`handheld-controller`, added by `add-handheld-controller`), which needs no text change.

## Impact

- `core/gamepad_mapping/gamepad_mapping.h`, `desktop/handheld/internal/padmap/mapping.go`, `desktop/internal/session` (the initial gait), and the tests and fixtures that start from Tri: `test/golden/gamepad_mapping.txt` (hand-written: every expected gait moves one step on) and the session tests.
- `CHANGELOG.md`, `docs/controls.md` if it names the starting gait.
- `core/movement/gait.h` (`START_GAIT`), `movement_command.h`, `body.cpp`, `core/wifi_drive/handheld_controller.cpp`, `firmware/bridge/main/link.cpp`, `desktop/internal/wifiprobe` and their tests.
- Flash the robot and the bridge and install the pak.
