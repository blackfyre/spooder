# Proposal

## Why

On the bench, with the desktop calibrator connected, walking the robot still needs the bridge and a gamepad. The calibrator can take control, but it only requests Stand, Calibrate and Sleep, and its translate and rotate are always zero. Driving from the keyboard while in control lets one computer bring up, calibrate and walk the robot.

## What Changes

- While the app is in control, held keys drive the robot, as the bridge's gamepad does in its default (car) layout:
  - W and S walk forward and back;
  - A and D steer, turning on the spot when neither W nor S is held, and mirrored when reversing, as a car's;
  - Q and E step sideways.

  Each key gives full input. The robot eases it, as it does a stick.
- Pressing a drive key while the robot is requested to Stand requests Walk. Releasing every drive key leaves Walk with no input, so the robot holds its feet. Stand (1) stops walking. In Sleep and Calibrate the drive keys do nothing.
- Keys for the walking settings, sent in every command frame while in control:
  - Z and X lower and raise the speed dial by 10;
  - C and V lower and raise the body height by 10;
  - G selects the next gait;
  - T switches between the dynamic and fixed stride.

  They start as the bridge's do: speed 44, height 0, Tri, dynamic.
- Losing the window's focus, releasing control or losing the connection drops every held drive key, so no input stays applied.
- The header shows the gait, speed, height and stride while in control, and the key list includes the drive keys.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `desktop-calibrator`: driving from the keyboard while in control. This modifies "Observe and control" (translate and rotate follow the drive keys instead of being always zero; Walk is requested by driving) and "Keyboard controls shown" (the drive keys). Both requirements come from the in-flight `add-desktop-calibrator` change, so that change must be archived first.

## Impact

- `desktop/internal/session`: the drive state (held keys, speed, height, gait, stride), its mapping to the command frame, and the Walk request.
- `desktop/internal/ui`: the drive keys (held, with focus loss), the settings keys, the drive readout and the key list.
- No firmware, protocol or bridge change: the robot already takes these fields from any source in control.
