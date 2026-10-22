# Proposal

## Why

Gestures lift legs and shift the body, and the planned dance, attack and ball move it further. Whether the robot stays on its feet depends on where its centre of gravity falls among the feet on the ground, not on where the body's centre is. The eighteen servos (about 1 kg) outweigh the body and the battery, so the centre of gravity moves with the legs. The robot is being rebuilt; until it can be weighed and tried, an estimate is the only way to judge a pose's balance, on the virtual robot as on the real one.

## What Changes

- **Mass model.** A hardware-free estimate in `core/movement/centre_of_gravity`: point masses for the body (frame and electronics, battery not included), the battery, and each leg's three servos and three links, placed by forward kinematics of the joint angles on the schematic leg mounts. The servo (55 g) and the battery (284 g) are weighed; the other masses are estimates from typical printed-part weights, in one table; weighing the parts later only changes that table.
- **Battery.** The default model has no battery fitted. The 2S 5000 mAh pack can be added as a mass in the centre body.
- **Support margin.** The polygon of the feet on the ground (their convex hull, from three feet), and the distance of the centre of gravity's ground projection inside it.
- **Golden vectors.** `test/golden/centre_of_gravity.txt`, written from the C++ model, pins the desktop's Go copy, as `leg_pose.txt` does for the kinematics.
- **Four-view window.** It marks the estimated centre of gravity in every view, drops it to the ground in the front, left and isometric views, draws the support polygon in the top view, and shows the position, the total mass and the margin, coloured by how close it is to the edge. A button there fits or removes the battery; the window opens without it.
- **Gesture test.** The host test of the scripted gestures checks the estimated centre of gravity, with and without the battery, besides the body's centre.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `desktop-leg-view`: the four-view window shows the estimated centre of gravity and the support polygon.

## Impact

- New `core/movement/centre_of_gravity.{h,cpp}` with host tests and the golden generator (`test/test_centre_of_gravity`).
- `desktop/internal/legview` (the Go model, tested against the golden file) and `desktop/internal/ui/views.go`.
- No change to the firmware, the link protocol or the robot's behaviour.
