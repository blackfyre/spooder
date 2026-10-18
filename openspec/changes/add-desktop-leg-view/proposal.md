# Proposal

## Why

Calibrating a joint means judging whether the physical leg matches the pose the robot thinks it commands. The desktop calibrator shows numbers only: offsets and pulse end points, but not the angles the robot is commanding or the pulses it writes. A drawing of each leg as the robot commands it lets the operator compare it with the real leg by eye, see which joint is at a servo limit, and check that a nudge moves the joint they expect.

## What Changes

- **Pose frame (link protocol, robot to host).** A new frame type carries, for each of the 18 joints, the commanded angle from the IK before calibration, the servo angle after the joint's offset, and the pulse width last set for the output, plus whether servo output is on.
  - Sent on the USB link only, at 10 Hz; never on UART1, so the bridge and its bandwidth are unaffected.
  - An addition to the version 2 protocol of `add-live-joint-calibration` (design D3): the version stays 2. Golden vectors are appended to `test/golden/protocol_v2.txt`.
- **Desktop leg view.** A "legs" pane in the calibrator draws, from the latest pose frame:
  - the selected leg from the side (coxa, femur, tibia in the leg's vertical plane) with a top-view inset of its coxa angle;
  - a top view of all six legs on a schematic body;
  - each joint's pulse, a marker on joints near or at the 0°/180° servo limits, and the joint selected in the editor highlighted;
  - "no pose" instead of old geometry when no pose frame has arrived for 1 second.
- **Forward kinematics on the desktop**, tested against IK vectors produced by a C++ host test (`test/golden/leg_pose.txt`).

## Capabilities

### New Capabilities

- `pose-telemetry`: The robot's report of its commanded joint angles, servo angles and output pulses on the USB link: contents, rate, USB only, and its place in the link protocol.
- `desktop-leg-view`: The desktop calibrator's drawing of the legs from the reported pose: side and top views, limit markers, pulses, selection and staleness.

### Modified Capabilities

None. The frame is an addition to the protocol designed in `add-live-joint-calibration` (not yet archived) and to the calibrator of `add-desktop-calibrator`; neither change's requirements are altered.

## Impact

- `core/protocol`: frame type 5 (`Pose`), encoder and decoder; `link_receiver::makePose`. Golden vectors appended.
- `firmware/servo2040`: `usb_link::sendPose`, `servo_output::pulses()`, a few lines in `main.cpp`'s loop.
- `desktop/internal/protocol`: the pose frame. `desktop/internal/legview` (new): forward kinematics and view geometry. `desktop/internal/session`: the latest pose and its age. `desktop/internal/ui`: the legs pane and screenshot states.
- `test/`: protocol and link-receiver tests, `test_leg_pose_golden` writing `test/golden/leg_pose.txt`.
- The bridge firmware is rebuilt (it links `core/protocol`) but unchanged in behaviour.
