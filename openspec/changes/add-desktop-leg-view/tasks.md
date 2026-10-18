# Tasks

## 1. Protocol

- [x] 1.1 Add frame type 5 (`Pose`) to `core/protocol` (encoder, decoder, payload size), with round-trip and decoder tests in `test/test_protocol`; verify `mise run test` passes
- [x] 1.2 Append pose vectors to `test/golden/protocol_v2.txt` from `test/test_protocol_golden`, leaving every existing line unchanged; verify with `git diff` that the file only gains lines and `mise run test` passes
- [x] 1.3 Add the pose frame to `desktop/internal/protocol` (encode, parse, fields, summary) and require a pose vector in the golden test; verify `mise run desktop-test` passes

## 2. Robot

- [ ] 2.1 Add `link_receiver::makePose` with a host test of commanded and servo angles, pulses, rounding and saturation; verify `mise run test` passes
- [ ] 2.2 Add `servo_output::pulses()`, `usb_link::sendPose` and the 10 Hz call in `main.cpp` (two ticks after telemetry); verify `mise run build` and `mise run build --serial` build without warnings and `mise run build-bridge` builds

## 3. Leg model

- [ ] 3.1 Add `test/test_leg_pose_golden`, writing `test/golden/leg_pose.txt` from `kinematics::solveLeg`; verify `mise run test` passes against the committed file
- [ ] 3.2 Add `desktop/internal/legview`: forward kinematics, side and top projections, leg mounting, limit classification and "not set"; verify its tests pass, including the round trip against `leg_pose.txt` and the calibration pose reading 0°/90°/90°
- [ ] 3.3 Keep the latest pose and its age in `session` (forgotten on a new connection, stale after 1 s); verify the session tests pass

## 4. Desktop UI

- [ ] 4.1 Add the "legs" pane (side view with coxa inset, top view, pulses, servo angles, limit markers, editor selection, "no pose"); verify `mise run desktop-test` and `mise run desktop-build` pass
- [ ] 4.2 Add leg-view states to the screenshot test (calibration pose, walking, a joint at its limit, no pose; light and dark), render them and check the layout by eye

## 5. Wrap-up

- [ ] 5.1 Update `CLAUDE.md` and the skills minimally (pose frame, legs pane); verify `openspec validate add-desktop-leg-view` passes
