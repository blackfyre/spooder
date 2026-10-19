# Tasks

## 1. Part one: Perform mode, protocol 3, say hi and wiggle

- [ ] 1.1 Add `Mode::Perform` and the keyframe gesture player to `core/movement/body`, with entry from a settled Stand, return to Stand, and interruption as in design D1; host tests
- [ ] 1.2 Add protocol version 3 (gesture id and counter in the command, gesture in telemetry), counter baselines after a link gap in `link_receiver`, and regenerate the golden vectors deliberately (D4); host tests
- [ ] 1.3 Move the Go protocol to version 3 against the golden vectors; session and telemetry show the gesture
- [ ] 1.4 Script say hi and wiggle (translate sway only); host tests for reachability, servo range and stability (D2)
- [ ] 1.5 Bridge: the L2 gesture layer (D5) with host tests; the screen shows the gesture
- [ ] 1.6 Desktop: gesture buttons and keys 4–8 while in control; e2e test playing each gesture on the virtual robot
- [ ] 1.7 Verify every gate (`mise run test`, both robot builds, `build-bridge`, `desktop-test`, `desktop-build`, `e2e`); flash both boards together and play both gestures on the supported robot

## 2. Part two: body geometry, dance, attack and ball

- [ ] 2.1 Derive each leg's mount position and angle from the upstream STL files (or measurements) into `core/movement/body_geometry`, with a golden file the desktop legs pane reads (D3)
- [ ] 2.2 Body pose (yaw, pitch, roll, translation) with planted feet in the player; host tests
- [ ] 2.3 Script dance, attack and ball (with unfold); host tests for every keyframe at both height extremes
- [ ] 2.4 Rehearse all five on the virtual robot; then on the supported robot, attack and ball last
- [ ] 2.5 Update the skills, readme controls and changelog
