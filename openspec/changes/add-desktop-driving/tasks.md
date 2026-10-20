# Tasks

## 1. Session

- [x] 1.1 Add `session.Drive`, `SetDrive`, the drive mapping in `Command()` (car layout, mirrored reversing, sidestep), and the Walk request from Stand. Clear the drive state on release of control and on disconnect. Verify with session tests for walking, steering, turning on the spot, reversing, sidestepping, the single Walk request, Sleep and Calibrate ignoring the keys, observing, and release.
- [x] 1.2 Add `StepSpeed`, `StepHeight`, `NextGait` and `ToggleStride` (in control only, clamped 0–100), sent in every frame while in control and shown in the snapshot. Verify with session tests for the "Settings" scenario and the defaults while observing.

## 2. Window

- [x] 2.1 Filter the drive keys' presses and releases, keep the held set, call `SetDrive` on each change, and clear the set on focus loss. Map Z/X, C/V, G and T to the settings through the action latch. Verify with window tests (held W gives a Walk frame; focus loss zeroes it; settings keys).
- [x] 2.2 Show the gait, speed, height and stride while in control, and add the drive and settings keys to the key list. Verify with `mise run desktop-test`, including the screenshot test at the minimum window size.

## 3. Integration and documentation

- [x] 3.1 Add an e2e test that drives the virtual robot forward and sees Walk in telemetry. Verify with `mise run e2e`.
- [x] 3.2 Update `CHANGELOG.md`, the `spooder-hardware-check` skill (desktop calibrator controls) and `CLAUDE.md` where it describes the calibrator. Run `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`, and verify that all pass.
