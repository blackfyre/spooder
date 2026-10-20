# Tasks

## 1. Detector

- [x] 1.1 Add `core/gamepad_mapping/rumble` (D1, D2) with host tests for every event, the baselines, priority, the repeat limit and no gamepad
- [x] 1.2 Verify `mise run test`

## 2. Bridge

- [x] 2.1 Step the detector in `loop()` and play its requests with `playDualRumble`; verify `mise run build-bridge`

## 3. Documentation and hardware

- [x] 3.1 Update the hardware-check skill (the rumble table), the readme and the changelog
- [ ] 3.2 On the robot: pull the e-stop, unplug the link, request a gesture while walking; feel each pattern on the SF30 Pro
