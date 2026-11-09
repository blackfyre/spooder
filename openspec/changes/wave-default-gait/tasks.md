## 1. Default gait

- [x] 1.1 Start the gamepad mapping (`core/gamepad_mapping`) and its Go port (`desktop/handheld/internal/padmap`) at Wave; move every expected gait in `test/golden/gamepad_mapping.txt` one step on, and test that the start is Wave and `l1` reaches Tri
- [x] 1.2 Start the desktop calibrator's keyboard driving (`desktop/internal/session`) at Wave, with a test
- [x] 1.3 Changelog and the docs that name the starting gait
- [x] 1.4 One starting gait for the robot as well (design D2): `gait::START_GAIT` for `body`'s initial gait, `MovementCommand`'s default (the no-source command, the serial keys) and `gamepad_mapping::START_GAIT`; the WiFi drive's command without a live handheld, the bridge's first frames and `spooder-wifi-probe` start at Wave; tests

## 2. Gates

- [x] 2.1 `mise run lint`, `test`, `build`, `build-bridge`, `desktop-test`, `e2e`, `handheld-test`
- [ ] 2.2 With the robot, the bridge and the pak flashed: the bridge screen, the pak's robot and requested values and the calibrator show Wave after start-up, before and after a controller is in control, also with no servo supply (requires the user)
