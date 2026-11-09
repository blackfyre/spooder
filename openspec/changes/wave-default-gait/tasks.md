## 1. Default gait

- [x] 1.1 Start the gamepad mapping (`core/gamepad_mapping`) and its Go port (`desktop/handheld/internal/padmap`) at Wave; move every expected gait in `test/golden/gamepad_mapping.txt` one step on, and test that the start is Wave and `l1` reaches Tri
- [x] 1.2 Start the desktop calibrator's keyboard driving (`desktop/internal/session`) at Wave, with a test
- [x] 1.3 Changelog and the docs that name the starting gait

## 2. Gates

- [x] 2.1 `mise run lint`, `test`, `build`, `build-bridge`, `desktop-test`, `e2e`, `handheld-test`
- [ ] 2.2 On the bridge and the pak: the drive screen shows Wave after start-up (requires the user)
