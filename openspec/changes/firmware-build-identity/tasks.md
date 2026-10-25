## 1. Identity and banner text

- [ ] 1.1 Add `cmake/build_identity.cmake` (script mode: `git describe --always --dirty --abbrev=12` of the repository root, `unknown` on any failure, the generated source rewritten only when its content changes). Verify by running it twice with `cmake -P` into a scratch directory (the file's modification time unchanged the second time), once in a clean and once in a dirty tree, and once with `GIT_DIR` pointing at an empty directory (`unknown`).
- [ ] 1.2 Add `core/identity/build_identity.{h,cpp}` (`banner` and `fit`, the `ID` declaration) and list it in the root `platformio.ini`; verify with a new `test/test_build_identity` covering the banner form, an empty id, non-printable bytes, the 40-character cut and `fit` keeping the end.

## 2. Robot and virtual robot

- [ ] 2.1 In `core/robot/robot_tick`, take the identity in `init` (defaulting to `"unknown"`), add `Inputs::usbConnected`, and log the banner on the first tick (before the servo profile line) and on each false-to-true change of `usbConnected`. Verify with `test/test_robot_tick` (banner first, then the profile line; repeated on a reconnect; not repeated while connected) via `mise run test`.
- [ ] 2.2 Wire the generated source into `firmware/servo2040/CMakeLists.txt` and pass the identity and `stdio_usb_connected()` from `main.cpp`. Verify that `mise run build` and `mise run build --serial` pass, that a second build with no change recompiles nothing (Ninja reports no work beyond the identity target), and that `strings build/servo2040/spooder.elf` shows the current describe output.
- [ ] 2.3 Wire it into `tools/virtual-robot/CMakeLists.txt` and pass the identity and `board.usb.connected()` from its `main.cpp`. Verify that `mise run virtual-robot` passes and `mise run e2e` passes.

## 3. Bridge

- [ ] 3.1 Wire the generated source into `firmware/bridge/main/CMakeLists.txt` and add `build_identity.cpp` there; print `spooder bridge: build <id>` first in `setup()` and in `setup_mode_start`, and add `Build <fitted id>` as the first start-up log line. Verify that `mise run build-bridge` passes and the size check still passes, and that after an empty commit a second `mise run build-bridge` puts the new identity in the image (`strings` on the built `.elf`).

## 4. Documentation and gates

- [ ] 4.1 Update the `spooder-hardware-check` skill (the robot banner on connect, the bridge console line and the start-up log line), `core/CLAUDE.md` (the new module) and `CHANGELOG.md` under Unreleased. Verify by reading them against the spec deltas.
- [ ] 4.2 Run the gates: `mise run test`, `mise run build` with and without `--serial`, `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`. Verify that all pass.

## 5. On the boards

- [ ] 5.1 Flash the robot (`mise run upload`). Verify that `mise run monitor`, opened after boot, shows `spooder robot: build <id>` as its first line, matching `git describe --always --dirty --abbrev=12`, and again after closing and reopening it.
- [ ] 5.2 Flash the bridge (`mise run upload-bridge`). Verify that `mise run monitor-bridge` and the start-up log on the screen show the build line with the same identity, and that the console shows it again in setup mode.
