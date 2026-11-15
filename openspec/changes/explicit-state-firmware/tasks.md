# Tasks

## 1. The emergency-stop rule in the core

- [ ] 1.1 Add `handheld_controller::commandStop(const Controller &c, bool bridgeStop)` to `core/wifi_drive/handheld_controller.{h,cpp}` (D3). Verify with new cases in `test/test_handheld_controller` via `mise run test`: toggle on with no handheld frame (stop), toggle off with an accepted frame carrying the flag (stop), the flag kept after a 151 ms lapse (stop), a later accepted frame without the flag (no stop), and neither (no stop); and `mise run lint`.

## 2. One boot mode and one stop in the bridge

- [ ] 2.1 Replace `start()`, `startPortal()` and `startHandheld()` with `robot_link::start(drive_mode::Mode)` and the `portalMode`/`handheldMode` bools with one `mode`, every branch an exhaustive `switch` without `default:` (D1); update the callers in `bridge.cpp`, `setup_mode.cpp` and `wifi_drive.cpp`. Pin `bridge_boot_mode` to `drive_mode::Mode` with `static_assert`, drop the `default:` in `bridge_boot_mode_select`, and make `app_main` an exhaustive `switch` (D2). Verify `mise run build-bridge` passes; that `grep -n "portalMode\|handheldMode\|startPortal\|startHandheld\|default:" firmware/bridge/main/link.cpp firmware/bridge/main/wifi_drive.cpp firmware/bridge/main/main.c` finds nothing; and that temporarily adding a fourth `drive_mode::Mode` enumerator, then a fourth `bridge_boot_mode` value, then swapping two `bridge_boot_mode` values each fail `mise run build-bridge` (each reverted).
- [ ] 2.2 Compute the commanded emergency stop in one `commandedStop()` under the lock, used by `sendCommand` and `status()`, with `Status::emergencyStop` documented as the stop the frames carry; remove `HandheldView::emergencyStop` and the OR in `wifi_drive.cpp`; fix the `Inputs::emergencyStop` comment in `core/status/status_screen.h` (D3). Verify `mise run build-bridge`, `mise run test` and `mise run lint` pass, and that `grep -n "emergencyStop(" firmware/bridge/main` finds `handheld_controller` called only inside `commandedStop()`.
- [ ] 2.3 Replace `receive()`'s function statics with the `RxGap` state and its transition function (D4). Verify `mise run build-bridge` passes and `grep -n "static" firmware/bridge/main/link.cpp` shows no function-local `static` in `receive()`.

## 3. Virtual robot and documentation

- [ ] 3.1 Make `WifiBridge::beforeTick` use `handheld_controller::commandStop` (D3). Verify `mise run virtual-robot` and `mise run e2e --wifi-bridge` pass.
- [ ] 3.2 Record the change in `CHANGELOG.md` under Unreleased (Changed), and name `robot_link::start(drive_mode::Mode)` and the shared stop in `firmware/bridge/CLAUDE.md` and the boot-path section of `.claude/skills/spooder-firmware-shape/SKILL.md`. Verify `openspec validate explicit-state-firmware` passes and `grep -rn "startPortal\|startHandheld" firmware .claude/skills CHANGELOG.md` finds nothing outside the changelog entry.

## 4. Hardware check

- [ ] 4.1 Flash the bridge (`mise run upload-bridge`) and, with the robot on the bench, follow `spooder-hardware-check` for each boot path. Verify on the boards: with the switch open the bridge boots Bluetooth drive, a gamepad drives, and a GPIO35 press shows `E-STOP` on the screen and inhibits the robot; with the switch closed it boots WiFi drive, `spooder-wifi-probe` drives, the probe's e-stop flag alone shows `E-STOP` on the screen and inhibits the robot, and the flag stays shown after the probe pauses past the lapse; holding the left button during the logo enters setup mode, the portal shows `estop` following GPIO35 and calibration edits reach the robot; and in every path the link page's frame counts rise with no discarded frames growing while the robot runs.
