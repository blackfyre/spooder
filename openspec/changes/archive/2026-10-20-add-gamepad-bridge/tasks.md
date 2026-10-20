# Tasks

## 1. Shared protocol and mapping (host-tested)

- [x] 1.1 Create `core/protocol/` (frame layout, CRC-16/CCITT-FALSE, an encoder, an incremental decoder, and the command and telemetry structs from design D2) and add it to the native env. Verify host tests for:
  - round trips of both frame types;
  - rejection of a single-byte corruption;
  - resynchronisation over garbage;
  - rejection of unknown versions and types;
  - sequence wrap.
- [x] 1.2 Create `core/gamepad_mapping/` (`PadState` in, desired command state out, with the edge and hold handling from design D5). Verify host tests for:
  - every row of the spec's mapping table;
  - gait wrap;
  - the Calibrate combo needing a 1 s hold;
  - combination priority over the Stand/Walk toggle;
  - the 8 % dead zone and continuity at its edge;
  - disconnect requesting Sleep with zero input;
  - reconnect starting in Stand with kept settings.
- [x] 1.3 Extend `core/protection` with the `estop` input and the `EmergencyStop` fault. Verify host tests for:
  - output inhibited on the same tick;
  - release without a clear keeping output off;
  - clearing after release.

## 2. Robot side (Servo 2040)

- [x] 2.1 Implement `firmware/servo2040/src/link`: UART1 on GPIO20 (TX) and GPIO21 (RX) with the RX pull-up, 115200 8N1, an IRQ-driven RX ring buffer, and decoding of the newest valid command each tick. Verify the build. The decode path must be covered by the 1.1 tests.
- [x] 2.2 Integrate the link into `main`:
  - arrival-time `issuedAtMs`;
  - clear-counter edges turned into clear requests;
  - the e-stop and controller-connected flags fed to protection and to command overrides;
  - telemetry every fifth tick;
  - `SPOODER_SERIAL_COMMANDS` made a build option, off by default.
  
  Verify `mise run build` with and without the option, with no warnings.

## 3. Bridge firmware (TTGO T-Display v1.1)

- [x] 3.1 Create `firmware/bridge/` from the Bluepad32 ESP-IDF + Arduino template, pinning the template commit, the pioarduino platform version and the Bluepad32 release, with WiFi disabled. Add the `build-bridge`/`upload-bridge`/`monitor-bridge` mise tasks. Verify that `mise run build-bridge` succeeds and record the pinned versions in `design.md`.
- [x] 3.2 Add the display using LovyanGFX for the ST7789 135×240 on the documented pins. Verify the build. If LovyanGFX doesn't build or run with Arduino 3.x, switch to `esp_lcd`, and record the decision in `design.md`.
- [x] 3.3 Implement the link on UART1 (GPIO27 TX, GPIO26 RX): command TX every 20 ms from a dedicated link task above `loop()` (design D3), and telemetry RX and decode. Verify the build.
- [x] 3.4 Wire up Bluepad32: a single controller slot, filling `PadState` from `Controller` and feeding `core/gamepad_mapping`. The GPIO35 button toggles e-stop. Verify the build.
- [x] 3.5 Implement the status screen (the spec's content list, link-loss state, partial redraws at most every 250 ms). Verify the build.

## 4. Verification and documentation

- [x] 4.1 Run `mise run test`, `mise run build` and `mise run build-bridge` from a clean checkout. Verify all three pass.
- [ ] 4.2 Bring up the hardware (requires the user at the hardware), recording results in this change:
  - the link on a bench cable: screen telemetry, then a cable pull with the robot asleep after 1 s and the screen showing the link lost;
  - a gamepad connect and disconnect, checking the Sleep request;
  - every row of the mapping table, checking the Y-axis sign;
  - e-stop latency: output off within 20 ms, measured on a scope or logic analyser;
  - command frame timing under heavy screen redraws;
  - finally, a walk with all 18 servos, the robot on a stand.
- [x] 4.3 Update `CLAUDE.md` (two firmwares, link, mise tasks), `spooder-firmware-shape` (link boundary, J6 UART pins, bridge architecture) and `spooder-hardware-check` (bridge flashing, link wiring, bring-up order). Add a gamepad mapping reference to the skills. Verify that every symbol and path mentioned exists (`grep`).
