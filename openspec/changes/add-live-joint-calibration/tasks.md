# Tasks

## 1. Hardware-free core (host-tested)

- [x] 1.1 Create `core/movement/joint_calibration` and `calibration_defaults.cpp` in the format of design D6, seeded with today's `LEG_OFFSETS`, 500/2500 µs and normal direction. Remove `kinematics::LEG_OFFSETS`; `solveLeg` and the pulse conversion use the table. Verify host tests for:
  - the spec's pulse scenarios (calibrated range, reversed joint, clamping) and the offset scenario;
  - the existing IK and body tests passing unchanged with the defaults;
  - the rate limit (a 20° offset change takes at least 1/3 s) and convergence;
  - `test/golden/calibration_defaults.cpp` equal to the committed defaults file.
- [x] 1.2 Protocol version 2 (design D3): calibration set and report frames, telemetry flags, `MAX_FRAME_SIZE` 135, and `test/golden/protocol_v2.txt`. Verify host tests for round trips, rejection of version-1 frames, the golden vectors, and the existing tests updated.
- [x] 1.3 Calibration guards in `link_receiver` (design D2). Verify host tests for rejection outside Calibrate or without output, the clamps and minimum span, a repeated request id, and the report contents and result codes.
- [x] 1.4 Source arbitration (design D5). Verify host tests for every row of the arbitration rule, the conflict flag, per-source request counters across a source switch, the e-stop from the idle source, and a clear accepted only from the source in control.

## 2. Robot (Servo 2040)

- [x] 2.1 Requested and applied tables in `main`, `joint_calibration::step` each tick, applied values to the IK and `servo_output`. Verify `mise run build` (both variants) with no warnings.
- [x] 2.2 Calibration sets from either link through the guards; reports after each set and at 1 Hz in Calibrate (capped at 5 Hz per link); a 256-byte whole-frame TX queue on UART1. Verify the build.
- [ ] 2.3 `usb_link` (design D4) in default builds, with arbitration in `main` and telemetry and reports on both links. Verify the build, and on the board that `mise run monitor` still shows readable debug text (requires the Servo 2040 on USB).

## 3. Bridge

- [x] 3.1 Move the bridge to protocol version 2: decode reports and the new telemetry flags, and show a source conflict on the status screen. Verify `mise run build-bridge` and the status screen tests.

## 4. Verification and documentation

- [x] 4.1 Run `mise run test`, `mise run build` (with and without `--serial`) and `mise run build-bridge` from a clean state. Verify all pass with no warnings in our sources.
- [ ] 4.2 Bring up on the hardware (requires the user): flash both boards, and confirm the robot behaves as before with the defaults, the bridge link works at version 2, and debug text is still readable on USB.
- [x] 4.3 Update `CLAUDE.md`, `spooder-firmware-shape` (calibration table, protocol v2, USB link and arbitration), `spooder-hardware-check` (flash both boards together) and `spooder-leg-kinematics` (calibration). Verify that every symbol and path mentioned exists (`grep`).
