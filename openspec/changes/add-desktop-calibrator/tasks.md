# Tasks

## 1. Module and protocol

- [x] 1.1 Create the `desktop/` Go module (design D1) with Go, Gio and `go.bug.st/serial` pinned (design D2), and the `desktop-build`, `desktop-run` and `desktop-test` mise tasks including the dependency check. Verify `mise run desktop-test` on an empty test and `mise run desktop-build`.
- [x] 1.2 `internal/protocol`: frames, CRC and the incremental decoder for version 2. Verify `go test` against `test/golden/protocol_v2.txt` (decode every vector and re-encode it byte for byte), corruption rejection and resynchronisation.
- [x] 1.3 `internal/link`: the stream split into frames and text lines, the 20 ms writer, port discovery by USB ID, and reconnect (design D3). Verify `go test` with an in-memory port: interleaved text and frames, a stray 0xA5, partial reads, and reconnect.

## 2. Models (no window)

- [x] 2.1 `internal/session` (design D4): observe and control, e-stop, mode and clear counters, stale telemetry after 1 s, and the source and conflict flags. Verify `go test` for each requirement scenario that does not need hardware.
- [x] 2.2 `internal/calib` (design D5). Verify `go test` for pending, resend with the same id, confirmation, clamps and rejections shown as reported, and editing disabled outside control or Calibrate.
- [x] 2.3 `internal/export` (design D6). Verify `go test`: the golden test against `test/golden/calibration_defaults.cpp`, the checkout check, the diff, and an atomic write that touches only the one file.

## 3. User interface

- [x] 3.1 Copy and adapt `internal/appearance` and the theme scaffolding from `bopen`; the Blackfyre tokens, embedded fonts with licences, and the tactile button widget (design D7). Verify the build and theme tests for both palettes.
- [ ] 3.2 Header (port picker, connection, control toggle, e-stop with Space), telemetry strip and log pane. Verify the build, then on the board: connect, log, telemetry, unplug and replug (requires the Servo 2040 on USB).
  - Built, and tested headless (`internal/ui` window tests; off-screen renders with `SPOODER_SCREENSHOTS`). The board check is still open.
- [x] 3.3 Joint editor (6 × 3 cards, pointer and keyboard nudges, pending markers) and the export pane (diff, confirm, write, copy). Verify the build.

## 4. Verification and documentation

- [x] 4.1 Run `mise run desktop-test` and `mise run desktop-build` from a clean state, plus `mise run test` for the golden files. Verify all pass.
- [ ] 4.2 Bring up on the hardware (requires the user), recording results here:
  - observe while the gamepad drives; e-stop from the app while observing;
  - take control on the bench without the bridge; Calibrate; nudge one servo and watch the rate limit; release control and see the robot sleep;
  - the conflict with the bridge's gamepad connected;
  - export into the checkout, rebuild and flash, and confirm the robot reports the same table.
- [x] 4.3 Update `CLAUDE.md` (the desktop app, its tasks and dependencies) and `spooder-hardware-check` (the bench calibration workflow with the app, `dialout`, Linux packages). Verify that every symbol and path mentioned exists (`grep`).
