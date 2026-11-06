# Tasks

## 1. Driving view gesture

- [x] 1.1 Add the GESTURE cell to the driving view (D1). Verify with `go test ./cmd/spooder-handheld` in `desktop/handheld` that `driveViewCells` names the gesture from fresh telemetry, `None` without one and `—` without telemetry, and that the test fails with the cell removed; and that the layout test still fits the driving view's cells and faults.

## 2. Pressed hint keys

- [x] 2.1 Give the REQUESTED panel's hint keys their controls and draw them pressed while held (D2). Verify with `go test ./cmd/spooder-handheld` that `hintPressed` presses a held key, not an unheld one, not Start, X or Y with Select held, not A, B, X or Y with L2 held, and the D-pad with Select held; and that every requested cell's keys name a control.

## 3. Supply bands

- [x] 3.1 Add `core/status/supply_band` (D3), `test/golden/supply_bands.txt` and `test/test_supply_band` (`SPOODER_SUPPLY_BANDS_GOLDEN` names another copy); list the source in `platformio.ini` and `firmware/bridge/main/CMakeLists.txt`. Verify `mise run test` passes, and fails with `SPOODER_SUPPLY_BANDS_GOLDEN` naming a scratch copy with one band changed.
- [x] 3.2 Add the Go copy, `desktop/handheld/internal/supplyband`, with its golden test. Verify `go test ./internal/supplyband` passes, and fails with `SPOODER_SUPPLY_BANDS_GOLDEN` naming the same changed copy.
- [x] 3.3 Carry the newest diagnostics' limits into the handheld's screen model and colour the robot panel's supply well and the driving view's supply cell (D3). Verify with `go test ./internal/app` that a diagnostics frame sets the limits and they stay while diagnostics are stale, and with `go test ./cmd/spooder-handheld` that the colours follow the bands, the normal colour without limits and muted while stale.
- [x] 3.4 Keep the diagnostics' limits on the bridge and colour the drive and power pages' supply values (D3), the drive page's held-extreme warning kept. Verify with `mise run test` (`test_status_screen`) the tones for each band, without limits and with a held-extreme warning in the ok band; `mise run build-bridge`; and render `docs/images/bridge-*.png` with `mise run bridge-screenshots`.

## 4. Documentation

- [x] 4.1 Render `docs/images/handheld-*.png` with `SPOODER_SCREENSHOTS`; update `docs/handheld.md`, `docs/screenshots.md` as needed, `desktop/CLAUDE.md` (`internal/supplyband`), `core/CLAUDE.md` (`status/supply_band`), `test/CLAUDE.md` (`supply_bands.txt`) and `CHANGELOG.md` under Unreleased. Verify by reading them against the spec deltas.

## 5. Gates

- [ ] 5.1 Run every gate, one at a time: `mise run lint`, `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run e2e`, `mise run handheld-test`, `mise run handheld-build` and `mise run handheld-package`. Verify all pass.

## 6. Audio feedback

- [ ] 6.1 Settle D4's open questions with the user; then add the implementation tasks here.

## 7. On the devices

- [ ] 7.1 With the bridge in WiFi drive and the robot on its supply, check on the TrimUI that the driving view names say hi while it plays, that the hint keys press, and that the supply values read green at rest; on the bridge's drive and power pages, that the values read green and turn amber or orange as the bands say (a lab supply turned down through 5.6 V and 5.2 V).
