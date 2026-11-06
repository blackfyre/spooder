# Tasks

## 1. The gesture golden file

- [ ] 1.1 Add `test/test_gesture_golden` (design D1: `gesture::Player` from the standing point at height 0, sampled every 50 ms until done, solved with `kinematics::solveLegInRange` and the default calibration's offsets; `SPOODER_WRITE_GOLDEN=1` writes, `SPOODER_GESTURES_GOLDEN` names another copy) and generate `test/golden/gestures.txt`; add it to the golden files in `test/CLAUDE.md`. Verify `mise run test` passes; that ids 1 and 2 read `scripted` with pose lines and ids 3 to 5 `none` without; that a second `SPOODER_WRITE_GOLDEN=1 mise run test` leaves the file unchanged (`git diff --exit-code test/golden/gestures.txt`); and that the test fails with `SPOODER_GESTURES_GOLDEN` naming a scratch copy with one angle changed.

## 2. The handheld's gesture table

- [ ] 2.1 Add `desktop/handheld/internal/gestures` (D2: `Parse`, `Format`, the generator `gen` behind `//go:generate`, the generated `table_gen.go`) and its golden test. Verify with `go test ./internal/gestures` in `desktop/handheld` that the table equals `Format(Parse(test/golden/gestures.txt))`, that say hi and wiggle are scripted with their frames and dance, attack and ball are not, that the test fails with `SPOODER_GESTURES_GOLDEN` naming a scratch copy with one angle changed and with one `none` turned `scripted`, and that `go generate ./internal/gestures` leaves `table_gen.go` unchanged (`git diff --exit-code`).

## 3. The overlay

- [ ] 3.1 Add `Requested.Gesture`, `Requested.GestureSeq` and `Robot.GestureID` to `app.Screen` (D4). Verify with `go test ./internal/app` that L2 beyond half travel and A advance `GestureSeq` by one with the say-hi id, that the frames sent are unchanged by reading the screen, and that telemetry reporting wiggle sets `GestureID` 2.
- [ ] 3.2 Make `drawChase` draw into a given rectangle with a style (D6), the driving view passing today's. Verify `TestChaseCamera` and `TestDrawsEveryState` pass, and that `handheld-driveview.png` rendered with `SPOODER_SCREENSHOTS` before and after the change is byte-identical (`cmp`).
- [ ] 3.3 Add the overlay (D3 to D7): the screen's clock, the tile list, the animation from `gestures.Table`, the request highlight, the playing tile, `NOT YET` and `STAND FIRST`, open on either page while L2 is held. Verify with `go test ./cmd/spooder-handheld` that each tile's button with L2 through `padmap` requests the tile's gesture; that with a fixed clock the shown frame steps every 50 ms, holds the last frame for 600 ms and starts again; that a `GestureSeq` change lights its tile for 400 ms and the first frame lights none; that the playing tile is lit only from fresh telemetry; that `STAND FIRST` shows with no telemetry, stale telemetry, Sleep and Walk, and not with Stand or Perform; that the layout test fits the overlay, its tiles, names, keycaps, `NOT YET` and `STAND FIRST` inside 1280x720 with no overlap, and the camera test fits the standing robot in a tile; and that `TestDrawsEveryState` draws a new `gestures` case (overlay open over the panels page, wiggle playing, telemetry in Perform).

## 4. Documentation

- [ ] 4.1 Render `docs/images/handheld-gestures.png` with `SPOODER_SCREENSHOTS`; update `docs/controls.md` and `docs/handheld.md` (the L2 layer's buttons and gestures, which are not scripted yet, the overlay and its screenshot), `docs/screenshots.md`, `desktop/CLAUDE.md` (`internal/gestures`, the generated table and how to regenerate it) and `CHANGELOG.md` under Unreleased. Verify by reading them against the spec delta, and that `grep` finds `handheld-gestures.png` in `docs/handheld.md` and `docs/screenshots.md` and `gestures.txt` in `test/CLAUDE.md` and `desktop/CLAUDE.md`.

## 5. Gates

- [ ] 5.1 Run every gate from the worktree, one at a time: `mise run lint`, `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build`, `mise run e2e`, `mise run handheld-test`, `mise run handheld-build` and `mise run handheld-package`. Verify all pass.

## 6. On the handheld

- [ ] 6.1 With the bridge in WiFi drive and the robot board in bench simulation, install the new `Spooder.pak` on the TrimUI. Verify: holding L2 opens the overlay on both pages and releasing it closes it; the say-hi and wiggle tiles animate and the six legs are readable, the others read `NOT YET`; the screen still redraws at least 10 times a second with the overlay open; `STAND FIRST` shows in Sleep and goes after Y stands the robot; L2 + A lights the say-hi tile and the tile stays lit while telemetry reports say hi playing.
