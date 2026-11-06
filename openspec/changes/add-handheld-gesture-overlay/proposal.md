# Proposal

## Why

On the TrimUI handheld the gestures are a chord: while L2 is held beyond half travel, A, B, X, Y and R2 request say hi, wiggle, dance, attack and ball. The drive screen's footer only says `L2 GESTURES`, so the driver has to remember which button plays which gesture, cannot tell that only say hi and wiggle have scripts yet, and cannot see why a request does nothing (the robot plays a gesture only from a settled Stand).

## What Changes

- While L2 is held beyond half travel, a modal gesture overlay opens over the drive screen, on either page, and closes when L2 is released. The controls and the command frames do not change.
- Five tiles laid out like the TrimUI's buttons (A, B, X, Y as a diamond, R2 beside it), each with an animated pictogram, the gesture's name and its keycap, in the drive screen's Blackfyre dark look.
- A scripted gesture's tile loops that gesture: the robot drawn at thumbnail size from poses sampled from the firmware's own script, with the driving view's chase-camera renderer. A gesture without a script shows a still, muted standing robot marked `NOT YET`.
- A gesture request made while the overlay is open lights its tile briefly; the tile of the gesture the newest telemetry reports playing stays lit while it plays; `STAND FIRST` shows while the robot cannot start a gesture (its body mode neither Stand nor Perform, or no fresh telemetry).
- A new golden file, `test/golden/gestures.txt`, written and checked by a new C++ host test from `gesture::Player` and the inverse kinematics: per gesture id, whether it has a script, and all six legs' joint angles every 50 ms through it. The handheld embeds a generated Go table of it, and a Go test keeps the table and the file equal, so availability and animation have one source.
- No protocol, firmware behaviour or input-mapping change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `handheld-controller`: the drive screen requirement gains the gesture overlay. The capability is added by `add-handheld-controller`, merged but not yet archived; this delta builds on its text.

## Impact

- `desktop/handheld/cmd/spooder-handheld`: the overlay (`screen.go`, a new `gestures.go`), the chase-camera renderer drawing into a given rectangle (`driveview.go`), a clock the screen takes, the layout and screenshot tests.
- `desktop/handheld/internal/app`: the screen model carries the requested gesture and its request counter, and the reported gesture's id.
- `desktop/handheld/internal/gestures` (new): the generated table, its generator and its golden test.
- `test/test_gesture_golden` (new) and `test/golden/gestures.txt` (new); `test/CLAUDE.md`.
- `docs/controls.md`, `docs/handheld.md`, `docs/screenshots.md`, `docs/images/handheld-gestures.png`, `desktop/CLAUDE.md`, `CHANGELOG.md`.
- No change in `core/`, the firmwares, the protocol or `internal/padmap`'s behaviour.
