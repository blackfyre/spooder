# Design

## Context

See proposal.md for the motivation. What exists:

- The L2 layer is `gestureButtons` in `desktop/handheld/internal/padmap/mapping.go`, a line-for-line port of `core/gamepad_mapping`, kept in step through `test/golden/gamepad_mapping.txt`. `app.Screen.Held` already carries `padmap.HandL2` while the trigger is beyond half travel.
- The robot accepts a gesture only in a settled Stand that stays requested, and only for an id `gesture::available` accepts (`core/movement/body.cpp`); today that is say hi and wiggle (`core/movement/gesture.cpp`). In Perform the telemetry's gesture field names the gesture playing; otherwise it is 0.
- `gesture::Player` turns a script into foot targets in each leg's frame (`start`, then `advance` by whole milliseconds), from the standing point `(gait::STANCE_DISTANCE, 0, groundForHeight(height))`. The body solves each target with `kinematics::solveLegInRange` and the leg's offsets from the default calibration (`joint_calibration::loadDefaults`, `legOffsets`). The pose frame's commanded angles are those solutions, and `legview.FromFrame` and `legview.Body3` turn them into the body-frame legs the driving view draws.
- The driving view (`cmd/spooder-handheld/driveview.go`) already projects through a `viewport` built from a rectangle (`chaseViewport(x, y, w, h)`) and draws one `app.Pose` with `drawChase`; its grid, line widths, leg numbers and limit rings are sized for the full page.
- Golden files under `test/golden/` are written by a host test with `SPOODER_WRITE_GOLDEN=1 mise run test` and compared otherwise (`test_leg_pose_golden`); `test/CLAUDE.md` lists them.

## Goals / Non-Goals

**Goals:**
- One source of truth for which gestures have scripts and how they move: the firmware's own player, through a golden file both sides check.
- The overlay is drawn from the screen model only; it never changes what is sent.
- Every overlay state is testable on the host with a fixed clock.

**Non-Goals:**
- Any change to the mapping, the command frames, the protocol or the firmware.
- An overlay on the bridge's screen or in the desktop calibrator.
- New gesture scripts (dance, attack and ball stay with `add-gestures` part two).

## Decisions

### D1. The gesture golden file

`test/test_gesture_golden` (Unity, native env) builds the text and compares it with `test/golden/gestures.txt`, or writes it under `SPOODER_WRITE_GOLDEN=1`, as `test_leg_pose_golden` does; `SPOODER_GESTURES_GOLDEN` names another copy to compare with (for the "a changed value fails" check, as `SPOODER_GAMEPAD_GOLDEN` does). For each id 1 to 5 (`GESTURE_ID_COUNT` − 1) in order:

- `gesture <id> <name> scripted` or `gesture <id> <name> none`, with `scripted` exactly when `gesture::available(id)`;
- for a scripted id, `pose <id> <t_ms>` followed by 18 angles in degrees with two decimals (leg 0 to 5, coxa femur tibia each): the player started from every foot at the standing point at height 0, sampled at t = 0 and after each 50 ms `advance` until `done`, the last sample included, or until a cap of 10 s (`MAX_PREVIEW_MS`): a scripted gesture not done by then fails the test, so a gesture that holds its last pose (ball, in `add-gestures`) cannot make the generator loop and needs its preview end decided when its script lands; each foot solved by `kinematics::solveLegInRange` with the default calibration's offsets for its leg, as the body does.

A target `solveLegInRange` refuses fails the test (the scripts are checked reachable already by `test_gesture`). Comment lines start with `#`, as in the other golden files. The text is compared whole, so any change of a script, its timing, the IK or the default offsets shows as a diff to review.

Alternative considered: storing foot targets and solving IK in Go. Rejected: the commanded angles are what the pose frame carries and what the driving view already draws, and a Go IK would be one more port to keep in step.

### D2. The generated Go table

A new package `desktop/handheld/internal/gestures` holds `Parse` (the golden text to a table) and `Format` (the table to Go source), the checked-in `table_gen.go` (`var Table = [...]Gesture{...}`, each with `ID`, `Name`, `Scripted`, `StepMs` 50 and `Frames [][18]float32`), and a generator `internal/gestures/gen` run by `//go:generate go run ./gen` in the package. Its test reads the fixture (`../../../../test/golden/gestures.txt`, or `SPOODER_GESTURES_GOLDEN`) and fails when `Format(Parse(fixture))` differs from `table_gen.go`'s bytes, so a regenerated fixture without a regenerated table fails, and so does a hand edit of the table. The generator is run by hand (`go generate ./internal/gestures` in `desktop/handheld`), not by the build: no build resolves anything new.

Alternative considered: embedding the text with `go:embed` and parsing at start-up. Rejected because the fixture lives outside the nested module, which `go:embed` cannot reach, and the brief asks for a generated table.

### D3. Tiles and their buttons

The overlay owns its tile list: button, gesture id, keycap label and place. Places follow the TrimUI Smart Pro's face buttons (Nintendo layout: X top, Y left, A right, B bottom), with R2 to the right of the diamond, level with X. A test steps `padmap` with L2 beyond half travel and each tile's button and checks that the request carries that tile's gesture id, so the overlay cannot drift from the mapping without `padmap` changing.

### D4. What the overlay reads

`app.Screen` gains `Requested.Gesture` and `Requested.GestureSeq` (the desired state's gesture and gesture-request counter) and `Robot.GestureID` (the telemetry's gesture id). The overlay is open when `Held` has `HandL2`. The screen keeps the last `GestureSeq` it saw; a change lights that gesture's tile for 400 ms (the first frame after start only records it). A tile is lit as playing while `Robot.Have`, not `Robot.Stale`, and `Robot.GestureID` is its id. `STAND FIRST` shows unless the telemetry is fresh and its body mode is Stand or Perform: in Perform the robot is playing, so the notice would mislead.

### D5. A clock for the screen

`screen` takes `now func() time.Time` (the wall clock in `main`, a fixed or stepped one in tests). The animation runs from the moment the overlay opened: frame `⌊elapsed / StepMs⌋` of the gesture, then the last frame held for 600 ms, then again from frame 0. The highlight's 400 ms runs on the same clock.

### D6. One chase renderer for the page and the tiles

`drawChase` takes a rectangle and a style (camera, line scale, whether to draw the grid, the leg numbers and the limit rings, the palette: normal, lit or muted) instead of assuming the page's well; the driving view passes the style it draws today, so its picture does not change (checked by comparing its PNG before and after). Tiles draw from the body-frame legs of a `legview.Pose` built from the table's angles (every leg set), without leg numbers or limit rings and with thicker relative lines. If six legs are unreadable at tile size, the tiles get their own camera (higher and steeper than the chase camera); the camera test checks that the standing robot fits the tile's rectangle with both cameras.

### D7. Look

A scrim dims the page; the overlay is a panel in the drive screen's palette with the title `GESTURES`, the five tiles and, when needed, the `STAND FIRST` chip in the warning colour with its label. Each tile shows its pictogram, the gesture's name in capitals and its keycap. A lit tile (request or playing) uses the single orange accent with the label `PLAYING` while playing; an unscripted tile is muted with a `NOT YET` chip. Colours are never the only signal: every state has its label.

### D8. Draw time

Input is read on the same thread as the draw (`drive` in `main.go`: poll every 5 ms, draw every 50 ms), so a slow draw delays the controls the 20 ms sender tick sends. With `SPOODER_DRAW_TIMING=1` in the environment, `drive` prints to stderr, once a second, the longest `scr.draw` of that second and the longest interval between successive `SetControls` calls (the complete sampling gap: poll delay plus draw); without it, nothing changes. The budget, with the overlay open, is a sampling gap of at most 25 ms (the 5 ms poll plus a 20 ms draw), so a control change waits at most 25 ms to be read and then until the next sender tick; the on-device task checks it.

## Assumptions

- The brief asks to add to the drive screen requirement of `handheld-controller`, which only exists in the unarchived `add-handheld-controller`. The delta modifies that requirement, so `add-handheld-controller` must be archived before this change (`openspec validate` reports that archive would otherwise refuse the delta).
- "Default calibration" means the compiled defaults' offsets (`joint_calibration::loadDefaults`), as the body uses at start-up; "standing pose" means the standing point at body height 0, the mapping's starting height.
- The animation shows the script itself, not the return the robot plays when a gesture is stopped early.
- `STAND FIRST` is not shown in Perform (D4), reading the brief's "not Stand" as "cannot start a gesture".
- The highlight lasts 400 ms and the loop holds its last frame for 600 ms; both are local constants, tuned on the device if needed.
- The brief's parts form one coherent feature (the golden file is what the tiles draw); it is delivered whole.

## Risks / Trade-offs

- [Five perspective robots per frame on the device, drawn on the thread that reads the controls] → tiles draw without grid detail beyond a few lines; the on-device task checks the redraw rate (at least 10 per second) and the longest sampling gap (at most 25 ms, D8) with the overlay open.
- [Floating-point output differing between host compilers] → two decimals in the fixture; the fixture is only compared on the host test env, as `leg_pose.txt` is.
- [A new script in `add-gestures` part two] → it changes `gestures.txt`, which fails `mise run test` until regenerated and then `handheld-test` until `go generate` is run: the intended reminder.

## Follow-ups

- When dance, attack and ball gain scripts (`add-gestures` part two), regenerate `gestures.txt` and the Go table; the tiles follow with no code change, except that ball, which holds its pose until another request, needs a preview end (D1's cap fails until one is chosen).
