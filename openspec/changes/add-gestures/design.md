# Design

## Context

The body is a non-blocking state machine stepped on a fixed 5 ms grid; every mode writes foot targets in each leg's own frame, and IK, reachability checks, the servo output and the calibration table follow unchanged. Legs know their placement only as an angle (`LEG_PLACEMENT_ANGLE × ROTATION_MULTIPLIER`), used by the gait to rotate strides; no mount positions exist in `core/`. Commands carry a mode and a mode-request counter; a gesture needs its own one-shot request.

## Goals / Non-Goals

**Goals:**

- Gestures as data (keyframes), played by one generic player that every safety rule already covers.
- Reachable, limit-respecting keyframes, proven by host tests before any hardware run.
- Rehearsal on the virtual robot through the desktop legs pane.

**Non-Goals:**

- Music or beat detection for dance (fixed tempo).
- Balance control from sensors (body sensing is suspended); stability comes from conservative poses.
- User-editable gestures at run time.

## Decisions

### D1. Perform mode and the gesture player

- `body::Mode::Perform`, entered from Stand only once its transition is done (as Walk is), when a new gesture request arrives.
- A gesture is a constant table of keyframes: duration, body pose (translation x/y/z and yaw/pitch/roll in the body frame) and per-leg foot overrides (lifted or moved feet; others stay planted). The player interpolates between keyframes with an ease-in/out curve on the 5 ms grid.
- Planted feet stay fixed in the world: a body pose change becomes the inverse transform of each planted foot into its leg frame (D3).
- Part two: a keyframe may give any of the six legs a foot target of its own (ball folds all six; part one allowed two), and carries a pose (translation, yaw, pitch, roll) instead of a shift and a twist.
- Part two: the player interpolates the body pose (translation and yaw, pitch, roll, eased as above), not the feet, and places each planted foot from the pose on every 5 ms step. Interpolating the feet instead moves a planted foot along the chord of its arc under rotation, so it slides on the floor mid-move (about 2 mm for a 15° roll with a foot 200 mm from the centre, repeated on every beat of dance). Lifted feet still move from target to target in their leg frames. The rate limit measures each planted foot's move along the pose path. `twistMm` gives way to yaw. The extra cost is one pose transform per leg per step; `test_transitions_and_gestures_stay_within_the_budget` shows whether it fits the tick.
- At the end the player returns to the stand pose and the mode to Stand; ball instead holds its last keyframe until any other request, then plays its unfold keyframes into Stand. A script marks whether it holds and names its unfold script; while it holds, the body stays in Perform, and `body.cpp` sends any request other than Sleep (a mode, Stand again or another gesture) through the unfold before acting on it. A gesture request during the hold only unfolds.
- A stop before a holding script reaches its hold finishes the current keyframe, then plays the full unfold from that reached pose. The unfold first spreads the feet at the lowered height, then raises the body; it never plants at a folded pose or raises the body with the feet still drawn in. Host tests sample this return from every provisional ball keyframe at both height extremes, checking reachability and default servo travel.
- Sleep from the held ball goes straight to Sleep, as the spec requires of every mode: the body already rests on the ground, so Sleep's eased move to its rest pose carries no weight. A host test requests Sleep during the hold and checks every intermediate target is reachable and in range. Should it fail, the rule becomes unfold, then sleep, which is a spec change.
- Interruption: Sleep, a stale command, the e-stop and faults pre-empt at once, exactly as for Walk (the supervisor and protection are unchanged). Another mode request finishes the current keyframe, then returns to Stand first.

### D2. Safety bounds

- Every keyframe target is checked by `kinematics::isValidTarget` and every joint angle against the servo range with the default calibration, in host tests for every gesture at every body height.
- Static stability for gestures that lift legs: the body's centre, projected down, stays inside the support polygon of the planted feet with a margin (host test, from the body geometry).
- Speed: foot and body moves are rate-limited like Calibrate's approach, so peak servo current stays near walking levels; attack's strike is the fastest move and is bounded explicitly.
- Part two: the explicit bound is the joint speed budget (`BodyState::jointBudgetDegPerS`, from `servo_profile::SELECTED`), which already governs every 20 ms motion piece, gestures included. A keyframe may raise the foot speed limit (`MAX_FOOT_SPEED_MM_S`) for itself; attack's strike does, and the budget still caps it. No other speed limit is added. Host tests report the strike's actual duration under the selected profile.

### D3. Body geometry

- `core/movement/body_geometry`: each leg's coxa pivot position (x, y, z) and mount angle in the body frame, from the upstream STL files (or the user's measurements), with the body-centre origin and axes of the desktop legs pane.
- The desktop's `internal/legview` takes the same values (a golden file, as for `leg_pose.txt`), so the legs pane draws the real shape.
- Say hi needs no geometry (one leg moves, the body shift is a small translation along the mount angles); wiggle's sway needs only the angles. Yaw, pitch and roll (dance, attack, ball) need positions.
- Part two: ball also needs the frame's underside, its height below the coxa pivots, measured from upstream's `Frame Bottom.stl` the way the pivots were, into `body_geometry.h` and `test/golden/body_geometry.txt`. Ball lowers the body until the underside is a few millimetres above the ground, then folds; while the underside is on the ground, the support check counts the frame as supporting the body rather than the feet.

### D4. Protocol version 3

- Command frame payload gains `gesture` (u8, 0 = none, 1–5 the gestures) and `gestureRequestCounter` (u8). A new counter value with a gesture id is one request; repeats change nothing. After a link gap the first frame only sets the baseline, as for the other counters.
- Telemetry gains the gesture being played (u8), so the bridge screen and the desktop show it.
- `protocol::VERSION` becomes 3; golden vectors are regenerated deliberately; the Go implementation follows byte for byte. A mismatched pair fails safe as today.

### D5. Triggers

- Bridge: while L2 (`BUTTON_TRIGGER_L`) is held, `a`, `b`, `x`, `y` and R2 request say hi, wiggle, dance, attack and ball, and their normal functions are suppressed; releasing L2 restores them. One request per press.
- Desktop: gesture buttons next to the mode buttons, enabled in control; keys 4–8 in the same order. The virtual robot needs nothing new: it runs the shared tick.

### D6. Choreography of dance, attack and ball

Upstream (Aecert's Hexapod V1, at `ca5ed27` and at its current HEAD) has only one of the five: `slamAttack()` in `Hexapod_Code/Attacks.ino`. It places the middle and front feet, heaves the body by sweeping the front and middle feet along a Bézier dipping 300 mm below them, and raises indices 2 and 3 to 300 mm above the coxa pivot, then slams them down to (250, 0, 0). By Spooder's leg order, taken from upstream and confirmed on the robot (0 and 5 the front pair, 2 and 3 the rear), that is a rear-pair slam. It runs as blocking loops without delays, so the servos move at full speed to points written without a servo-range check, and nothing can interrupt it. Spooder takes neither its leg pair nor its timing; it takes the shape of brace, heave, raise and strike.

- **Attack** (about 3 s): the front pair strikes, as the proposal says. The body shifts back and pitches nose up over the middle and rear four; legs 1 and 6 (indices 0 and 5) rise as high as their servo travel allows (say hi's wave stays below 70 mm over the coxa for the same reason), strike forward and down twice under D2's bound, plant, and the body straightens. The centre of gravity stays inside the four planted feet with the margin (`centre_of_gravity`).
- **Dance** (about 4 s, 120 BPM, a beat every 500 ms): all six feet planted except the last beat; roll left and right, a pitch nod, a yaw twist, a low bounce, then a front-leg tap (one front foot lifted and put down) before the body straightens.
- **Ball**: lower the body until the underside is on the ground (D3), fold the legs in towards the body, and hold (D1). The unfold reverses it into the stand pose.
- Every keyframe of all three passes D2's checks at both height extremes. The fixture `test/golden/gestures.txt` and the handheld's gesture table (`desktop/handheld/internal/gestures`) are regenerated, so the handheld's gesture overlay animates the three instead of marking them not yet available.
- The comment in `core/movement/gesture.cpp` that says dance, attack and ball wait for the measured body geometry goes: the geometry is in (task 2.1), and they waited on the body pose (task 2.2).

## Risks / Trade-offs

- **Attack and ball load single servos hardest** (rearing on four legs; the body lowered onto the ground). Bounds and protection limits apply; both come last and are tried with the robot supported. Attack's strike under the joint budget may look slower than upstream's full-speed slam; that is the price of the bound.
- **The underside height comes from the STL**, like the pivots; a measured value overrides it in one place.
- **The STL-derived geometry may differ from the build.** A measured value overrides it in one place.
- **Protocol 3 breaks mixed versions.** Fails safe; flash both boards and rebuild the desktop together.

## Migration Plan

Land in two parts: (1) Perform mode, protocol 3, triggers, say hi and wiggle; (2) body geometry, dance, attack and ball. Flash both boards together for part 1.
