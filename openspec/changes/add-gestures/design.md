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
- At the end the player returns to the stand pose and the mode to Stand; ball instead holds its last keyframe until any other request, then plays its unfold keyframes into Stand.
- Interruption: Sleep, a stale command, the e-stop and faults pre-empt at once, exactly as for Walk (the supervisor and protection are unchanged). Another mode request finishes the current keyframe, then returns to Stand first.

### D2. Safety bounds

- Every keyframe target is checked by `kinematics::isValidTarget` and every joint angle against the servo range with the default calibration, in host tests for every gesture at every body height.
- Static stability for gestures that lift legs: the body's centre, projected down, stays inside the support polygon of the planted feet with a margin (host test, from the body geometry).
- Speed: foot and body moves are rate-limited like Calibrate's approach, so peak servo current stays near walking levels; attack's strike is the fastest move and is bounded explicitly.

### D3. Body geometry

- `core/movement/body_geometry`: each leg's coxa pivot position (x, y, z) and mount angle in the body frame, from the upstream STL files (or the user's measurements), with the body-centre origin and axes of the desktop legs pane.
- The desktop's `internal/legview` takes the same values (a golden file, as for `leg_pose.txt`), so the legs pane draws the real shape.
- Say hi needs no geometry (one leg moves, the body shift is a small translation along the mount angles); wiggle's sway needs only the angles. Yaw, pitch and roll (dance, attack, ball) need positions.

### D4. Protocol version 3

- Command frame payload gains `gesture` (u8, 0 = none, 1–5 the gestures) and `gestureRequestCounter` (u8). A new counter value with a gesture id is one request; repeats change nothing. After a link gap the first frame only sets the baseline, as for the other counters.
- Telemetry gains the gesture being played (u8), so the bridge screen and the desktop show it.
- `protocol::VERSION` becomes 3; golden vectors are regenerated deliberately; the Go implementation follows byte for byte. A mismatched pair fails safe as today.

### D5. Triggers

- Bridge: while L2 (`BUTTON_TRIGGER_L`) is held, `a`, `b`, `x`, `y` and R2 request say hi, wiggle, dance, attack and ball, and their normal functions are suppressed; releasing L2 restores them. One request per press.
- Desktop: gesture buttons next to the mode buttons, enabled in control; keys 4–8 in the same order. The virtual robot needs nothing new: it runs the shared tick.

## Risks / Trade-offs

- **Attack and ball load single servos hardest** (rearing on four legs; the body lowered onto the ground). Bounds and protection limits apply; both come last and are tried with the robot supported.
- **The STL-derived geometry may differ from the build.** A measured value overrides it in one place.
- **Protocol 3 breaks mixed versions.** Fails safe; flash both boards and rebuild the desktop together.

## Migration Plan

Land in two parts: (1) Perform mode, protocol 3, triggers, say hi and wiggle; (2) body geometry, dance, attack and ball. Flash both boards together for part 1.
