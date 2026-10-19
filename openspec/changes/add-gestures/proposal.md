# Proposal

## Why

Spooder should do more than stand and walk: short, playful moves on a button press make it fun to show off and exercise the legs and the power protection in a controlled way. The requested set is say hi, wiggle, dance, attack and ball (curl up like a pill bug: body down on the ground, legs folded in).

## What Changes

- **Gestures.** A new body mode, Perform, plays one gesture: a scripted sequence of foot targets and body poses with timings, through the existing IK on the 5 ms grid. A gesture starts from Stand and ends in Stand (ball ends curled and waits; any other request uncurls through Stand). Sleep, a stale command, the e-stop or a fault interrupts it at once, as for every mode; another mode request ends it at the next safe point.
- **The five gestures:**
  - say hi: shift the body back, lift a front leg high and wave it, put it down;
  - wiggle: sway and twist the body with all six feet planted;
  - dance: a timed routine of body bobs, sways and leg taps;
  - attack: rear up on the back four legs, raise both front legs, strike down;
  - ball: lower the body onto the ground and fold the legs in.
- **Body geometry.** Body sway, lean and pitch with planted feet need each leg's mount position and angle in the body frame. They are taken from the upstream STL files (or measurements) into `core/movement`, and the desktop legs pane uses the same values instead of its schematic.
- **Protocol version 3.** The command frame gains a gesture id and a gesture-request counter, so that holding a button plays a gesture once and a lost frame loses nothing. Both boards must be flashed together.
- **Triggers.** On the gamepad, L2 held turns the face buttons into gestures (L2 + `a` say hi, `b` wiggle, `x` dance, `y` attack, R2 ball). The desktop calibrator gets gesture buttons while in control, so each gesture can be rehearsed on the virtual robot in the legs pane before the real robot tries it. The bridge screen and telemetry show the gesture being played.
- **Delivery order:** say hi and wiggle first (wiggle's translate sway needs only the mount angles); dance, attack and ball once the body geometry is in.

## Capabilities

### New Capabilities

- `gestures`: the Perform mode, the gesture set, interruption and safety bounds.

### Modified Capabilities

- `controller-link` (from `add-gamepad-bridge`): protocol version 3 with the gesture request.
- `gamepad-control` (from `add-gamepad-bridge`): the L2 gesture layer.
- `desktop-calibrator` (from `add-desktop-calibrator`): gesture buttons.

## Impact

- `core/movement/` (Perform mode, gesture player and scripts, body geometry), `core/protocol/` (v3, golden vectors regenerated), `core/protocol/link_receiver` (gesture counter baselines after a link gap), `core/gamepad_mapping/`, `core/status/`.
- `desktop/` (Go protocol v3, session, gesture buttons, legs pane mounts), the virtual robot (no change beyond the shared tick).
- Both firmwares and the desktop calibrator must move to protocol 3 together.
- Skills, readme and changelog.
