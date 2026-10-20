# Design

## Context

`session.Session` owns what the app sends. `Command()` builds every 20 ms frame from the control flag, the requested mode and the counters; height and speed are constants (0 and 44), and translate and rotate are always zero. `RequestMode` refuses Walk. The window (`internal/ui/window.go`) turns Gio key events into session calls. One-shot actions (modes, clear fault, gestures) go through a press latch that also resets on focus loss (`focusChanged`). The robot eases translate, rotate, height, the speed dial and the stride mode itself, and gates movement on the source in control.

The bridge's car layout (`core/gamepad_mapping`) defines the driving: walk on Y, steer on X, steering mirrored when reversing, and turning on the spot with a sideways stick.

## Goals / Non-Goals

**Goals:**
- Walk, steer, turn and sidestep from the keyboard, with the same meaning as the bridge's car layout.
- No input left applied when the keyboard stops reporting: on focus loss, release of control or disconnect.
- Testable in the session without a window.

**Non-Goals:**
- A gamepad on the computer, analogue input or key-rate ramps (the robot eases).
- The crab layout as a mode: Q and E give sidestepping directly.
- Changes to the firmware or the link protocol.

## Decisions

### The session holds the drive state; the UI only reports keys

`session.Drive` is the set of held drive keys (forward, back, steer left and right, step left and right). `SetDrive(d)` replaces it. The UI recomputes it from its own held-key set on every press and release, and clears it on focus loss. `Command()` maps it to the frame only when the app is in control, connected and requesting Walk:
- translate Y = 100 × (forward − back);
- translate X = 100 × (step right − step left);
- steer = steer right − steer left, and rotate = −100 × steer, negated while reversing (back held without forward).

Positive rotate turns anticlockwise, as the robot and the bridge define it. Keeping the mapping in the session lets unit tests check the frames directly.

*Alternative:* map in the UI and pass axis values. Rejected: the car semantics then live in the UI, untested without a window.

### Driving requests Walk from Stand, only once

`SetDrive` with any key held while the requested mode is Stand sets the mode to Walk and advances the mode-request counter, as the gamepad's walk toggle does. Releasing the keys keeps Walk with zero input: the robot holds its feet, and Stand (1) ends walking. Sleep and Calibrate ignore the drive keys, so a stray W cannot pull the robot out of Calibrate. `RequestMode` keeps refusing an explicit Walk; it is entered only by driving.

*Alternative:* return to Stand when the keys are released. Rejected: each release would restart the Stand transition, and walking in short taps would then lift legs between steps.

### Settings are one-shot keys on the existing latch

Z/X, C/V, G and T are one-shot actions through `actionKey`'s press latch, as the mode keys are, calling `StepSpeed`, `StepHeight`, `NextGait` and `ToggleStride`. They work only in control. Their values persist across releases of control within a run, as the bridge's do across stick moves. `Command()` sends them only in control; observing keeps sending the defaults.

### Held keys need press and release

The drive keys are filtered in `keyFilters`, and their press and release events update a held set in the window. The set is cleared in `focusChanged(false)`, and the session clears its drive state on release of control and on disconnect.

Walk is requested only by a newly pressed key (`Drive.pressedSince`). A key still held after Stand (1), or a release, never restarts walking. Keys that are down when the app cannot follow them are suppressed until their release arrives. That covers keys pressed while observing, keys held when control or the link is lost, and keys held when the window loses focus. The operating system's key repeat would otherwise look like a fresh press once control or focus returns. The price is one ignored press for a key released in another window.

The drive, mode, gesture and settings keys accept every modifier, as Space does, so a release with Shift or Ctrl down still reaches the held set and the latches.

## Risks / Trade-offs

- [A release event lost by the windowing system leaves a key held] → Focus loss stops all input. Space stops the robot whatever happens. The robot also sleeps one second after frames stop.
- [Walking from the keyboard is coarse (full input only)] → Accepted for bench work. The speed dial (Z/X) sets how fast full input walks.
