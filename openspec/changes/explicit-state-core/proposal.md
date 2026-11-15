# Proposal

## Why

The project rule (`spooder-firmware-shape`: state machines) asks for mode-dependent behaviour as one `enum class` with one transition function and its guards, and no flags that duplicate a state. `core/` and its Go mirror break it in many places: booleans, timestamps, counters and sentinels doubling as state, and states written from outside their transition function. Each is a place where an impossible combination can be reached or a transition can bypass its guard.

The brief lists sixteen such places. They are independent of each other, so this change delivers the first coherent one, the gamepad mapping's mode request, and records the rest as follow-ups (design.md). It is the one that also needs a specification correction the user decided: the gamepad-control wording "Leaving Sleep returns to the mode requested before it" contradicts today's behaviour, which is kept.

## What Changes

- `gamepad_mapping::Mapper`'s `baseMode`, `sleeping` and `calibrating` become one `enum class ModeState { Stand, Walk, SleepFromStand, SleepFromWalk, Calibrate }` with one transition function over the mapping's mode events (connect, `miscHome` press, `y` press, Calibrate hold). The requested mode is derived from it. Behaviour does not change: `test/golden/gamepad_mapping.txt` stays as it is and passes.
- The handheld's line-for-line Go port (`desktop/handheld/internal/padmap/mapping.go`) takes the same enum and transition function. `Handheld.RequestSleep`, which writes the mapper's private fields, becomes a transition of the Go mapper (a sleep-request event) that `Handheld` calls; its behaviour does not change.
- The gamepad-control specification states the behaviour the mapping has: leaving Sleep returns to Stand or Walk, whichever Sleep was entered from; Sleep entered from Calibrate returns to Stand; `miscHome` while Sleep is requested switches the mode Sleep returns to and leaves Sleep requested.
- `REVIEW.md` gains "implicit state machine" as a known bug class, for C++ and Go.
- No protocol change, no firmware behaviour change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gamepad-control`: the input mapping states how leaving Sleep and Calibrate resolves the requested mode, matching the implementation.

The handheld follows `gamepad-control` through its mapping requirement (`handheld-controller`), which needs no text change.

## Impact

- `core/gamepad_mapping/gamepad_mapping.h`, `gamepad_mapping.cpp`; `test/test_gamepad_mapping` (transition tests).
- `desktop/handheld/internal/padmap/mapping.go`, `handheld.go` and their tests.
- `REVIEW.md`, `CHANGELOG.md`.
- The bridge compiles the mapping; flashing it is not needed for behaviour, which the golden file pins.
