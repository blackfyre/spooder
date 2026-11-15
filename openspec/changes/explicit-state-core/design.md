# Design

## Context

`gamepad_mapping::Mapper` (`core/gamepad_mapping/gamepad_mapping.h`) holds the requested mode as three fields: `baseMode` (Stand or Walk), `sleeping` and `calibrating`. `requestedMode()` reads them in priority order, and `update()` writes them in four places: a fresh connection, the Calibrate hold, the `miscHome` press and the `y` press. Of the twelve combinations, five are reachable, because each writer restores the invariants by hand (`calibrating` implies `!sleeping` and `baseMode == Stand`). The handheld's Go port (`desktop/handheld/internal/padmap/mapping.go`) mirrors this line for line, and `Handheld.RequestSleep` (`handheld.go`) writes the mapper's private fields from outside it. `test/golden/gamepad_mapping.txt` runs both over the same hand-written steps.

## Goals / Non-Goals

**Goals:**

- One `enum class` for the mapping's mode request, one transition function, no field that duplicates it, in C++ and Go.
- No behaviour change: the golden file is not edited and passes in both languages.
- The gamepad-control specification describes the behaviour that is kept.
- Reviews catch new implicit state machines (`REVIEW.md`).

**Non-Goals:**

- The other fifteen items of the brief (Follow-ups).
- The combo hold detectors (`calibrateHeld/Fired/SinceMs`, `clearHeld/Fired/SinceMs`): item 16, a follow-up.
- `Supervisor::bodyRunning` and the telemetry body mode (change `body-stopped-telemetry`).

## Decisions

### D1. Five states

`enum class ModeState : uint8_t { Stand, Walk, SleepFromStand, SleepFromWalk, Calibrate }` replaces `baseMode`, `sleeping` and `calibrating`. These are exactly the reachable combinations today. The requested mode is a pure function of the state: Stand, Walk, Sleep (both sleep states) and Calibrate.

### D2. One transition function over mode events

`ModeState next(ModeState, ModeEvent)` with `ModeEvent { Connect, Home, Sleep, CalibrateHold }` holds every rule in one table:

| State | Connect | Home | Sleep (`y`) | CalibrateHold |
|---|---|---|---|---|
| Stand | Stand | Walk | SleepFromStand | Calibrate |
| Walk | Stand | Stand | SleepFromWalk | Calibrate |
| SleepFromStand | Stand | SleepFromWalk | Stand | Calibrate |
| SleepFromWalk | Stand | SleepFromStand | Walk | Calibrate |
| Calibrate | Stand | Stand | SleepFromStand | Stand |

`update()` keeps its order of evaluation (connect, then the Calibrate hold or else `miscHome`, then `y` after the gesture layer has masked it), applies one event per detected input and advances the mode-request counter on Home, Sleep and CalibrateHold, as now. Two events in one sample (the hold firing and `y` pressed together) are applied in that order, as the fields are written now. The guards (`miscBack` held suppresses Home; the gesture layer masks `y`) stay input detection in `update()`, not states.

### D3. The handheld's sleep request is a Go mapper transition

`Handheld.RequestSleep` becomes `Mapper.RequestSleep()` in the Go port, a fifth event (`ModeEventSleepRequest`): from Stand, Walk or Calibrate to SleepFromStand with the counter advanced; from either sleep state to SleepFromStand with no counter step. `Handheld.RequestSleep` calls it. It stays out of the C++ mapping and the golden file, because no C++ caller requests Sleep this way (the bridge's session start holds Sleep outside the mapper) and the shared golden steps cover pad samples only; the Go table test covers it.

### D4. Specification correction

The user decided to keep the behaviour and correct the specification. The `Input mapping` requirement of gamepad-control is modified from the text of the unarchived `wave-default-gait` change (which already modifies it and whose code is merged), so archiving either one first keeps the Wave start-up text.

## Assumptions

- The golden file's own comment ("Sleep on Y returns to the mode requested before it") describes its steps correctly as they stand and is left untouched, as the brief requires the file unchanged.
- `handheld-controller` needs no delta: its requirement refers to gamepad-control and its sleep-request behaviour does not change.
- No hardware check is needed: behaviour is pinned by the golden file in both languages and the bridge build compiles the mapping.

## Risks

- A transition that differs from today's in one cell of D2 would change the robot's mode on a button press. The golden file and a host test of all twenty table cells (and the five Go sleep-request cells) guard against it.

## Follow-ups

Each is its own change, with host tests of its transitions; numbers are the brief's.

1. Resume gate latched in two places (`robot_tick.cpp` writes `supervisor.gate.waiting/requestSeqAtLatch` besides `resumeGate()`): one latch function.
2. `BodyState::outputsEnabled` duplicates `mode == Sleep && sleepFolded`: Sleep{Folding, Folded} and Stand{Transition, Settled} sub-phases, `outputsEnabled` derived; readers in `bench_model::Servos::apply` and `servo_output` callers.
3. Body mode changes outside `resolveMode()` (`stepInitialise`, `stepPerform`, the Calibrate assignment): one transition function.
4. `gesture::Player::done` beside `Stage`: `Stage::Done`.
6. Protection supply gate (`gateOpen/validRun/validSinceMs`) to `SupplyGate {Closed, Settling, Open}`, and the `aboveTrip/belowMin` debounces; the fault bitmask stays.
7. `link_receiver::haveCalibrationRequest` duplicates `lastResult != NoneYet`.
8. `calibration_sync` `haveReport/inFlight/editedInFlight/haveLastId`: a flight enum.
9. Rumble gesture refusal tracker (`gesturePending/sawNotPlaying/gestureAtMs`): an enum.
10. `relay_queue` `epoch/pollEpoch/clean`: a gate enum with its epoch.
11. Command freshness (`issuedAtMs != 0` in `robot_tick` and `link_receiver`, `MAX_COMMAND_LEAD_MS`/`staleLatched` in `body`): one freshness function returning {None, Fresh, Stale} with an explicit has-command flag.
12. `robot_tick` `profileAnnounced` as first-tick flag; `legsOn/outputKnown` to `SwitchOn {Off, Releasing, On}`.
13. `bench_model` `on` versus `inhibited`, and the `desiredUs == 0` sentinel.
14. `BenchBoard::simulated_` duplicates `Entry::phase == Simulating`; phase-only fields carried with the phase.
15. `portal_controller::desired()` resetting to Sleep inside the getter: `Connection {Never, Live, Lapsed}` judged on every call.
16. The debounce pairs in `setup_entry`, the gamepad combo detector, `page_button`, `drive_mode::Notice` and `peak_hold`: explicit enums.
