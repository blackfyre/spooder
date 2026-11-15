# Design

## Context

`desktop/internal/ui/window.go` keeps the port selection in `portSel` (an index, -1 for none) and `picked` (the operator chose a port, not a preselection). `picked` is set on a click on a port and never cleared. `setPorts` keeps the old selection only when it was picked or is the connected device, and otherwise preselects the `--port` path, else a Servo 2040. After the picked port vanishes, the fallback preselection is selected while `picked` is still true, so it is kept from then on, and the `--port` path never wins again.

The robot's firmware already has the rule this change brings to Go: `spooder-firmware-shape`, *Mode-dependent behaviour is an explicit state machine* and *No flags that duplicate a state*.

## Goals / Non-Goals

**Goals:** the rule in `desktop/CLAUDE.md`; the port selection as one explicit state, which fixes the defect; a regression test for the vanishing pick.

**Non-Goals:** the other implicit states (see Follow-ups); any change of the port list, the connection or reconnection behaviour; the handheld module.

## Decisions

**D1. The rule's wording for Go.** Go has no `enum class`, so the rule names the Go form: a named integer type with `iota` constants (and a `String` method when it is shown or logged), one transition function or method that is the only writer of the state and holds its guards, and data that only exists in one state kept beside it so a reader checks the state, not the data. No bool may duplicate a state, and nil, a zero `time.Time` or an empty string may not stand for one. It applies to `desktop/` without `desktop/handheld/`, which has its own module and gates (the brief excludes it). The rule is prospective: it binds state that a change adds or modifies, so the implicit states listed under Follow-ups stay until their own changes and do not make the rule false on the day it lands; the rule's text in `desktop/CLAUDE.md` says so and points to those follow-ups.

**D2. `portChoice` with three states.** `None` (no port selected: the list is empty or nothing qualifies), `Preselected` (the app chose: the `--port` path, else a Servo 2040), `Picked` (the operator clicked a port). `portSel` stays the index; the choice replaces `picked`. One method, `choosePort`, writes both, and `setPorts` and the port click go through it:

- a click while disconnected → `Picked` at that index;
- a new list while disconnected: a `Picked` selection found again in the list (`link.SameDevice`) stays `Picked`, ahead of the `--port` path; otherwise the preselection rule runs at once and yields `Preselected` (the `--port` path, else a Servo 2040) or `None`. So a pick that vanishes falls back to `Preselected` in the same `setPorts` call, and later lists preselect again, the `--port` path first;
- a new list while connected (`w.device != nil`): as today, the current selection, whatever its state, stays when found again, keeping its state; otherwise the preselection rule runs.

**D3. The connected device is unchanged.** `w.device` duplicating the link lifecycle is follow-up F9; this change keeps its role in `setPorts` as it is.

**D4. The tests.** A window test: pick a port, remove it from the list with the `--port` path absent and a Servo 2040 listed, and assert the Servo 2040 is selected and the choice `Preselected` right after that `setPorts`; then add the `--port` path and assert the path is selected. It fails on the current code at the last assertion. The pick kept while the `--port` path is listed is already asserted by `TestExplicitPortIsPreselectedAheadOfAServo2040` (its last step) and stays. A second window test pins the connected case D3 keeps: connected to the Servo 2040, a list that adds the `--port` path leaves the Servo 2040 selected.

## Assumptions

- While connected, the selection behaves as today: it stays while it is listed; if the connected device vanishes, the preselection rule picks a fallback, and that fallback stays when the device comes back (the link reconnects by serial number regardless of the selection). The defect is only about a pick while disconnected, so the connected case is kept, not corrected (follow-up F9).
- After a vanished pick, the selection counts as a preselection, not as no selection: the operator sees the same fallback port as at start-up.
- The brief's other items are independent of the port selection; this change delivers the smallest coherent part (the rule and D3 of the brief), as the workflow asks.
- `choosePort` takes the event (`portClicked` with the clicked index, `portsListed` with the previous selection's index in the new list, -1 if gone) rather than a target state, so the guards (a pick only while disconnected; keep only a pick, or any selection while connected; else preselect) live in it and not at its callers. The window's constructor also goes through it, to start at `None`.
- The WiFi probe defect (brief D4) is left for a follow-up because it has no requirement in the main specifications: the probe belongs to `add-wifi-drive`, whose `wifi-drive` capability is not archived yet, so a delta for it here could not be archived cleanly.

## Follow-ups

Each is its own change, under the rule this change adds:

- **WiFi probe session phase (defect, brief D4).** `desktop/internal/wifiprobe/probe.go`: requests arm on `frames > 0`, which never resets, so after `--pause-for` outlasts the bridge's liveness timeout (150 ms) the first frame starts a new bridge session that only sets baselines (`core/wifi_drive/handheld_controller.cpp`), and a `--mode-after`/`--clear-after` firing on it is lost. An explicit phase {AwaitingBaseline, Live}, reset by the pause, with requests applied only when Live; consider replacing the `ticks` nil channel and the first timer; a pause + request test. Best after `add-wifi-drive` is archived.
- **Session control (F3).** `internal/session`: control spread over `control`, `link.Status`, `mode`, `drive`; `mode = RequestSleep` written in three places; seven guards re-test control and connection. One control state (nil while observing) with one transition.
- **Freshness (F4).** `haveTelemetry`, a zero time and stale values kept across reconnect in the session, and `reportAt` with `calib.HaveReport`: stamped values, nil when none.
- **Key state (F5).** `held` and `suppressed` maps, `unfocused`, `keyLatch.down`, two policies for keys held across a focus change: one `map[key]keyState` {Up, Held, Suppressed} for every key including Space (the e-stop) and the action keys, so a key held across a focus change can never toggle the e-stop; with a test.
- **Text pane (F6).** `anchored`, `dragging`, `held`, `atEnd`, `haveBase`, `held` written in four places: {Following, Dragging, Holding}.
- **Calibration table (F7).** `haveReport`, `haveLastID`, `seeded`, `pending` nil plus `sent`.
- **Export pane (F8).** `plan`, `status`, `failed`: an `exportPhase`.
- **Connected device (F9).** `w.device` duplicating the link lifecycle: derive it from the link status. Disconnecting does not re-run the preselection, so a selection kept only because the app was connected stays after Disconnect even when the `--port` path is listed (pre-existing, found in this change's review); a disconnect event in `choosePort` closes it.
- **Remaining UI flags (F10).** `copied`/`noticeAt`/`notice` zero values; the CSV save's `saving` + `done`; `hoverOn`/`hoverAt`/`hoverT`; the console printer's `printedAny`/`lastKey` not reset on reconnect.

## Risks

- `setPorts` keeps its early return when the list is unchanged, so a press in progress on a port button survives; the choice must not change in that path either.
