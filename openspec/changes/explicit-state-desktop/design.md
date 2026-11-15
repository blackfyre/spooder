# Design

## Context

`desktop/internal/ui/window.go` keeps the port selection in `portSel` (an index, -1 for none) and `picked` (the operator chose a port, not a preselection). `picked` is set on a click on a port and never cleared. `setPorts` keeps the old selection only when it was picked or is the connected device, and otherwise preselects the `--port` path, else a Servo 2040. After the picked port vanishes, the fallback preselection is selected while `picked` is still true, so it is kept from then on, and the `--port` path never wins again.

The robot's firmware already has the rule this change brings to Go: `spooder-firmware-shape`, *Mode-dependent behaviour is an explicit state machine* and *No flags that duplicate a state*.

## Goals / Non-Goals

**Goals:** the rule in `desktop/CLAUDE.md`; the port selection as one explicit state, which fixes the defect; a regression test for the vanishing pick.

**Non-Goals:** the other implicit states (see Follow-ups); any change of the port list, the connection or reconnection behaviour; the handheld module.

## Decisions

**D1. The rule's wording for Go.** Go has no `enum class`, so the rule names the Go form: a named integer type with `iota` constants (and a `String` method when it is shown or logged), one transition function or method that is the only writer of the state and holds its guards, and data that only exists in one state kept beside it so a reader checks the state, not the data. No bool may duplicate a state, and nil, a zero `time.Time` or an empty string may not stand for one. It applies to `desktop/` without `desktop/handheld/`, which has its own module and gates (the brief excludes it).

**D2. `portChoice` with three states.** `None` (no port selected: the list is empty or nothing qualifies), `Preselected` (the app chose: the `--port` path, else a Servo 2040), `Picked` (the operator clicked a port). `portSel` stays the index; the choice replaces `picked`. One method, `choosePort`, writes both, and `setPorts` and the port click go through it:

- a click while disconnected → `Picked` at that index;
- a new list: a `Picked` selection, or the connected device, found again in the list stays (`Picked` stays `Picked`; the connected device keeps its state); otherwise the preselection rule runs and yields `Preselected` or `None`. So a pick that vanishes falls back to `Preselected`, and the next list preselects again, the `--port` path first.

**D3. The connected device is unchanged.** `w.device` duplicating the link lifecycle is follow-up F9; this change keeps its role in `setPorts` as it is.

**D4. The test.** A window test: pick a port, remove it from the list with the `--port` path absent (the Servo 2040 is preselected), then add the `--port` path; the path is preselected. It fails on the current code.

## Assumptions

- A connected device that vanishes and comes back is kept selected as today (the link reconnects to it by serial number); the defect is only about a pick while disconnected.
- After a vanished pick, the selection counts as a preselection, not as no selection: the operator sees the same fallback port as at start-up.
- The brief's other items are independent of the port selection; this change delivers the smallest coherent part (the rule and D3 of the brief), as the workflow asks.
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
- **Connected device (F9).** `w.device` duplicating the link lifecycle: derive it from the link status.
- **Remaining UI flags (F10).** `copied`/`noticeAt`/`notice` zero values; the CSV save's `saving` + `done`; `hoverOn`/`hoverAt`/`hoverT`; the console printer's `printedAny`/`lastKey` not reset on reconnect.

## Risks

- `setPorts` keeps its early return when the list is unchanged, so a press in progress on a port button survives; the choice must not change in that path either.
