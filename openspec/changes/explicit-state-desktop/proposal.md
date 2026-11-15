# Proposal

## Why

The robot's firmware keeps mode-dependent behaviour in explicit state machines (the `spooder-firmware-shape` skill: one `enum class`, one transition function with its guards, no flags that duplicate a state). The desktop calibrator has no such rule, and its state is spread over bools, nil pointers, zero times and empty strings that stand for states. One of them breaks the specification: the port selection's `picked` flag is never cleared, so once the operator's picked port is unplugged, the fallback preselection counts as a pick and the `--port` path is never preselected when it appears, against the requirement *Explicit port* ("listed and preselected whenever the path exists").

## What Changes

- **The rule for Go.** `desktop/CLAUDE.md` states the explicit-state rule for the calibrator's module (`desktop/`, not `desktop/handheld/`): state that changes behaviour is one enum-like type with one transition function and its guards; no bool that duplicates a state; no nil, zero time or empty string standing for a state.
- **The port selection as one state.** The window's port choice becomes one explicit state, `portChoice` (`None`, `Preselected`, `Picked`), with one transition function. When the operator's picked port leaves the list, the choice falls back to a preselection, so the `--port` path is preselected again once it appears.
- The other implicit states the brief lists (the WiFi probe's session phase, session control, freshness, key state, the text pane, the calibration table, the export pane, the connected device, the remaining UI flags) are follow-ups, recorded in `design.md`.
- No protocol, firmware or bridge change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `desktop-calibrator`: *Explicit port* gains a scenario for a picked port that is unplugged while the `--port` path is absent and then appears.

## Impact

`desktop/internal/ui/window.go` (port selection) and `window_test.go`, `desktop/CLAUDE.md` and `CHANGELOG.md`.
