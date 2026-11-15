# Proposal

## Why

The handheld (`desktop/handheld/`) keeps several state machines implicit, against the rule in the `spooder-firmware-shape` skill (one enum-like type with one transition function; no flag duplicating a state; no nil, zero time or empty string as a state). One of them has a defect. When `wlan0` rejoins the network the run started on, `internal/app` calls `Link.Rebaseline()` and then `Link.TakeEvents()`, two separate lock acquisitions. A robot frame accepted between them is the rebaselining frame, and the drain discards it. A leave in progress learns that the sequence numbers were rebaselined only from that frame's `Rebaseline` flag, so it keeps its reference from before the press. Telemetry after the rejoin is numbered afresh and is compared against that stale reference, so genuine Sleep confirmations can be rejected until the leave times out (3 s).

## What Changes

- **Atomic rebaseline and drain.** The link rebaselines its receiver and drains its event queue under one lock, so no frame can fall between the two. The resume from suspend and the rejoin both use it.
- **The leave's reference is explicit.** The receiver numbers its baselines: each rebaseline starts a new one, and every event and status carries the baseline it belongs to. A leave records the baseline and newest sequence number at the press. A telemetry frame from that baseline is compared by its sequence number. A frame from a later baseline counts as sent after the press. A frame from an earlier baseline does not count. The leave no longer infers its reference state from one flagged event, and it can no longer miss a rebaseline. `refValid` is removed.
- A test drives a rejoin during a leave, with a frame accepted between the rebaseline and the drain, and checks that two Sleep telemetry frames after the rejoin end the leave before the timeout.
- The other implicit state machines named in the brief are recorded as follow-ups in `design.md`. They are independent of this defect.
- No protocol, firmware or bridge change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `handheld-controller`: the leaving requirement states how frames taken after a rebaseline during the wait are ordered against the press, and gains a scenario for a rejoin during a leave. The capability is added by `add-handheld-controller` and last modified by `handheld-network-check`, both merged but not yet archived; this delta builds on the `handheld-network-check` text.

## Impact

- `desktop/handheld/internal/udplink` (baseline number on `Event` and `Quality`, the atomic rebaseline and drain), `desktop/handheld/internal/app` (the `Link` interface, the leave's reference, the rejoin and the resume), their tests, `desktop/CLAUDE.md` (the `internal/udplink` entry) and `CHANGELOG.md`.
