# Design

## Context

See proposal.md for the defect. Today `protection::Supervisor` holds `bool bodyRunning`: `supervise` sets it false when output stops being allowed (`stopBody`) and true when output is allowed, no fault waits for a new request and the request is not Sleep (`startBody`). `robot_tick` passes `run.runBody` to the LED state and to the gesture field of telemetry, and to the console status line, but `sendTelemetry` reports `s.body.mode` unconditionally. Telemetry flags bits 0 to 5 are taken (`TELEMETRY_SIMULATED` is bit 5); bit 6 is free. The decision to add `TELEMETRY_BODY_STOPPED = 1u << 6` and bump `protocol::VERSION` to 4 was made by the user.

Several unarchived changes modify the same capabilities (`add-bench-simulation` modifies controller-link's "Telemetry frames"; `add-wifi-drive` modifies status-display's "Status content"; `relay-pose-diagnostics` modifies diagnostics-telemetry's "Diagnostics frame in the link protocol"; the handheld capability exists only in `add-handheld-controller`). Their code is on `main`.

## Goals / Non-Goals

**Goals:**
- One explicit run state that the flag, the LED input, the gesture guard and the console line all derive from.
- Every front-end that shows the body mode shows a stopped body as stopped.

**Non-Goals:**
- Telling "not started" from "stopped after a fault" on the link: one flag covers both; the console status line and the LEDs can tell them apart, the front-ends need not.
- Any change to when the body stops or restarts.
- Any change to the frame layout or to other frames.

## Decisions

### D1. `enum class RunState : uint8_t { NotStarted, Running, Stopped }` in `protection::Supervisor`

`Supervisor::bodyRunning` becomes `RunState runState = RunState::NotStarted`. `supervise` moves Running to Stopped where it sets `stopBody`, and NotStarted or Stopped to Running where it sets `startBody`; the start and stop conditions are unchanged. `Decision::runBody` stays (it equals `runState == Running` after the call) so the tick's step guard reads as before. A helper `bool bodyRunning(const Supervisor &)` is not added: callers compare with `RunState::Running`.

Alternative: keep the bool and add a separate "started once" bool. Rejected: two bools allow an impossible combination and the brief asks for an explicit state.

### D2. The flag is computed in `robot_tick`, beside `TELEMETRY_SIMULATED`

`flags |= runState != Running ? TELEMETRY_BODY_STOPPED : 0`, using the state after this tick's `supervise`. The gesture argument becomes `runState == Running ? body::playingGesture(s.body) : 0`, the LED input `bodyRunning` is set from `runState == Running` (the `led_pattern::State` bool stays: led_pattern does not need the three states), and `reportStatus` prints `not started`, `running` or `stopped`. The telemetry's `bodyMode` and `gait` keep the body's last values (front-ends hide them while the flag is set); zeroing them was rejected because `0` is Initialise, a real mode.

### D3. Protocol version 4, golden file renamed

`protocol::VERSION = 4` and Go's `protocol.Version = 4`. The golden file is regenerated (`SPOODER_WRITE_GOLDEN=1 mise run test`) and renamed `test/golden/protocol_v4.txt`, as its name carries the version; every reference (`test/golden_frames.h`, `test/test_protocol_golden`, `desktop/internal/protocol/golden_test.go`, `fields.go` and `protocol.go`, `desktop/internal/link/fuzz_test.go`, `desktop/internal/console/console_test.go`, `desktop/internal/wifiprobe/probe_test.go`, `test/CLAUDE.md`, `tools/CLAUDE.md`, `desktop/CLAUDE.md`) is renamed with it. One golden telemetry vector with `TELEMETRY_BODY_STOPPED` is added. The diff is reviewed: only the version byte, the CRCs and the new vector may change.

### D4. Front-ends hide the mode while the flag is set

- Bridge (`core/status/status_screen`): the header's mode text is `STOPPED` (normal tone) instead of `MODE · GAIT` or `PERFORM · gesture`; the setup screen's body cell shows `Stopped`, red while a fault is latched as today. The bridge firmware itself needs no code change beyond the rebuild.
- Desktop: `session.Snapshot` gains `Stopped bool` from the flag; `bodyText` returns `Stopped` when it is set; the charts' mode lane builds its snapshot with `Stopped` from the sample's flags; the console's `formatTelemetry` and the WiFi drive probe's telemetry summary (`desktop/internal/wifiprobe`) print `Stopped` in place of the body mode.
- Handheld: `Robot.Body` is `Stopped` when the flag is set, so the gesture guard (`rb.Body != "Stand" && rb.Body != "Perform"`) already treats it as unavailable; `confirms()` also returns true for the flag.

### D5. Spec deltas are ADDED requirements only

To avoid conflicting MODIFIED deltas with the unarchived changes above, each capability gets a new requirement. The older requirements that name a protocol version (diagnostics-telemetry's "version 3", pose-telemetry's "version 2" and "reserved in protocol version 3") describe the version that introduced those frames and fields, which stays true; they are not reworded here.

## Risks / Trade-offs

- [A board left on version 3 sees a dead link] → fail safe by design (the robot sleeps); the changelog, `CLAUDE.md` ("now 3" becomes 4) and the hardware task say to flash both boards together.
- [A stopped body with output on cannot occur today, so the handheld's new `confirms()` branch is defensive] → it costs one condition and keeps the leave rule independent of how stop and output interact.
- [Before the first start the front-ends now show `STOPPED` where they showed `INITIALISE`] → intended: the body is not running then.

## Migration Plan

Flash both boards from the same build, rebuild the desktop calibrator and the handheld pak. Rollback is reflashing both boards and rebuilding both apps from the previous commit.

## Assumptions

- The flag is set in both non-running states (not started and stopped), as the brief requires; the label is `STOPPED` / `Stopped` in both, since the link does not distinguish them.
- The flag uses the run state after this tick's `supervise`, so the frame sent on the tick a fault stops the body already has it.
- usb-link and diagnostics-telemetry need no delta: the telemetry carried on USB is the same frame specified in controller-link, and no diagnostics behaviour changes (D5).
- The handheld's requirement is ADDED to `handheld-controller`, the capability `add-handheld-controller` introduces; archiving either change first merges cleanly.
- `tools/bridge-screenshots` sets no telemetry flags, so its images show a running body and stay unchanged.

## Follow-ups

- Reword the version-numbered sentences in diagnostics-telemetry and pose-telemetry once `relay-pose-diagnostics` and `add-bench-simulation` are archived, if a single "current version" statement is wanted.
- The bridge's WiFi drive relay forwards frames without showing a body mode; if it gains one, it follows D4.
