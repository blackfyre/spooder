# Design

## Context

See proposal.md for the defect. Today `protection::Supervisor` holds `bool bodyRunning`: `supervise` sets it false when output stops being allowed (`stopBody`) and true when output is allowed, no fault waits for a new request and the request is not Sleep (`startBody`). `robot_tick` passes `run.runBody` to the LED state and to the gesture field of telemetry, and to the console status line, but `sendTelemetry` reports `s.body.mode` unconditionally. Telemetry flags bits 0 to 5 are taken (`TELEMETRY_SIMULATED` is bit 5); bit 6 is free. The decision to add `TELEMETRY_BODY_STOPPED = 1u << 6` and bump `protocol::VERSION` to 4 was made by the user.

Several unarchived changes modify the same capabilities (`add-bench-simulation` modifies controller-link's "Telemetry frames"; `add-wifi-drive` modifies status-display's "Status content"; `relay-pose-diagnostics` modifies diagnostics-telemetry's "Diagnostics frame in the link protocol"; the handheld capability exists only in `add-handheld-controller`). Their code is on `main`.

## Goals / Non-Goals

**Goals:**
- One explicit run state that the flag, the LED input, the gesture guard and the console line all derive from.
- Every front-end that shows the body mode shows a stopped body as stopped.

**Non-Goals:**
- Telling "not started" from "stopped after a fault" on the link: one flag covers both; only the robot's console status line tells them apart (the LEDs show both as the same blue breathing), and the front-ends need not.
- Any change to when the body stops or restarts.
- Any change to the frame layout or to other frames.

## Decisions

### D1. `enum class RunState : uint8_t { NotStarted, Running, Stopped }` in `protection::Supervisor`

`Supervisor::bodyRunning` becomes `RunState runState = RunState::NotStarted`. `supervise` moves Running to Stopped where it sets `stopBody`, and NotStarted or Stopped to Running where it sets `startBody`; the start and stop conditions are unchanged. A fault before the first start leaves NotStarted unchanged: Stopped is entered only from Running. `Decision::runBody` stays (it equals `runState == Running` after the call) so the tick's step guard reads as before. A helper `bool bodyRunning(const Supervisor &)` is not added: callers compare with `RunState::Running`.

Alternative: keep the bool and add a separate "started once" bool. Rejected: two bools allow an impossible combination and the brief asks for an explicit state.

### D2. The flag is computed in `robot_tick`, beside `TELEMETRY_SIMULATED`

`flags |= runState != Running ? TELEMETRY_BODY_STOPPED : 0`, using the state after this tick's `supervise`. The gesture argument becomes `runState == Running ? body::playingGesture(s.body) : 0`, the LED input `bodyRunning` is set from `runState == Running` (the `led_pattern::State` bool stays: led_pattern does not need the three states), and `reportStatus` prints `not started`, `running` or `stopped`. The telemetry's `bodyMode` and `gait` keep the body's last values (front-ends hide them while the flag is set); zeroing them was rejected because `0` is Initialise, a real mode.

### D3. Protocol version 4, golden file renamed

`protocol::VERSION = 4` and Go's `protocol.Version = 4`. The golden file is regenerated (`SPOODER_WRITE_GOLDEN=1 mise run test`) and renamed `test/golden/protocol_v4.txt`, as its name carries the version; every reference (`test/golden_frames.h`, `test/test_protocol_golden`, `desktop/internal/protocol/golden_test.go`, `fields.go` and `protocol.go`, `desktop/internal/link/fuzz_test.go`, `desktop/internal/console/console_test.go`, `desktop/internal/wifiprobe/probe_test.go`, `test/CLAUDE.md`, `tools/CLAUDE.md`, `desktop/CLAUDE.md`) is renamed with it. One golden telemetry vector with `TELEMETRY_BODY_STOPPED` is added, and a rejected vector `reject_version_3_command` (the previous version, with a valid CRC) beside `reject_version_2_command`, so every implementation proves it discards the version a not yet reflashed peer sends. The diff is reviewed: only the header's version, the version byte, the CRCs and the two new vectors may change.

### D4. Front-ends hide the mode while the flag is set

- Bridge (`core/status/status_screen`): the header's mode text is `STOPPED` (normal tone) instead of `MODE · GAIT` or `PERFORM · gesture`; the setup screen's body cell shows `Stopped` (its mode names are in that case), red while a fault is latched as today. The change is in `core/status`; `firmware/bridge/` needs no source change beyond the rebuild. The header region is redrawn only when its text changes, as today, so the screen's 20 ms budget is unaffected.
- Desktop: `session.Snapshot` gains `Stopped bool` from the flag; `bodyText` returns `Stopped` when it is set; the charts' mode lane builds its snapshot with `Stopped` from the sample's flags. The frames pane (`protocol.Summary`) is a raw field dump and keeps `bodyMode=` and `flags=` as received. The robot console's `formatTelemetry` (`spooder-monitor --frames`) prints `Stopped` in place of the body mode and gait, and its throttle treats a change of the flag as a change, like mode; the WiFi drive probe's telemetry summary (`desktop/internal/wifiprobe`, a development tool without a capability requirement) prints `Stopped` in place of the body mode.
- Setup portal (`firmware/bridge/main/portal_page.h`): the body line reads `Body Stopped` while the flag is set, and a stopped Calibrate does not count as calibrating, so the editor's hint says the robot is stopped. The page tests the flag by value (`fl&64`, like its other flag bits), and `setup_mode.cpp` asserts at compile time that `TELEMETRY_BODY_STOPPED` is 64. No host test reaches the page's JavaScript, as before.
- Handheld: `Robot.Body` is `Stopped` when the flag is set, so the gesture guard (`rb.Body != "Stand" && rb.Body != "Perform"`) already treats it as unavailable; `confirms()` also returns true for the flag.

### D5. Spec deltas are ADDED requirements only

To avoid conflicting MODIFIED deltas with the unarchived changes above, each capability gets a new requirement. Four unarchived changes name version 3 as the version frames carry or keep: `add-wifi-drive` (wifi-drive: the bridge accepts only version 3 handheld frames), `add-handheld-controller` and `handheld-network-check` (handheld-controller: the handheld sends version 3) and `add-bench-simulation` (the version stays 3). Their capabilities are not in the main specs yet, so a MODIFIED delta cannot target them; the controller-link requirement instead states that version 4 applies in their place, and that the interoperation promised between version 3 builds (relay-pose-diagnostics' "bridge built before or after this change", pose-telemetry's reserved bytes "of either age") holds among version 4 builds and not across versions, and the code (one `VERSION` constant in C++ and one in Go, used by the bridge, the handheld and the probe alike) follows it. The older requirements that name a protocol version (diagnostics-telemetry's "version 3", pose-telemetry's "version 2" and "reserved in protocol version 3") describe the version that introduced those frames and fields, which stays true; they are not reworded here.

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
- The mismatched-version scenario rests on three checks: the golden rejected version 3 vector (every decoder), the C++ and Go decoders' previous-version rejection tests, and a link receiver test that version 3 command frames alone leave the link unseen, so the robot requests Sleep. A protocol test also proves that a flagged telemetry frame of the previous or the next version, with a valid CRC, is discarded, which is the telemetry half of the scenario. The bridge's own link-loss display after 1 s without valid telemetry is existing behaviour (the freshness timeout in `firmware/bridge/main/link.cpp`, which needs the board; the screen's link-lost composition is host-tested under status-display, Link loss shown) and is not retested here.
- The robot's status line is not specified by any capability (leg-actuation only requires its slew-limit change to leave it unchanged); its `body` word becomes `not started`, `running` or `stopped`, checked by a host test.
- The virtual robot's `status` answer keeps `running=0|1` (1 only while Running), which the desktop end-to-end tests read; the `spooder-hardware-check` skill's description of the robot's status line names the three words.
- link-fuzzing names the golden file, so it gets a MODIFIED delta for `protocol_v4.txt`; no other main spec or active change's spec names it.
- `reject_version_3_command` is a version 4 command frame (the layout is unchanged from version 3) with the version byte 3 and its CRC recomputed; the stopped vector `telemetry_body_stopped_walk` is an emergency stop while walking on the bridge link (Walk, Ripple, output off, no gesture). Since task group 4, `spooder-monitor --frames` prints that vector, and `telemetry_extremes` (all flags set), as `Stopped`, which the console's golden lines and transcript record; an unknown mode and gait (`mode 255  gait 255`) stay covered by a running frame in the console's tests.
- The bridge's redraw decisions (`firmware/bridge/main/display.cpp`: a region or setup line is redrawn only when a field's text or tone changes) are unchanged and need LovyanGFX, so no host test reaches them. Task 3.1's "a second refresh redraws nothing" is checked where this change can affect it: the same flagged telemetry composes the same text and tone in every header field and setup line, which are what those comparisons read. Moving the comparisons into `core/status` to test them on the host is a refactor of unchanged code, left out of this change.
- The header shows `STOPPED` with the link lost only while telemetry is fresh, like every robot value; with the link lost it shows the usual placeholder. The flag wins over a gesture field, though the robot sends gesture 0 while the body is not running.
- The desktop's calibration editor is not live while the body is stopped, even with Calibrate as its last mode (`session.editable` checks the flag), and its hint says `The robot is stopped.`: the robot applies calibration only while the body runs in Calibrate (`robot_tick`: `calibrating = run.runBody && ...`), so a stopped Calibrate is not live, as controller-link requires of the reported mode. The editor's hint is a place the app shows the body mode, which desktop-calibrator's requirement covers.
- The handheld's `Stopped` label is the exported `app.BodyStopped`; the gesture guard in `cmd/spooder-handheld` is tested with it, and `internal/app` tests that flagged frames with Stand or Perform last give it, which together cover "a flagged frame with last mode Stand or Perform makes gestures unavailable". The drive view keeps showing the last gait while stopped, as the handheld requirement names only the body mode.
- `--telemetry-every`'s help text names a stopped body among the changes that print at once.
- `tools/bridge-screenshots` sets no telemetry flags, so its images show a running body and stay unchanged.
- Task 5.1: `test/CLAUDE.md` already names `protocol_v4.txt` (task 1.1) and says nothing else this change affects, so it is unchanged. The `spooder-hardware-check` skill's "it is 2 now" was stale before this change; it is corrected to 4 with the other version references, since the flash-both-boards advice is what this change relies on. `QR_VERSION = 3` in the bridge's `display.cpp` (the setup QR code's symbol version) matches the task's stale-reference grep but is unrelated to the protocol.

- The "Before the first start" scenario is conditional on no request having started the body; it does not say when the body starts, which this change leaves unchanged (Non-Goals). Whether the request-gated start in `protection::supervise` matches locomotion's "Boot sequence" and power-protection's "Supply connected" predates this change and is not reconciled here.

## Follow-ups

- Reword the version 3 sentences of wifi-drive, handheld-controller and bench-simulation to version 4 once their changes are archived, dropping the precedence sentence from controller-link.
- Reword the version-numbered sentences in diagnostics-telemetry and pose-telemetry once `relay-pose-diagnostics` and `add-bench-simulation` are archived, if a single "current version" statement is wanted.
- The bridge's WiFi drive relay forwards frames without showing a body mode; if it gains one, it follows D4.
