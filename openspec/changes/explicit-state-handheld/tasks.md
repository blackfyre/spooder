# Tasks

## 1. Link baselines

- [ ] 1.1 Number the receiver's baselines in `internal/udplink` (`Event.Baseline`, `Quality.Baseline`, incremented by every rebaseline) and make `Link.Rebaseline` drain the event queue under the same lock (D1, D2). Verify with `go test ./internal/udplink` that the first frame, the first after `Rebaseline()` and the first after `Gap` each start a new baseline, that later frames keep it, and, on a real `Link` fed through `datagram`, that frames queued before `Rebaseline` are gone after it and the next datagram is queued with the next baseline number.

## 2. Leave reference

- [ ] 2.1 Record the leave's reference as the press's baseline and newest sequence number, order telemetry by baseline then sequence number, remove `refValid` and the separate drains after `Rebaseline` in the rejoin and the resume (`internal/app`, D2). Verify with `go test ./internal/app` that a rejoin during a leave with a frame injected between the rebaseline and the drain ends the leave on the second Sleep telemetry after the rejoin, before `LeaveTimeout`, that after the rejoin a Sleep telemetry frame, then a Sleep frame late in the new numbering, then a newer Sleep frame keep the leave waiting after the late frame and end it on the third (the late frame neither counts nor breaks the run), that this test fails with the old two-call rejoin, and that the existing leave, delayed-frame, resume and network-check tests still pass.

## 3. Documentation and gates

- [ ] 3.1 Update `desktop/CLAUDE.md` (the `internal/udplink` entry) and `CHANGELOG.md` under Unreleased (Fixed). Verify by reading them against the spec delta.
- [ ] 3.2 Run `openspec validate explicit-state-handheld --strict`, `mise run handheld-test`, `mise run handheld-build`, `mise run e2e --wifi-bridge` and `mise run handheld-package`. Verify all pass.
