## 1. Mapping mode state

- [x] 1.1 Replace `Mapper::baseMode`, `sleeping` and `calibrating` in `core/gamepad_mapping` with `ModeState` and its transition function (design D1, D2), the requested mode derived from it; add a host test in `test/test_gamepad_mapping` of all twenty state/event cells and the counter steps, and an `update()` test of the scenario "Calibrate hold completes with y pressed" (Sleep requested, counter advanced by two, then Stand after `y`). Verify `mise run test` passes, including `test_gamepad_mapping_golden`, and `git diff --exit-code main -- test/golden/gamepad_mapping.txt` is empty
- [x] 1.2 Port `ModeState` and the transition function to `desktop/handheld/internal/padmap/mapping.go`; move `Handheld.RequestSleep` into `Mapper.RequestSleep` as a transition (design D3) and call it from `Handheld`; add a Go table test of every cell including the sleep request, and an `Update()` test of the scenario "Calibrate hold completes with y pressed". Verify `mise run handheld-test` passes, including the golden test

## 2. Review rule and changelog

- [ ] 2.1 Add "implicit state machine" to the bug classes in `REVIEW.md`, covering C++ and Go: flags, timestamps, counters or sentinels doubling as state, states written outside their transition function; record the change in `CHANGELOG.md` under Unreleased, naming `ModeState`. Verify `grep -n "implicit state machine" REVIEW.md` and `grep -n "ModeState" CHANGELOG.md` find the entries and `openspec validate explicit-state-core` passes
- [ ] 2.2 Replace the `Input mapping` block of `openspec/changes/wave-default-gait/specs/gamepad-control/spec.md` with this change's (design D4), so either archive order gives the same requirement. Verify `diff openspec/changes/wave-default-gait/specs/gamepad-control/spec.md openspec/changes/explicit-state-core/specs/gamepad-control/spec.md` is empty and `openspec validate wave-default-gait` passes

## 3. Gates

- [ ] 3.1 Verify `mise run lint`, `mise run test`, `mise run build` (with and without `--serial`), `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build`, `mise run handheld-test`, `mise run handheld-build`, `mise run handheld-package`, `mise run e2e` and `mise run e2e --wifi-bridge` pass
