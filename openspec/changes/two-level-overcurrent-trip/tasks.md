# Tasks

## 1. Protection core

- [x] 1.1 Add `tripPersistMs` and `ceilingCurrentA` to `protection::Limits` (after `tripCurrentA`), and the above-trip run to `State`. Latch `OVERCURRENT` on a reading above the ceiling, or on a reading above the trip current once the run has lasted the persistence time; keep the trip current as the condition for clearing. Verify with `test_protection` cases: a one-tick spike above the trip current below the ceiling does not trip; a run shorter than the persistence time, then a dip, does not trip; a run of the persistence time trips on the evaluation that completes it; a reading above the ceiling trips on the same step; and the existing cases adjusted.
- [x] 1.2 Build `Limits` with the new fields everywhere: `test_robot_tick`, `firmware/servo2040/src/config.h` (and through it the virtual robot). Verify that `mise run test` passes.

## 2. Build-time limits

- [x] 2.1 Add `SPOODER_CEILING_CURRENT_A` (13 A) and `SPOODER_TRIP_PERSIST_MS` (60 ms) to `power_limits.example.h` and `power_limits.h`, require them in `config.h`, and check sustained < trip < ceiling ≤ 15 A and persistence ≤ 200 ms. Verify that `mise run build` with and without `--serial` and `mise run virtual-robot` pass with the example limits, and that a limits file with the trip current above the ceiling fails to build with the message.

## 3. Integration and documentation

- [x] 3.1 Update the `spooder-hardware-check` overcurrent step and the `CHANGELOG.md` entry. Verify by reading them against the spec delta.
- [x] 3.2 Run the gates: `mise run test`, `mise run build` with and without `--serial`, `mise run virtual-robot` and `mise run e2e` (the overcurrent e2e test now waits through the persistence time). Verify that all of them pass.
