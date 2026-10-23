# Tasks

## 1. Model

- [x] 1.1 Add `core/movement/centre_of_gravity` (mass table, forward kinematics on the schematic mounts, the estimate, the support hull and margin) to the native test build. Verify host tests: a symmetric stand on the centre line, a lifted leg moving the centre by its share of the mass, the battery adding its mass towards its position, and fewer than three feet giving no margin.
- [x] 1.2 Write `test/golden/centre_of_gravity.txt` from the model (`SPOODER_WRITE_GOLDEN=1`) and check it otherwise. Verify `mise run test`.
- [x] 1.3 Check the estimated centre of gravity, with and without the battery, in the gesture support test. Verify `mise run test`.

## 2. Desktop

- [x] 2.1 Add the Go model to `desktop/internal/legview`, tested against the golden file. Verify `mise run desktop-test`.
- [x] 2.2 Draw the centre of gravity, the drop line, the support polygon, the readout and the battery control in the four-view window; tests for the battery default and toggle, and screenshots. Verify `mise run desktop-test` and `mise run desktop-build`.

## 3. Integration

- [x] 3.1 Update `CLAUDE.md`'s layout, the `spooder-leg-kinematics` skill where it lists the movement files, and `CHANGELOG.md`. Verify by reading them against this change.
- [x] 3.2 Run the gates: `mise run test`, `mise run desktop-test`, `mise run desktop-build`, `mise run e2e`. Verify that all of them pass.
