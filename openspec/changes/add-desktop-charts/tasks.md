# Tasks

## 1. Diagnostics frame

- [x] 1.1 Add the `Diagnostics` frame (type 6) to `core/protocol` (struct, encode, decode, payload size) with golden vectors, regenerated deliberately (`SPOODER_WRITE_GOLDEN=1 mise run test`). Verify that `test_protocol` and `test_protocol_golden` pass.
- [x] 1.2 Collect the window's diagnostics in `robot_tick` (motion share minimum, backstop count, tick time maximum and mean from `Inputs::previousTickUs`, averaged current, limits, budget and rated speed), and send the frame on USB only at tick 4. Measure the tick time in `firmware/servo2040/src/main.cpp` and in the virtual robot. Verify with `test_robot_tick` cases (USB only, its tick, its contents), `mise run build` and `mise run virtual-robot`.
- [x] 1.3 Add the frame to `desktop/internal/protocol` (decode, encode, summary), tested against the golden file. Verify with `mise run desktop-test`.

## 2. History

- [x] 2.1 Add `session.History`: the three ten-minute rings, cleared on a new connection, window queries, rates from wrapping counters, joint speeds from consecutive poses. Verify with session tests (rates across a wrap, joint speed, the ten-minute cap, cleared on connect).
- [x] 2.2 Add `WriteCSV` and the file placement (downloads directory, temporary file and rename). Verify with tests on a temporary directory (header, row count, values).

## 3. Charts tab

- [x] 3.1 Add the charts tab: the chart panel (axis, gridlines, reference lines, series, legend, decimation), the timeline band, the window selector, the shared hover crosshair and the CSV button. Verify with window tests (tab, window selector, hover, CSV notice, no-diagnostics message) and screenshots in light and dark mode, including the minimum window size.

## 4. Integration and documentation

- [x] 4.1 Add an e2e test: walking on the virtual robot yields diagnostics, with a motion share below 1000 at dial 100, and charts history. Verify with `mise run e2e`.
- [x] 4.2 Update `CHANGELOG.md`, `CLAUDE.md` (protocol frames, desktop layout) and the `spooder-hardware-check` and `spooder-firmware-shape` skills. Run every gate (`mise run test`, `mise run build` with and without `--serial`, `mise run build-bridge`, `mise run desktop-test`, `mise run desktop-build`, `mise run e2e`) and verify that all pass.
- [x] 4.3 Flash the robot and check the charts against the real board. Verify that diagnostics arrive at about 10 Hz and the tick time is as measured. Flashed; `mise run monitor --frames` shows diagnostics every 100 ms, with the tick at 0.4–0.8 ms idle (no controller). The walking tick under load is for the bench, with the charts tab.
