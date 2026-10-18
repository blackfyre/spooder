# Tasks

## 1. Shared tick

- [x] 1.1 Add `core/robot/robot_tick.{h,cpp}` (design D1) with the loop body moved from `main.cpp`, the link endpoints and the `Port` interface, plus the pose-frame hook (D2). Verify it builds in `spooder_core` with `-Wall -Wextra` and in the native test env.
- [x] 1.2 Reduce `main.cpp`, `link` and `usb_link` to adapters calling the tick. Verify `mise run build` and `mise run build --serial` without warnings, and an independent review comparing old and new loops call by call.
- [x] 1.3 Host tests in `test/test_robot_tick/`: e-stop latency, a stale command never starting the body, conflict → Sleep, calibration only in Calibrate, telemetry every fifth tick, report scheduling, frame sequence numbers only on sent frames, status text. Verify `mise run test`.

## 2. Virtual robot

- [x] 2.1 `tools/virtual-robot/` with CMake and the `virtual-robot` mise task (D3): terminals, link path, the timing modes. Verify it builds without warnings and a pty client reads text and telemetry.
- [x] 2.2 Simulated outputs, supply, current, simulated bridge, control commands and observation output. Verify by hand in stepped mode: overvoltage trips, e-stop latches, unplug and plug.

## 3. Desktop and end to end

- [ ] 3.1 `--port` (D4): `link.WithPort`, `ui.Env.Port`. Verify `mise run desktop-test` with unit tests for listing, opening and preselection.
- [ ] 3.2 `desktop/e2e` tests (D5) and the `e2e` mise task: observe, control and Calibrate, a confirmed calibration change with the applied value at ≤ 60°/s, e-stop latching and blocking clears, release → Sleep, overvoltage. Verify `mise run e2e` passes and the tests skip without the binary.
- [ ] 3.3 Screenshots against the live robot (D6), light and dark. Verify the PNGs exist and show the states.

## 4. Documentation

- [ ] 4.1 Update `CLAUDE.md`, `spooder-firmware-shape` (the tick's location) and `spooder-hardware-check` (the virtual robot before the board). Verify every path and symbol mentioned exists, and `openspec validate add-virtual-robot`.
