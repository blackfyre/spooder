## 1. Core content

- [x] 1.1 Add `core/status/power_history` (600 samples at 100 ms, gaps while the link is lost, newest value, window minimum voltage and maximum current, per-column min/max for a trace) and verify with a new `test/test_power_history` covering sampling, gaps, the window's extremes and samples leaving the window.
- [x] 1.2 Add `core/status/page_button` (press counted on release after 30 ms to under 1 s, three pages cycling) and verify with host tests for bounce, long holds and the cycle.
- [x] 1.3 Replace `status_screen::compose` with `composePage` and the `View` model for the drive, power and link pages, the status strip with its priority, battery levels and the link-page rates; keep `composeSetup`. Verify with the updated `test/test_status_screen` (every spec scenario that is content, including the fault strip on each page, link loss, no battery report and the rates with counter wrap).

## 2. Bridge firmware

- [x] 2.1 Add `firmware/bridge/scripts/make_fonts.py` and the generated VLW headers (JetBrains Mono 10 and 12 px, Archivo SemiBold 14 and 22 px) with the OFL licences under `firmware/bridge/main/fonts/`; verify that `mise run build-bridge` passes and the size check reports the new headroom.
- [x] 2.2 Rework `firmware/bridge/main/display` to draw a `View` into the reusable region sprite with the Blackfyre palette, redrawing only changed regions, and the start-up log in the new palette; feed the power history and the link rates from `update()`. Verify that `mise run build-bridge` passes.
- [x] 2.3 In `bridge.cpp`, read the left button after the start-up sequence into the page button, and pass the gamepad's battery level. Verify that `mise run build-bridge` passes and that the setup-mode hold during the logo is unchanged in code.

## 3. Integration and documentation

- [x] 3.1 Update the `spooder-hardware-check` skill (the bridge screen's pages and the left button) and `CHANGELOG.md`; verify by reading them against the spec delta.
- [x] 3.2 Run the gates: `mise run test`, `mise run build-bridge`, `mise run build` (shared core unchanged in behaviour). Verify that all pass.
- [x] 3.3 On the bridge: flash it, check each page against the spec (pages cycle, the status strip on every page, link loss when the robot is unplugged, the power traces during a stand, the link rates), and that the gamepad still drives without delay.
- [x] 3.4 Split the region drawing into `firmware/bridge/main/screen_draw` (any LovyanGFX surface) and add `tools/bridge-screenshots` with `mise run bridge-screenshots`, rendering each page and the fault, e-stop and link-lost states to `docs/images/bridge-*.png` for the readme; verify that the bridge still builds and the images show each page as on the board.

## 4. Held extremes

- [x] 4.1 Add `core/status/peak_hold` (held maximum or minimum: at once beyond, 3 s hold, then towards the reading at a set rate; reset while the link is lost) and verify with `test/test_peak_hold` covering the spec scenarios.
- [x] 4.2 Mark the held current and voltage on the bridge's drive bars (the view's bars carry the held fraction, the display feeds the holds from telemetry and draws a tick); verify with `test_status_screen`, `mise run build-bridge` and the regenerated screenshots.
- [x] 4.3 Add the held lines to the desktop calibrator's current and supply charts, computed from the history with the same rule; verify with Go tests of the same scenarios, `mise run desktop-test` and `mise run desktop-build`.
- [x] 4.4 Update `CHANGELOG.md` and the skill text where the drive page and charts are described; rerun `mise run test`, `mise run e2e`; flash the bridge.
- [x] 4.5 Warn on the bridge's drive page when the current beats the held peak by more than 0.1 A or the voltage the held low by more than 0.05 V (kept 1 s after the last new extreme); verify with `test_peak_hold`, `test_status_screen` and `mise run build-bridge`, and flash the bridge.
