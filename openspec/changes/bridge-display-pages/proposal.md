## Why

The bridge's 240×135 colour screen shows eight lines of plain text in four colours. While driving, the operator has to read sentences to find the mode, the supply and a fault. Today's bench work also showed that supply sag and current peaks matter during a stand, and the bridge cannot show their recent history. Everything needed is already in the telemetry the bridge receives, so the screen can show far more without a protocol change.

## What Changes

- **Graphical drive page in the Blackfyre look.** The dark app palette, Archivo and JetBrains Mono, a mode chip with the gait or gesture, link and gamepad indicators with the gamepad's battery, voltage and current as values with bars, the requested height and speed as bars with stride and stick mode, and a status strip. The current text lines are replaced.
- **Pages on the left button.** A short press of the left button (GPIO0) cycles Drive → Power → Link. It counts only after the start-up sequence; holding it during the logo still enters setup mode.
- **Power page.** The supply voltage and current over the last 60 s as traces, with the current value, the lowest voltage and the highest current in that window. The history is kept on the bridge from the telemetry it receives (10 Hz samples, so short peaks between samples are not seen).
- **Link page.** The frames each side's decoder accepts and rejects per second, the telemetry's age, which source the robot follows (bridge, USB in control, conflict), and the gamepad's connection and battery.
- **Held extremes.** The drive page's current bar marks the recent highest current and its voltage bar the recent lowest voltage: held 3 s, then easing back to the reading. The desktop calibrator's current and supply charts get the same held value as a named line.
- **Safety state on every page.** An active emergency stop, any latched fault and a lost link are shown in the status strip on every page, labelled in words, not by colour alone.
- The setup-mode screen and the start-up logo are unchanged; the start-up log takes the new palette.
- No protocol change: only the bridge is flashed.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `status-display`: the status content is organised into pages switched by the left button, with the safety state on every page; adds the power history and link pages, held extremes on the drive bars, and the visual style.
- `desktop-calibrator`: the current and supply charts show the held extremes.

## Impact

- `core/status/status_screen`: composes the content of each page (texts, tones, bar fractions, the status strip) instead of eight lines; host tests updated.
- New hardware-free modules in `core/status/`: the power history (60 s of voltage and current samples) and the page selection with the button's debounce, host-tested.
- `firmware/bridge/main/display`: drawing with LovyanGFX into one reusable region sprite, smooth fonts, redraw of changed regions only.
- `firmware/bridge/main/bridge.cpp`: reads the left button after the start-up sequence and the gamepad's battery level.
- New generated font data (Archivo SemiBold, JetBrains Mono, SIL OFL, from the copies in `desktop/internal/ui/fonts`) with the script that makes it; flash use grows by the fonts (the app has 60 % of its partition free).
- Documentation: the `spooder-hardware-check` skill (the bridge screen and its button) and `CHANGELOG.md`.
