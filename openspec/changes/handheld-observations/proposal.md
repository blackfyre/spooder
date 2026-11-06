# Proposal

## Why

The user's field notes after driving with the TrimUI pak list four things: the driving view does not show the gesture that is playing; the small keys beside the REQUESTED labels never show a press, unlike the footer's keycaps; audio feedback from the pak would help; and the supply voltage and current should read green, orange or red by how close they are to the robot's protection limits, on the pak and on the bridge's screen alike.

## What Changes

- **Driving view gesture.** The driving view gains a GESTURE cell beside BODY, from the newest telemetry, as the panels page's robot panel has. Today the driving view's pose already moves with a gesture, but its cells read only `Perform`, so which gesture plays is not shown on that page (design D1 gives the evidence).
- **Pressed hint keys.** The small keys beside each REQUESTED label press down while their control is held and its press reaches that value, as the footer's keycaps do.
- **Supply bands.** The supply voltage and current are coloured by band: ok (bf-ok green), warning (bf-warn amber) and fault (the orange, bf-accent-on-dark, which the Blackfyre palette and the bridge's `Tone::Bad` use for red), always with the value as the label. The bands come from the limits the robot itself reports in its diagnostics frames (brown-out, undervoltage, overvoltage, sustained and trip current), so a build's `power_limits.h` sets them and no display has numbers of its own. The handheld colours the robot panel's supply well and the driving view's supply cell; the bridge colours the drive page's voltage and current and the power page's newest values. One band rule in `core/status/supply_band` for the bridge and a Go copy for the handheld, both run over one hand-written fixture, `test/golden/supply_bands.txt`.
- **Sound feedback.** The pak plays short generated tones (about -12 dBFS, no new dependency: SDL's core audio, which gabagool already initialises) for the bridge's rumble events (fault latched, e-stop set and released, link lost and regained, gesture refused) and for a gesture accepted and a leave, at most once a second each, the more important replacing the less (design D4). `Select` + `B` switches the sound off and on, kept between runs.
- No protocol, robot firmware or gamepad-mapping change; the handheld layer gains `Select` + `B`.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `handheld-controller`: the drive screen requirement gains the driving view's gesture, the pressed hint keys and the supply bands; a new requirement adds the sound feedback. The capability is added by `add-handheld-controller` and modified by `add-handheld-gesture-overlay`, both merged but not yet archived; this delta builds on the latter's text.
- `status-display`: a new requirement for the supply bands on the bridge's screen.

## Impact

- `core/status/supply_band` (new), `core/status/status_screen` (the band tones), `platformio.ini` and `firmware/bridge/main/CMakeLists.txt` (the new source), `test/test_supply_band` (new), `test/golden/supply_bands.txt` (new), `test/test_status_screen`.
- `firmware/bridge/main`: the diagnostics frame's limits kept and handed to the status screen; `tools/bridge-screenshots` and `docs/images/bridge-*.png`.
- `desktop/handheld`: `internal/supplyband` (new, with its golden test), `internal/feedback` (new: the sound events, tones and setting), `internal/padmap` (`Select` + `B`), `internal/app` (the limits, fault bits and sound toggle in the screen model), `cmd/spooder-handheld` (`driveview.go`, `screen.go`, `sound.go`, the tests) and `docs/images/handheld-*.png`.
- `docs/handheld.md`, `docs/controls.md`, `docs/screenshots.md`, `desktop/CLAUDE.md`, `core/CLAUDE.md`, `test/CLAUDE.md`, `CHANGELOG.md`.
