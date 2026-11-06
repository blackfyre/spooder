# Design

## Context

See proposal.md for the motivation; the four items come from the user's field notes after driving with `Spooder.pak` on the TrimUI. What exists:

- The drive screen (`desktop/handheld/cmd/spooder-handheld/screen.go`) has two pages. The panels page's robot panel shows the body mode, the output, the supply well (voltage and current), and cells for gait, gesture and source. The driving view (`driveview.go`) draws the robot from the newest pose frame and beside it `driveViewCells`: body, gait, output and supply.
- The REQUESTED panel's cells name, beside each label, the keys that change the value (`requestedCells`, drawn by `hintKey`): flat keys that never change. The footer's keycaps (`keycap`) sit down on their edge with their label in the orange text tone while `app.Screen.Held` has their control.
- The robot reports its configured protection limits in every diagnostics frame (`protocol::Diagnostics`: `minMv`, `maxMv`, `brownoutMv`, `sustainedCurrentMa`, `tripCurrentMa`), ten times a second, on every link. The limits are the build's `firmware/servo2040/src/power_limits.h` (the example: brown-out 4.8 V, minimum 5.2 V, maximum 6.8 V, sustained 6 A, trip 9 A, for a 6 V BEC feeding the MG996R servos; the 2S LiPo is planned behind that BEC, which the board does not sense). The handheld keeps the newest diagnostics frame (`udplink` per-type freshness); the bridge decodes diagnostics frames and ignores them outside WiFi drive's relay.
- The bridge's drive page colours its voltage and current values amber only while `peak_hold` reports a new extreme beyond its margin (status-display: held extremes); its tones map to the Blackfyre palette in `firmware/bridge/main/screen_draw.cpp`: Good bf-ok, Warn bf-warn, Bad bf-accent-on-dark.
- The bridge already gives gamepad feedback for events, as rumble (`core/gamepad_mapping/rumble`): fault latched, e-stop on and off, link lost and regained, gesture refused.

## Goals / Non-Goals

**Goals:**
- Each item fixed at its cause, with a test that fails without the fix.
- The supply bands derived from the limits the robot enforces, one rule for the bridge and the handheld, kept equal by a shared fixture.
- Colour is never the only signal: the value stays the label.

**Non-Goals:**
- Any protocol, robot firmware or mapping change; bands for the 2S pack itself, which the board does not measure.
- Changes to the gesture scripts or `core/movement`.

## Decisions

### D1. The driving view names the gesture playing

**What the user sees.** Reproduced against the virtual robot through the virtual bridge (`--wifi-bridge`) with the handheld's run loop (the e2e harness): Y stands the robot, L2 + A requests say hi, and the screen model is sampled every 100 ms. The model's pose follows the gesture at the pose frame rate (leg 1's femur rises from 59° to 104° and its coxa waves between −17.5° and 17.5°), and the model's telemetry reports body `Perform` and gesture `say hi` (id 1) throughout. The driving view drawn from those poses moves the leg (rendered with the screenshot test's software renderer at say hi's and wiggle's frames). So the drawing is not the fault.

**Root cause.** `driveViewCells` takes only the body mode, gait, output and supply: the gesture, which the model carries (`app.Robot.Gesture`) and the robot panel shows, is never shown on the driving view. The page reads `Perform` and nothing names the gesture selected and playing; the gesture overlay names it only while L2 is held, and it covers the view.

**Fix.** A GESTURE cell beside BODY, from the same telemetry as the robot panel's (`None` when none plays, `—` without telemetry), in the value tone (muted while stale). The cells become BODY, GESTURE, GAIT, OUTPUT, SUPPLY in two columns. A unit test checks the cells name the gesture from `app.Robot`; it fails without the cell.

Alternative considered: animating a marker or the gesture's name over the drawing. Rejected: the cell is the same pattern as the robot panel and needs no new element.

### D2. Pressed hint keys

Each hint key carries its control (`padmap.Buttons`) as the footer's keycaps do. While the control is held, the key presses: it sits down by its edge and its label turns to the orange text tone, as `keycap` does on a press; at rest the key gains a small edge so the press reads as a movement. A key is not pressed while its control does something else: Start, X and Y while Select is held (the e-stop, the page toggle and the fault clear), A, B, X and Y while L2 is held (the gesture layer). The rule, `hintPressed(control, held)`, is unit-tested with the footer's test pattern.

The mapping's other masks (a button pressed with Select stays masked until released, even after Select is let go) are not reproduced: the screen model carries only what is held, and the key reads as pressed for at most that moment.

### D3. Supply bands

**Source of the limits.** The robot's own diagnostics frame, not constants in the displays: the limits are a per-build choice (`power_limits.h`), and a display with its own numbers would drift from what protection enforces. Until a diagnostics frame has arrived, or when its limits are not ordered (`brownoutMv < minMv < maxMv`, `0 < sustainedCurrentMa < tripCurrentMa`), the band is unknown and the value keeps today's tone. The limits are kept once known (they change only with a new build), also while diagnostics are stale.

**The rule** (`core/status/supply_band`, and `desktop/handheld/internal/supplyband`):

| Reading | Fault | Warning | Ok |
|---|---|---|---|
| Voltage | at or below `minMv` (undervoltage; brown-out is below it) or at or above `maxMv` (overvoltage) | within `minMv − brownoutMv` of either limit | otherwise |
| Current | at or above `tripCurrentMa` (overcurrent) | at or above `sustainedCurrentMa` (the average that puts the robot to sleep) | otherwise |

With the example limits: voltage fault ≤ 5.2 V or ≥ 6.8 V, warning 5.2–5.6 V and 6.4–6.8 V, ok 5.6–6.4 V; current warning from 6 A, fault from 9 A. The current bands are the limits themselves; the voltage warning margin is the one value the limits do not give, and the user chose this one (below).

**One fixture.** `test/golden/supply_bands.txt`, hand-written as `gamepad_mapping.txt` is: `limits <brownout> <min> <max> <sustained> <trip>` lines followed by `voltage <mV> <band>` and `current <mA> <band>` cases, including each boundary and unordered limits (`unknown`). `test/test_supply_band` runs the C++ rule over it, the handheld's `internal/supplyband` test the Go copy; `SPOODER_SUPPLY_BANDS_GOLDEN` names another copy.

**Colours.** The Blackfyre palette has no red: its alarm colour is the orange (bf-accent, and bf-accent-on-dark for text on the dark grounds), which the bridge's `Tone::Bad`, the handheld's e-stop chip and its weak-signal dot already use. So: ok bf-ok (green), warning bf-warn (amber), fault bf-accent-on-dark (orange). The user's "orange" warning is bf-warn's amber; the "red" fault is the system's orange. A stale or absent value stays muted, whatever its band.

**Where.** Handheld: the robot panel's SUPPLY and CURRENT figures and the driving view's SUPPLY cell. Bridge: the drive page's voltage and current, and the power page's newest voltage and current and its window's lowest voltage and highest current. On the bridge's drive page the held-extreme warning keeps its meaning: the tone is the band's, raised to the warning tone while `peak_hold` warns and the band is ok.

### D4. Sound feedback

**What the pak can play.** gabagool initialises SDL with `SDL_INIT_AUDIO` (`pkg/gabagool/internal/sdl.go` in v2.26.6), and go-sdl2 v0.4.40 binds SDL's core audio API (`sdl.OpenAudioDevice`, `sdl.QueueAudio`, `sdl.ClearQueuedAudio`). That plays short PCM buffers with no new dependency; SDL_mixer is not in the device build image and would be a new pinned package. The audio driver SDL selects on the tg5040 has not been checked on the device.

**Events**, the bridge's rumble events (`core/gamepad_mapping/rumble`) with its rules, so both controllers tell the driver the same things, plus two of the pak's own:

| Event | Tone | Priority |
|---|---|---|
| Fault latched (a new fault bit in fresh telemetry, the e-stop's excepted) | three 90 ms beeps at 330 Hz, 60 ms apart | 5 |
| E-stop set | 880 Hz 120 ms, then 440 Hz 200 ms | 4 |
| Link lost (telemetry stale) | 300 Hz 250 ms | 3 |
| Leaving (a leave starts: Menu or Power) | 660, 440, 330 Hz | 3 |
| Gesture refused (not playing 600 ms after the request, rumble's rule) | 200 Hz square, 150 ms | 2 |
| E-stop released | 440 Hz 80 ms, then 880 Hz 80 ms | 1 |
| Link regained | 1200 Hz 80 ms | 1 |
| Gesture accepted (telemetry reports the requested gesture playing, after a moment it did not) | 1500 Hz 50 ms | 1 |

As in rumble: the first update only takes what is already so (latched faults, the e-stop, the gesture counter) as known; after a link gap the faults reported are the new baseline; a gesture request pending when the link is lost sounds only the loss; one event sounds at most once per second (`RepeatAfter`, rumble's `REPEAT_MS`) and an event that cannot sound within a second of happening is dropped; a higher priority replaces a lower one still playing, and a lower one waits for a higher one to end.

**How.** `desktop/handheld/internal/feedback` (no SDL): the detector over the screen model (`FromScreen`: the fault bits, the pak's e-stop and gesture request, the gesture playing, freshness, the leave), the tone table, `Render` (sine or square segments with 5 ms ramps, 16-bit mono at 48 kHz, peak 0.25 of full scale, about -12 dBFS), rendered once at start, and a `Player` interface. `cmd/spooder-handheld/sound.go` implements it on one SDL audio device opened after gabagool's start (none: the pak drives silently and says so on stderr), a new tone replacing what is queued; it updates at each redraw (every 50 ms) and lets a queued tone finish, for at most 500 ms, before closing at exit. The detector is not kept in step with rumble by a fixture: the pak's inputs differ (its own e-stop and counters, staleness at 500 ms, the leave) and the tests pin its rules.

**Sound on/off.** `Select` + `B` toggles the sound in the handheld layer (`padmap.Handheld`, masked like `Select` + `X`, so the stick mode does not change), shown as a `SOUND OFF` header chip while off. The choice is kept between runs as an empty file `sound-off` beside the binary (the pak's folder). With the sound off, events are still judged, so switching it on does not play old ones. The device's volume sets the level; whether NextUI's mute reaches SDL output is checked on the device (task 7.2).

### Decisions taken by the user (2026-11-06)

- D3: the voltage warning margin is `minMv − brownoutMv` on both sides, as proposed; fault colour is the orange (bf-accent-on-dark), warning bf-warn, ok bf-ok, no new red token.
- D4: every event in the table sounds, accepted gestures included; generated tones at about -12 dBFS with the 1 s repeat guard and no new dependency; a sound on/off setting in the pak, since NextUI's mute reaching SDL output is unverified.

## Risks / Trade-offs

- [Bands absent until diagnostics arrive] → the value keeps today's tone for the first 100 ms or so; with a robot firmware too old to send diagnostics the bands never show. Acceptable: protocol version 3 has always carried diagnostics.
- [The bridge's drive page shows green almost always] → that is the point of the band; the held-extreme warning still raises it to amber on a new dip.
- [Voltage band without a limits-derived warning level] → the margin is derived from two limits and was chosen by the user; it is one expression in each implementation, pinned by the fixture.
- [NextUI's mute may not reach the pak's tones] → `Select` + `B` silences them whatever the device does; the on-device check (task 7.2) settles it.
- [A tone at exit cut short] → close waits for the queued tone, at most 500 ms; a suspend or power-off may still cut the leaving tone.
