## Context

`core/status/status_screen::compose` turns the bridge's inputs into eight text lines with a tone each, and `firmware/bridge/main/display.cpp` draws them with LovyanGFX's Font2, redrawing changed lines every 250 ms from `loop()`. Command frames leave from the `robot_link` task, so drawing time in `loop()` does not delay them. The ST7789 runs at 40 MHz SPI, 240×135 in landscape. The bridge image uses 1.52 MB of a 3.88 MB partition, and static RAM is 97 KB of 320 KB, with BTstack and Bluepad32 taking more from the heap at run time. The left button (GPIO0) is read only during the logo, for setup mode. Bluepad32 reports a gamepad's battery as 0 (not available), 1 (empty) to 255 (full).

## Goals / Non-Goals

**Goals:**
- Content stays hardware-free and host-tested: what each page shows, the power history and the page button.
- Exact Blackfyre colours and anti-aliased fonts without flicker, in bounded RAM.

**Non-Goals:**
- The setup-mode screen and the boot logo (only the start-up log takes the new palette).
- The robot's configured limits on the screen: they are only in the USB diagnostics frame. Bars use fixed scales.
- A light theme or brightness control.

## Decisions

### Content as a view model in core
`status_screen` gains `composePage(const Inputs&, Page, View&)`. `View` holds the status strip (text and tone), the page marker, and per page the fields it shows: texts with tones, bar fractions (0–1), and for the power page the trace points and the window's extremes. `Inputs` gains the gamepad battery, the page's rates and a pointer to the power history. The drawing code only lays out a `View`; host tests check the content. The eight-line `compose` and its tests are replaced; `composeSetup` stays as it is.

*Alternative:* compose drawing commands in core. Rejected: it moves layout into core without making it testable in a useful way.

### Power history in core
`status/power_history`: a ring of 600 samples (60 s at 100 ms), each voltage and current in mV and mA, or a gap. `record(nowMs, fresh, telemetry)` adds one sample per 100 ms of time passed (a gap while the link is lost). A late call gives its reading to at most the five newest missed samples, enough to ride out a redraw delaying `loop()`; older missed samples are gaps, so a stall never invents readings, and a pause longer than the window fills it once. It answers the newest sample, the window's lowest voltage and highest current, and per-column min/max for a trace of a given width. 600 × 4 bytes is 2.4 KB. It is fed from `loop()` on every pass, independent of the page shown.

### Page button in core
`status/page_button`, like `setup_entry`: debounced in time, a press counts on release after 30 ms to under 1 s. `bridge.cpp` reads GPIO0 only after the start-up sequence (`display::showingLogo()` false and the boot log done), so a setup-mode hold during the logo is never seen as a press.

### Rates for the link page
The bridge keeps the previous telemetry's robot-side counts and its own decoder counts once a second and shows the differences (16-bit robot counters wrap; differences are taken modulo 2^16).

### Drawing: one region sprite, regions redrawn on change
The screen has fixed regions: header (mode chip, indicators), body (page content), status strip, page marker. Each region is drawn into one reusable 16-bit `LGFX_Sprite` sized for the largest region (240×82, about 39 KB, allocated once at start-up), then pushed. A region is redrawn only when the part of the `View` it shows changed (compared field by field). 16-bit keeps the token colours exact and lets the fonts anti-alias against the region's ground; drawing off screen removes flicker.

*Alternatives:* a full-screen 16-bit sprite (65 KB, too much next to BTstack's heap); an 8-bit or 4-bit palette sprite (cannot hold the tokens and the font's anti-aliasing shades).

### Fonts: generated VLW data, checked in
LovyanGFX draws VLW smooth fonts from a byte array. A script, `firmware/bridge/scripts/make_fonts.py` (Python with Pillow, run by hand, not by the build), renders the printable ASCII range plus `·` from the TTFs already in `desktop/internal/ui/fonts` into VLW headers under `firmware/bridge/main/fonts/`, with the OFL licences next to them: JetBrains Mono at 10 and 12 px for labels, Archivo SemiBold at 14 and 22 px for values and the mode. That adds well under 100 KB of flash.

### Fixed bar scales
Voltage 4.5–7.5 V and current 0–15 A (the board's measurable range). Height and speed 0–100. Traces on the power page scale the current from 0 to the larger of the window's highest current and 2 A, and the voltage over the window's range, at least 0.5 V.

### Layout

| Region | Drive page | Power page | Link page |
| --- | --- | --- | --- |
| Header, 22 px | mode chip (`STAND · TRI`, or the gesture; in the 10 px font when the widest case needs it), `PAD` with battery, link state | same | same |
| Body, 82 px | sensors when available; V and A values with bars; H and SPD with bars; stride and stick, `OUT ON`/`OUT OFF` | V and A traces with the newest value, lowest V and highest A, "10 HZ SAMPLES · 60 S" | robot and bridge accepted and rejected per second, telemetry age, source, gamepad and battery |
| Status strip, 24 px | up to two lines of 37 characters: e-stop, faults, link lost, conflict or USB in control; else "NO FAULTS" | same | same |
| Page marker, 7 px | three dots, the shown one in `bf-accent-on-dark` | | |

The status strip's order: the e-stop, the faults, then link lost or the source state. Words are two spaces apart, so every fault with the e-stop and the source state fits two lines. The e-stop, faults, link lost and a conflict fill it with `bf-accent` and `bf-ink` text; USB in control uses `bf-warn`; "NO FAULTS" is `bf-ok` on `bf-ink-raised`. The voltage takes `bf-series-1-dark` and the current `bf-series-2`, in bars and traces; the accent never charts. The fonts carry no letter-spacing (VLW has none), so labels are not tracked out as on the web.

### Held extremes
One rule on both screens: a held maximum (current) or minimum (voltage) moves at once to a reading beyond it, holds 3 s from when it was reached, then moves towards the reading at 2 A/s or 0.5 V/s until it meets it. On the bridge, `core/status/peak_hold` keeps the state, fed from `display::update` on every pass (telemetry changes at 10 Hz); a lost link resets it. The drive page draws it as a 2 px tick in `bf-surface` across the bar. On the desktop it is recomputed from the history's telemetry samples each frame (a pure function, no new session state), and drawn as a dashed line in the series' colour, named with its value like the reference lines. Both sides are tested against the same scenarios.

On the bridge only, the hold also warns: a reading beyond the held value by more than a margin (0.1 A, 0.05 V) sets `warning`, which every further new extreme extends to 1 s after it; the drive page then shows that value in `bf-warn`. The margin keeps idle noise from flickering amber once the held value has eased back to the reading; the 1 s keeps a single 10 Hz spike visible at the 4 Hz redraw. A slow climb of under 0.1 A per sample does not start a warning; that is accepted.

## Risks / Trade-offs

- [The region sprite's 39 KB cannot be allocated once BTstack has taken the heap] → Allocate it in `display::setup()`, before Bluepad32 starts; if it still fails, draw straight to the panel (flicker, but correct) and log it.
- [The 10 Hz history misses peaks between samples] → The power page says so; the trip itself acts on every 20 ms reading on the robot.
- [Fonts look different from the desktop at small sizes] → Sizes chosen at the screen's pixel grid and checked on the board.
- [GPIO0 is a strapping pin] → It is only read, after boot, as today during the logo.

## Migration Plan

Bridge-only flash (`mise run upload-bridge`); no protocol change, so the robot is unaffected. Rollback: flash the previous bridge build.
