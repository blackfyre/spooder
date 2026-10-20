# Design

## Context

The robot sends telemetry (supply, current, mode, faults, frame counts) on every link every fifth 20 ms tick, and the pose frame on USB two ticks later. The calibrator's session keeps only the latest of each. Gio has no chart widgets; the legs pane already draws custom vector graphics with `clip.Path`. The robot's internals the charts need (the budget's motion share, backstop count, tick time, limits, averaged current) live in `robot_tick` and `protection` and are not on the link.

## Goals / Non-Goals

**Goals:**
- Every chart readable at the minimum window size, in light and dark mode.
- Diagnostics that cost the 20 ms loop almost nothing.

**Non-Goals:**
- Charts for the bridge on its own screen, or in the portal.
- Configurable chart sets, zooming beyond the three windows, or a pause control. Hovering reads past values, and CSV covers deeper analysis.
- Recording across reconnections: a new connection starts a new history.

## Decisions

### A USB-only diagnostics frame, version unchanged

Type 6, `Diagnostics`, with a 24-byte payload of little-endian 16-bit fields:
1. motion share ‰;
2. backstop ticks;
3. tick maximum µs;
4. tick mean µs;
5. averaged current mA;
6. trip current mA;
7. sustained current mA;
8. minimum mV;
9. maximum mV;
10. brown-out mV;
11. budget °/s;
12. rated speed °/s.

It follows the pose frame's precedent: an additional type of version 3, sent on USB only, which other receivers ignore. That leaves the bridge as it is. It is sent at `telemetryTick == DIAGNOSTICS_TICK` (4), so telemetry (0), pose (2) and diagnostics never share a tick's USB budget.

*Alternative:* extend telemetry. Rejected: telemetry also goes to the bridge, and the bridge would need a version bump and a reflash for data it never shows.

### The tick time is measured by the caller

`Inputs::previousTickUs` is the working time of the previous `tick()` call, measured by the board's `main.cpp` (`time_us_32`) and by the virtual robot (`steady_clock`). `robot_tick` keeps the window's maximum and mean. The tick cannot time itself, because the hardware calls it makes are part of the work.

### The session keeps three rings; the UI derives the series

`session.History` holds three time-stamped rings of ten minutes each (6,000 entries at 10 Hz): telemetry with the host link's counts at arrival, diagnostics, and per-pose joint speeds plus servo angles. Rates come from the wrapping 16-bit counters and the elapsed time between consecutive samples. Joint speed is |Δ servo angle| / Δt between consecutive pose frames, maximised per joint kind over the six legs. The budget limits the IK angles with the calibration offsets included, which is the servo angle, and an offset ramp really moves the servo. Being a 100 ms average, it reads lower than a single tick's peak. A new connection clears the history. The UI asks for a window's samples and builds polylines, so the session stays free of drawing.

### Charts drawn by hand in Gio, one value axis each

Each chart is a fixed-height panel with:
- a title and unit;
- 3–4 recessive horizontal gridlines with tick labels;
- named reference lines as muted dashed rules;
- 2 px series lines;
- a legend with the latest (or hovered) values.

The time axis is shared, with labels on the bottom chart only. The timeline is a band of labelled mode segments, an output bar and fault marks in the app's fault colour, with text.

The series colours are a three-slot categorical set (blue, magenta, green), validated with the dataviz validator on the app's chart surfaces for all pairs:
- light on #F3F2EE: #2A78D6, #D55181, #008300 pass every check;
- dark on #2C2C29: #3987E5, #D55181, #1A9A1A pass every check.

Orange is avoided because it is the app's accent, reserved for the e-stop and active controls. Text stays in the palette's ink.

### Hover is a shared crosshair

A pointer over any chart sets a hover time. Every chart draws a vertical rule there, and its legend shows the values at the nearest sample instead of the latest. Leaving the charts returns them to the latest values.

### CSV export

`History.WriteCSV` writes one row per telemetry sample. Each row carries the latest diagnostics and pose-derived values at or before its time. The header names the column units. The file is written to a temporary file and renamed, in `$XDG_DOWNLOAD_DIR`, `~/Downloads` or the home directory, as `spooder-<connection start>.csv`.

## Risks / Trade-offs

- [The diagnostics add USB traffic: 31 bytes at 10 Hz] → Negligible next to the pose frame (117 bytes at 10 Hz). Frames that do not fit are dropped whole, as before.
- [Drawing up to 6,000 points per series at 60 fps] → Polylines are decimated to about one point per pixel column (minimum and maximum per column, so spikes survive).
- [A robot built before this change sends no diagnostics] → Those charts say "no diagnostics from this robot".
- [No frames arriving (a stalled stream, a disconnect) would freeze "now"] → The tab redraws every 250 ms, and the model is rebuilt when time has moved on, so old samples scroll away.
