# Design

## Context

- **Robot side** (`add-live-joint-calibration`): protocol version 2 frames on the USB port, interleaved with ASCII debug text; source arbitration; calibration guards and reports; golden files `test/golden/protocol_v2.txt` and `test/golden/calibration_defaults.cpp`.
- **Reference app:** `~/Projects/blackfyre/bopen`, a Gio application:
  - `gioui.org v0.10.3` with `gioui.org/x`, Go 1.26 with toolchain 1.27.1;
  - `internal/ui` (theme, style, window, form patterns, models tested apart from drawing) and `internal/appearance` (system light or dark mode: D-Bus on Linux, the registry on Windows);
  - goreleaser with Linux cgo builds and Windows builds without cgo.
- **Design source:** `~/Projects/blackfyre/legendary-winner` (blackfyre.ninja), whose UI is maintained in the Claude Design project "Blackfyre" (read only).

## Goals / Non-Goals

**Goals:**
- Every guard is on the robot; the app is a thin, honest client that shows reports, never its own assumptions.
- The protocol and export code are tested against the same golden files as the C++.
- UI models are testable without a window.

**Non-Goals:**
- Walking the robot from the app.
- Release packaging, installers and macOS (the scaffolding from `bopen` makes this a later, separate step).
- Sharing UI code with the setup portal.

## Decisions

### D1. Layout

```
desktop/                          Go module github.com/blackfyre/spooder/desktop
  cmd/spooder-calibrator/         main: window, wiring
  internal/protocol/              frames, CRC-16/CCITT-FALSE, incremental decoder (v2)
  internal/link/                  serial port I/O, frame/text split, 20 ms command ticker, reconnect
  internal/session/               state: telemetry, control toggle, e-stop, mode counter
  internal/calib/                 joint table model, pending edits, resend, confirmation
  internal/export/                calibration_defaults.cpp writer, checkout check, diff
  internal/appearance/            copied from bopen, unchanged in behaviour
  internal/ui/                    Blackfyre theme, widgets, screens
  internal/ui/fonts/              Archivo SemiBold, JetBrains Mono (+ OFL licences), go:embed
```

A separate module keeps Gio and cgo out of anything the firmware build or the host tests touch.

### D2. Dependencies and toolchain

- Go pinned in `mise.toml` (1.27.1, matching `bopen`'s toolchain); `gioui.org v0.10.3`; `go.bug.st/serial` v1.8.0, with no cgo on Linux or Windows. `gioui.org/x` is not needed: nothing in the app uses it.
- Linux build needs cgo plus the Wayland, X11 and EGL development packages. Fedora: `wayland-devel libxkbcommon-devel libxkbcommon-x11-devel libX11-devel libXcursor-devel libXfixes-devel mesa-libEGL-devel vulkan-headers`. These are the providers of the `pkg-config` names in Gio's cgo directives (`wayland-client wayland-cursor wayland-egl egl xkbcommon xkbcommon-x11 x11 x11-xcb xcursor xfixes`) and of `<vulkan/vulkan.h>`; GLES is loaded at run time, so `mesa-libGLES-devel` is not needed. They are listed in the hardware-check skill, and `desktop/scripts/check-deps.sh` (the `desktop-deps` task, run before `desktop-build` and `desktop-test`) checks each entry and prints the `dnf install` line for whatever is missing.
- mise tasks: `desktop-build`, `desktop-run`, `desktop-test` (`go test ./...` in `desktop/`). `desktop-test` is a verification gate next to `mise run test`.

### D3. Serial I/O and the stream split

- One reader goroutine owns the port. Bytes go to the decoder; a byte that is not part of a frame candidate goes to the text accumulator, which emits lines on `\n`. A 0xA5 that turns out not to start a valid frame is flushed back to the text as `\xA5`, so nothing is lost silently (and the robot's ASCII-only rule is checked in practice).
- One writer goroutine sends a command frame every 20 ms, and at most one calibration set per period after it, the same rule as the bridge.
- Port discovery: `go.bug.st/serial/enumerator` for USB IDs; the Servo 2040 is VID 2e8a, PID 000a. On loss the link retries every 500 ms for the same serial number.
- **Permissions:** on Linux the port needs the `dialout` group; the app shows this hint when opening fails with a permission error.
- UI updates go through Gio's `Invalidate` from a single state owner (`session`), with snapshots copied under a mutex. No UI code touches the port.

### D4. Control semantics

- **Observe** (default): `FLAG_CONTROLLER_CONNECTED` clear, mode Sleep, zero input. The robot therefore ignores the app for movement, but accepts its e-stop.
- **Control:** the flag is set, and mode buttons advance the mode-request counter. Releasing control clears the flag.
- **Clear fault:** advances the clear counter; enabled only in control.
- **Window close:** the writer sends one last frame without the controller flag, then closes the port. Even without it, the robot's 1 s stale rule applies.

### D5. Calibration model (`internal/calib`)

- The same rules as the bridge's `calibration_sync`: the latest value per joint, the oldest pending sent first, a request id per send, confirmation by the report's request id, a resend after 200 ms with the same id, and the reported value replacing the shown one, including clamps and rejections.
  - **One change in flight.** The robot recognises only its last request id per link as a retransmission, so a resend of an older change after a newer one was processed would be processed again (and, if output was meanwhile disallowed, reported as rejected although applied). Until the sent change is confirmed, only it is resent; other edits wait, keeping their latest value. A newer value for the joint in flight replaces it and is sent with a new id.
  - **Request ids** start after the `lastRequestId` of the first report on a connection (the robot keeps it across a desktop restart), and a new id never equals the latest reported one. A report with result "none yet" confirms nothing.
  - Unconfirmed edits are dropped when control is released or the link is lost; sets are sent only while in control. On a new connection the table is cleared until the robot reports again (it reports at 1 Hz in Calibrate and after each processed set).
- Keyboard: arrow keys select a joint and field; `+`/`-` nudge fine, Shift+`+`/`-` coarse (0.5° and 5°; 10 µs and 50 µs); `R` toggles direction.

### D6. Export

- The writer produces the format of `add-live-joint-calibration` D6. Its test exports the defaults and compares them with `test/golden/calibration_defaults.cpp` byte for byte.
- **Checkout check:** the chosen directory must contain `core/movement/calibration_defaults.cpp` and `openspec/`. The file is written atomically (temporary file plus rename), after a line diff is shown and confirmed. The last chosen directory is remembered in the user config directory.
  - Neither the file nor `core/` and `core/movement/` may be symbolic links, so the write cannot land outside the checkout.
  - Before writing, the file is read again; if it no longer holds what the diff was made from, nothing is written and the export must be repeated. The temporary file is removed on any failure, and the file keeps its permissions.

### D7. Look

- **Tokens** (from blackfyre.ninja):
  - light: paper `#e6e4df`, ink `#1b1b19`, muted `#5e5c56`;
  - dark: `#1b1b19` background, `#121211` wells, `#2c2c29` panels, `#f3f2ee` text, `#3a3935` selection;
  - accent `#e2551d` (pressed `#a93a0f`, as text `#a3390c`);
  - status: green `#9ece6a`, yellow `#e0af68`, blue `#7aa2f7`.
- **Type:** Archivo SemiBold for text; JetBrains Mono, uppercase with wide letter-spacing, for labels, buttons and every numeric value. Gio's text layout has no letter-spacing, so labels are laid out a glyph at a time with 0.08 em between them; values keep their case.
- **Text on the accent** is ink, not white: white on `#e2551d` is 3.8:1, ink 4.6:1, and the labels are small. A theme test checks every text and fill pair the UI draws in both palettes.
- **Buttons:** 7 px radius, flat fill, a 2 px darker bottom shadow that flattens on press (a small custom Gio widget).
- **Screens:** a header (port, connection, control toggle, e-stop), a telemetry strip, the joint editor as a 6 × 3 grid of leg/joint cards, and the log and export panes.
  - **Keyboard:** the window takes Tab, so keyboard focus never moves onto a button, which would take Space for itself: Space is always the e-stop. For the same reason there is no text field: the export's checkout is chosen from the one given with `--checkout`, the remembered one, the working directory and the checkout the binary was built in. Before building, check the Claude Design project for anything newer than the site's CSS; the project is only read, never written.

## Risks / Trade-offs

- [The Go protocol drifts from the C++] → Both test against `test/golden/protocol_v2.txt`; a version bump breaks both suites.
- [An app left in control blocks the gamepad (conflict)] → Control is an explicit toggle, shown prominently; the conflict sleeps the robot and is shown on both front-ends.
- [Gio's Linux build dependencies are missing] → The mise task names the missing packages.
- [Writing into the wrong directory] → The checkout check, a diff, confirmation and an atomic write of one file only.

## Migration Plan

1. After `add-live-joint-calibration`: protocol, link and models with their tests.
2. The UI, first observe-only (log and telemetry), then control and e-stop, then the editor and export.
3. Bench test with the Servo 2040 on USB, one servo, then a leg.
4. **Rollback:** the app is separate; removing it changes nothing on either board.
