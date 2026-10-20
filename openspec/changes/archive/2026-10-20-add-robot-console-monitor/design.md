# Design

## Context

The desktop module (`desktop/`) already has everything the monitor needs below the window: `internal/link` (the stream split into frames and text lines, port discovery by USB ID, and reconnect; `add-desktop-calibrator` design D3), `internal/protocol` (the version 2 decoder) and `internal/session` (names for body modes, gaits, faults and the operator source). The robot writes frames and text from one thread, so they never interleave inside each other (`add-live-joint-calibration` design D4). The link's writer, however, always sends a command frame every 20 ms, which a monitor must never do.

## Goals / Non-Goals

**Goals:**
- One binary without Gio: static and cgo-free on Linux and Windows. On macOS `go.bug.st/serial`'s USB enumeration uses cgo, so the build there keeps cgo (only the system toolchain, no Gio packages).
- Exactly the desktop app's splitting rules, by sharing its code rather than copying it.

**Non-Goals:**
- Controlling the robot over the link (that is the desktop calibrator's job).
- Logging to files, colour or a full-screen view; output is plain lines a shell can redirect or `grep`.
- Bridge console support (`monitor-bridge` stays as it is).

## Decisions

### D1. Package layout

- `desktop/internal/console`: the monitor's logic, testable without a port: flag parsing, the link handler that formats lines, the throttle, the read-only port wrapper and key forwarding.
- `desktop/cmd/spooder-monitor/main.go`: wiring only (real serial ports, stdin, signals).
- It imports `link`, `protocol` and `session`, none of which imports Gio. `CGO_ENABLED=0 go build ./cmd/spooder-monitor` on Linux and for Windows is a gate.
- Alternative: a separate Go module. Rejected: it would need the shared packages moved out of `internal/`.

### D2. Read-only link

- `link.New` accepts a nil `Source`. The connection then runs the reader only: no ticker, no command frames, no final frame on disconnect. Supervising, reconnect and the reader are unchanged, so the split is the desktop app's.
- Defence in depth: the monitor wraps every port it opens. Without `--keys` the wrapper's `Write` refuses every byte; with `--keys` only the key forwarder writes, and only printable ASCII.
- Alternative: a second, read-only copy of the reader in `console`. Rejected: two splitters would drift.

### D3. Ports

- No `--port`: list the ports every retry interval until a Servo 2040 (2e8a:000a) appears, then hand it to the link, which follows that device by its USB serial number after an unplug, as in the desktop app.
- `--port PATH`: the monitor's port list is the system's plus PATH whenever PATH exists but is not listed (a pseudo-terminal or a symlink to one is never listed by USB enumeration). A pseudo-terminal has no modem lines, so opening it with DTR and RTS fails (`TIOCMGET`: inappropriate ioctl). When opening with the modem lines fails as an invalid serial port, the monitor opens again without them. The robot needs DTR to send anything, and a real USB port accepts it, so the first attempt stays.
- Connection lines: `-- connected <port label>`, `-- waiting for <port>: <reason>` (printed when the reason changes, not on every retry), with the desktop app's hints for permission (`dialout`) and a busy port.

### D4. Output

- Every line starts with the local time `HH:MM:SS.mmm` and two spaces.
- Text lines are printed as the link delivers them (non-ASCII as `\xNN`), with `   [n stray 0xA5]` appended when the line held a stray start marker, as the desktop log does.
- Frame lines and status lines are marked so text never looks like them: `> ` for a frame, `! ` for a rejection, `-- ` for connection and key notes.
- Telemetry: `> telemetry #7 Calibrate gait Tri output on 6.01 V 1.35 A faults none link 1000 ok 2 bad source USB`, plus `tof <mm> mm (status <n>)` and `pitch <°> roll <°>` when the sensor bits are set. Several faults are joined with commas.
- Calibration report: `> calibration report #9 request 17 clamped reversed 1`.
- Any other type (a future frame type once the protocol package knows it): `> ` + `protocol.Summary`.
- Rejections are noticed from the link's statistics at each frame or line, so they appear in stream order. A rejected candidate's bytes always end up in the text (its start marker as a stray 0xA5), and the reader flushes a partial line when the connection ends, so every rejection is followed by a line or frame that reports it; a candidate rejected on a line that then stays silent forever is reported when the connection ends or the monitor exits.
- Totals on exit: `-- totals: N telemetry, M calibration reports, K other; R rejected; S stray 0xA5`.

### D5. Keys

- `--keys` starts a reader on standard input. On a Linux or macOS terminal (through `golang.org/x/sys/unix`, already a dependency) it clears `ICANON` and `ECHO` but keeps `ISIG`, so keys arrive at once and Ctrl+C still raises SIGINT; the settings are restored on exit. On Windows, and when standard input is not a terminal, bytes are forwarded as they are read (on a Windows console, a line at a time after Enter). Windows console modes are out of scope.
- Only 0x20–0x7E is forwarded. 0xA5 (the start marker) is outside that range, so no forwarded byte can start a frame, and the robot's decoder discards the bytes as noise in a default build.
- A write is bounded by the link's `WriteTimeout`; a stalled port is closed, which the link reports as a loss.
- `mise run monitor --keys` is the console for a `--serial` build. Without `--keys` the monitor stays observe only even on that build: the keys simply are not sent.

### D6. mise tasks

- `monitor`: builds `build/desktop/spooder-monitor` with `CGO_ENABLED=0` and runs it, with usage flags `--frames`, `--keys`, `--telemetry-every <n>` and `--port <path>` passed through.
- `monitor-raw`: the former `pio device monitor -b 115200`.

## Risks / Trade-offs

- [The pseudo-terminal fallback also hides a real port that rejects modem lines] → Such a port opens without DTR; a robot then sends nothing and the monitor shows a connected but quiet port. No supported adapter does this.
- [Raw terminal mode left behind after a crash] → The settings are restored on every exit path the program controls; `stty sane` recovers otherwise.
- [A user expects the monitor to e-stop] → `--help` says it never sends frames; the e-stop belongs to the gamepad and the desktop app.
