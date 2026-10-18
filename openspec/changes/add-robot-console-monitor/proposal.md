# Proposal

## Why

In the default robot build the USB port carries protocol-v2 binary frames (telemetry every 100 ms, calibration reports in Calibrate) between the ASCII debug lines. `mise run monitor` is a raw `pio device monitor`, so the console fills with junk between the text lines and the robot's state in the frames cannot be read at all. The desktop calibrator decodes the stream, but it needs Gio, cgo and a window. Bring-up and remote sessions need a plain terminal console.

## What Changes

- **A terminal monitor, `spooder-monitor`, in the desktop module** (`desktop/cmd/spooder-monitor`). It reuses the module's stream splitter, frame decoder and names, and needs no Gio: on Linux and Windows it builds without cgo or any system packages (macOS needs cgo for USB port enumeration, as the desktop app does).
- **Text by default:** the robot's debug lines with timestamps; frames are decoded and kept out of the text, and a stray 0xA5 is marked as in the desktop app.
- **`--frames`:** a one-line decode per frame (telemetry by name, calibration reports by summary and result), throttled by `--telemetry-every N`, with rejected frames counted.
- **Port:** the Servo 2040 is found by its USB ID; `--port PATH` names any port, including a pseudo-terminal. It reconnects after an unplug and exits cleanly on Ctrl+C.
- **Observe only:** it never sends a frame, so it never claims control and never sends an e-stop. The one exception is the opt-in `--keys`, which forwards typed printable ASCII to the robot for the `--serial` build's keyboard commands. Printable ASCII can never form a frame.
- **mise:** `mise run monitor` runs the new monitor and takes the same flags (`--frames`, `--keys`, `--telemetry-every`, `--port`). The old raw console stays available as `mise run monitor-raw`.

## Capabilities

### New Capabilities

- `robot-console`: The text-mode USB console for the robot: port choice and reconnect, the text and frame views, the observe-only guarantee and key forwarding.

### Modified Capabilities

None.

## Impact

- **New code:** `desktop/cmd/spooder-monitor` and `desktop/internal/console`.
- **Changed code:** `desktop/internal/link` gains a read-only mode (no source, so no writer). The desktop calibrator's behaviour does not change.
- **Tasks and docs:** `mise.toml` (`monitor`, `monitor-raw`), `CLAUDE.md` and the `spooder-hardware-check` skill (monitor mentions).
- **No firmware or protocol change.**
