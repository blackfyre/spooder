# Proposal

## Why

The desktop calibrator, and later the bridge path, can only be exercised end to end on the robot itself. The loop that ties input, protection, supervision, calibration, movement and reporting together lives in the firmware's `main.cpp`, so it has never been tested on the host either: e-stop latency, a stale command never starting the body and report scheduling were checked only on the board. A host program that runs the robot's real logic and speaks the link protocol over a pseudo-terminal closes both gaps.

## What Changes

- **The control tick moves into `core/`.** The hardware-free part of the 20 ms loop (link decoding, arbitration, protection, the supervisor, calibration processing and stepping, the body, LED state, telemetry and calibration reports, debug messages) becomes one function shared by the firmware and the virtual robot. The firmware's `main.cpp` keeps only the adapters and the timing. Behaviour is unchanged: same order, same rates, same text. Host tests cover the loop's behaviour that was previously checked only on hardware.
- **A virtual robot under `tools/virtual-robot/`**, a native C++17 program built with CMake and Ninja (`mise run virtual-robot`). It opens a pseudo-terminal as the robot's USB port, optionally a second one as the bridge link, and runs the shared tick at 20 ms in real time, faster, or one step at a time. Debug text and frames share the USB terminal as on the board. A simulated supply (voltage, current growing with servo motion) feeds protection, and commands on standard input change the supply, signal an e-stop from a simulated bridge, or pull and replug the USB cable.
- **The desktop calibrator takes `--port PATH`**, listing that port (a pseudo-terminal or a stable link to one) and preselecting it.
- **End-to-end Go tests** in `desktop/` run the virtual robot and drive it through the calibrator's real link and session: observe, control, Calibrate, a calibration change and its rate limit, the e-stop, releasing control, and an overvoltage trip. They skip when the virtual robot is not built; a mise task builds it and runs them.

## Capabilities

### New Capabilities

- `virtual-robot`: The host program: what it runs, its terminals and their stream, timing modes, the simulated supply and current, its control commands and observation output.

### Modified Capabilities

- `desktop-calibrator` (in flight in `add-desktop-calibrator`, not yet in `openspec/specs/`): a requirement is added for an explicit port given on the command line.

The locomotion, leg-actuation and power-protection requirements do not change: the tick extraction is a refactor of where the loop's code lives.

## Impact

- **Firmware:** `firmware/servo2040/src/main.cpp`, `link.{h,cpp}` and `usb_link.{h,cpp}` shrink to adapters (bytes in, frames out); decoders and receivers move into the shared tick's state. `spooder_core` gains `core/robot/robot_tick.cpp`.
- **Host tests:** a new `test/test_robot_tick/`; `platformio.ini` builds the new core file.
- **New code:** `tools/virtual-robot/` (POSIX pseudo-terminals; Linux, and other POSIX systems untested). It reuses the firmware's `config.h` and a power-limits header.
- **Desktop:** `cmd/spooder-calibrator` (`--port`), `internal/link` (an explicit port: listed, opened without modem control lines), `internal/ui` (preselects it), and an e2e test package.
- **Not affected:** the bridge firmware (it compiles `core/protocol` but not the tick), the protocol and its golden files.
