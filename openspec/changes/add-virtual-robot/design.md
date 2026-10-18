# Design

## Context

- `firmware/servo2040/src/main.cpp` runs the 20 ms loop: input (`link::poll`, `usb_link::poll`, `link_receiver::arbitrate`, or `serial_commands::poll`), `power_monitor::read`, `protection::step`, `servo_output::setInhibited`, the stale rule, `protection::supervise`, stop and start, `processCalibration` on both links, `joint_calibration::step`, the body step with the applied offsets, `servo_output::apply`, `status_leds::show`, telemetry every fifth tick and calibration reports through `ReportSchedule`, then the once-per-second status line. Each step is core logic except the reads and writes.
- `link.cpp` and `usb_link.cpp` each own a `protocol::Decoder`, a `link_receiver::Receiver` and a transmit sequence number, and decode inside `poll()`.
- Two parallel changes touch nearby code: a USB-only pose frame (joint angles) in `core/protocol`, the firmware and the desktop leg view, and a text console under `desktop/cmd`.

## Goals / Non-Goals

**Goals:**
- One tick, compiled into the firmware, the host tests and the virtual robot.
- The firmware's behaviour unchanged: the same calls in the same order at the same rates, the same text.
- End-to-end tests of the calibrator against that tick, without hardware.

**Non-Goals:**
- Simulating servo dynamics, the LEDs' pixels, the ADC or UART timing.
- Running the bridge firmware on the host (the bridge terminal lets a later change attach one).
- Windows or macOS support for the virtual robot.

## Decisions

### D1. `core/robot/robot_tick`: state, inputs, and an output port

```
RobotState  body, protection, supervisor, requested/applied tables,
            per link: Decoder + Receiver + txSeq + ReportSchedule,
            Arbiter, last source, telemetry counter, status timer, last tick time
Inputs      bytes received on each link since the last tick (or the serial keys'
            input in --serial builds), the power reading
tick(state, inputs, nowMs, Hardware &hw)
```

`Hardware` is a small interface the adapter implements: `setInhibited`, `stopServos`, `applyServos(body, applied)`, `servosOn`, `showLeds`, `sendFrame(link, bytes, n) -> bool` and `log(text)`. The tick calls it at exactly the points where `main.cpp` called `servo_output`, `status_leds`, the link senders and `printf`.

- *Why an interface, not a pure `inputs -> outputs` function:* two results feed back within the tick. Telemetry carries `servo_output::outputsOn()` after `apply()`, which depends on the adapter's own state (pins forced low for 25 ms after a switch-off); and a frame's sequence number and a report's schedule advance only if the frame was actually queued, which only the adapter knows at the moment of sending. A two-phase split or a modelled output state would either reorder these calls or duplicate the adapter's rules in core; the interface keeps the order exact and the adapters dumb.
- *Decoding moves into the tick* (the decoders live in `RobotState`): it is hardware-free, and the virtual robot then decodes exactly as the board. The adapters' `poll()` becomes `read(buf, max)`: the UART ring drained as before (at most its 256 bytes), USB at most `MAX_BYTES_PER_POLL`. The tick pushes the bytes one by one with `handleFrames` after each, and calls `idle()` when none arrived, as `poll()` did.
- *Ordering:* the adapter reads the link bytes, then the power reading, then calls `tick`; previously decoding ran between the two reads. Decoding has no effect on the reading, so only the reading's sampling moment shifts, by the decode time (microseconds). An e-stop in a frame that has arrived still acts in the same tick.
- *Text:* the tick formats each message with `snprintf` into a line buffer and hands it to `log`; the firmware prints it with `printf("%s", ...)`, so the bytes are the same.
- *--serial builds:* `Inputs` carries the serial keys' input instead of link bytes; the tick then skips decoding, calibration processing, telemetry and reports, exactly as the `#ifdef`s did.
- *Constants:* `LOOP_PERIOD_MS`, `STATUS_PERIOD_MS` (was `REPORT_PERIOD_MS`) and `TELEMETRY_EVERY_TICKS` move into the tick header; the power limits are passed to `init`.
- *Alternative considered:* keeping decoders in the adapters and passing decoded `Input`s. Calibration processing and reports then need the receivers inside the tick anyway, so the adapters would have to hand out their internals; moving the endpoint whole is simpler.

### D2. The pose frame hook

The pose frame (USB only, joint angles) is being added in parallel. The tick marks the place where it belongs, after the servos are applied and next to telemetry, with a `HOOK(pose-frame)` comment; it is sent through `hw.sendFrame(Link::Usb, ...)` so the firmware and the virtual robot get it together. Integration happens after both changes merge.

### D3. Virtual robot program

```
tools/virtual-robot/
  CMakeLists.txt        core sources + firmware config.h, -Wall -Wextra, C++17
  src/main.cpp          options, control commands, the timing loop
  src/pty.{h,cpp}       pseudo-terminal endpoint: open, raw mode, connected?, writes
  src/sim.{h,cpp}       simulated servo outputs, supply and current, simulated bridge
```

- **Terminals:** `posix_openpt`; the slave is opened once to set raw mode (no echo, no CR/LF translation) and closed again. A master then reports `POLLHUP` while no program has the slave open; that is "no USB host", and output is dropped, as `stdio_usb_connected()` drops it. Writes are non-blocking; bytes the kernel does not take are held and sent first next time, so a frame is never cut. A frame is dropped whole if the held backlog exceeds 4 KB (the board's equivalent is the 256-byte CDC buffer; the host's is larger but bounded). Text is dropped the same way.
- **Stable path:** `--link PATH` keeps a symbolic link to the current USB terminal (replaced atomically); `unplug` closes the master and removes the link, `plug` opens a new terminal and links it.
- **Bridge:** `--bridge-pty [PATH]` opens a second terminal for the bridge link. Without it, the simulated bridge (`bridge off|idle|gamepad|estop`) encodes command frames every tick into the bridge link's bytes; `idle` reports no gamepad, `gamepad` a connected one requesting Stand, `estop` sets the e-stop flag.
- **Servo outputs:** the simulated `servo_output` follows the firmware's rules that matter to the tick: inhibited or not enabled means off; pulses are computed through the applied table for legs the body has set; on is reported after apply. The 25 ms pin release is not simulated (no pins).
- **Current:** `I = load × (n_on × 0.12 A + Σ|Δpulse| per tick × 0.004 A/µs)`, zero with output off; `current A` fixes the reading, `current auto` restores the simulation. Voltage defaults to 6.0 V.
- **Timing:** robot time starts at 0 and advances exactly 20 ms per tick. Real-time mode paces with `clock_nanosleep` on an absolute deadline (an overrun restarts the period, as on the board); `--speed N` divides the period; `--stepped` ticks only on `step N`.
- **Observation:** standard output carries answers (`ok …`, `error …`) and events with the robot time: `usb <path>`, `applied t=<ms> joint=<j> offset=<deg> pulse0=<us> pulse180=<us>`, `output t=<ms> on|off`, `tick t=<ms>` after a step. `--echo` also copies the debug text to standard error.
- **Limits:** compiled from a power-limits header (`SPOODER_POWER_LIMITS`, default `firmware/servo2040/src/power_limits.h`) through the firmware's `config.h`, with its static checks.

### D4. Desktop: `--port`

`link.WithPort(sys, path)` wraps a `System`: `List` adds `PortInfo{Name: path}` when the path exists and the enumeration lacks it. `Serial.Open` falls back to opening without `InitialStatusBits` when setting them fails for want of modem lines, as on a pseudo-terminal (go.bug.st/serial's `TIOCMGET` fails on one). `ui.Env.Port` preselects it ahead of a Servo 2040, also when it appears later, unless the operator has picked a port. `SameDevice` already compares names for devices without a serial number, so reconnection follows the path.

### D5. End-to-end tests

`desktop/e2e` (Go, test-only package) with `internal/virtualrobot` (start, commands, output lines): builds nothing itself; it looks for `build/virtual-robot/virtual-robot` (or `SPOODER_VIRTUAL_ROBOT`) and skips without it. The calibrator side lists only the robot's terminal, never the computer's ports. Each test starts the robot with a fresh `--link` in a temporary directory and real-time pacing (the protocol's 200 ms link-gap and 1 s staleness rules are wall-clock rules for the front-end, so faster-than-real-time ticks would reset counter baselines between commands), then drives a real `session.Session` through a real `link.Link`, reading the robot's standard output for applied values and output state. `mise run e2e` builds the robot and runs them.

### D6. Screenshots against the live robot

A test in `internal/ui` behind `SPOODER_SCREENSHOTS` and the virtual robot binary: the window's session is driven by a real link to the robot's terminal, and frames are rendered headless in the states observing, in control in Calibrate with a pending change, and e-stop, light and dark.

## Risks / Trade-offs

- [The extraction changes firmware behaviour subtly] → The tick is a line-by-line move of `main.cpp`'s body; tests pin the loop's rules; an independent review compares old and new loops call by call; the hardware check repeats the bring-up items for e-stop, faults and calibration.
- [Virtual-time ticks hide real overruns] → The virtual robot is for logic and protocol, not timing; the board's timing stays a hardware check.
- [A pseudo-terminal is not a CDC device: no DTR, a larger buffer] → Host presence comes from the slave being open; whole-frame writes are kept by the backlog rule.
- [The parallel pose-frame change also edits `main.cpp` and `usb_link`] → The hook marks the place; the coordinator moves its sending into the tick when merging.

## Migration Plan

1. Extract the tick with its tests; rebuild both firmware variants.
2. Add the virtual robot, then `--port`, the e2e tests and the screenshots.
3. **Rollback:** revert the extraction commit; the virtual robot and the desktop option are additions.
