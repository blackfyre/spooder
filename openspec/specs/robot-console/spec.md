# robot-console Specification

## Purpose
A text-mode console for the robot's USB port that shows the robot's debug text cleanly and, on request, a readable decode of its link frames, without ever taking part in the link.

## Requirements

### Requirement: Port choice and reconnect
The monitor SHALL use the first serial port with USB ID 2e8a:000a (the Servo 2040) when no port is named, and SHALL wait for one to appear when none is present. With `--port PATH` it SHALL use that port, which MAY be a pseudo-terminal or any other terminal device. It SHALL report each change of connection state on its own line, and SHALL reconnect to the same device after it is lost, without user action. On Ctrl+C (SIGINT) or SIGTERM it SHALL close the port, restore the terminal and exit with status 0.

#### Scenario: Robot plugged in
- **WHEN** the monitor starts with no `--port` and a Servo 2040 is on USB
- **THEN** it opens that port, says it is connected, and prints the robot's text

#### Scenario: Pseudo-terminal
- **WHEN** the monitor starts with `--port` naming a pseudo-terminal
- **THEN** it opens it and prints the text written to the other side

#### Scenario: Unplug and replug
- **WHEN** the device is lost and later appears again
- **THEN** the monitor reports the loss, waits, and reconnects by itself

#### Scenario: Ctrl+C
- **WHEN** the user presses Ctrl+C
- **THEN** the monitor releases the port, restores the terminal and exits with status 0

### Requirement: Text view
The monitor SHALL print every debug text line from the robot, in stream order, prefixed by the local time of arrival (`HH:MM:SS.mmm`). Link frames SHALL be decoded and SHALL NOT appear in the text. A text byte outside printable ASCII SHALL be shown as `\xNN`, and a line holding any 0xA5 outside a frame SHALL be marked with the count, as the desktop calibrator does (`[n stray 0xA5]`). Text split across reads SHALL be printed as one line.

#### Scenario: Interleaved text and frames
- **WHEN** the robot sends text lines with telemetry frames between and inside them
- **THEN** the monitor prints exactly the text lines, each once and whole, and no frame bytes

#### Scenario: Stray start marker
- **WHEN** a text line contains a byte 0xA5 that does not start a valid frame
- **THEN** that line is printed with `\xA5` in place and marked `[1 stray 0xA5]`

### Requirement: Frame view
With `--frames` the monitor SHALL also print one line per decoded frame, in stream order with the text and timestamped the same way:
- telemetry: sequence number, body mode and gait by name, output on or off, supply voltage and current, faults by name (or none), the robot's valid and discarded frame counts, the operator source from the flags (none, bridge, USB or conflict), and the sensor readings when their availability bits are set;
- calibration report: sequence number, last request id, the result by name (accepted, clamped, rejected, none yet), and the number of reversed joints;
- any other frame type: its type name, sequence number and fields.

The monitor SHALL report each increase of the rejected-candidate count on its own line, before the next text or frame line it prints, or at the latest when the connection ends or the monitor exits. `--telemetry-every N` (N ≥ 1, default 1) SHALL print only every Nth telemetry frame, except that a telemetry frame whose mode, gait, output, faults or source differs from the last one printed SHALL always be printed. Without `--frames` no frame lines SHALL be printed. On exit the monitor SHALL print the totals: frames by type, rejected candidates and stray 0xA5 bytes.

#### Scenario: Telemetry line
- **WHEN** `--frames` is given and the robot sends telemetry in Calibrate, output on, 6.012 V, 1.350 A, no faults, USB in control
- **THEN** one line names Calibrate, the gait, output on, 6.01 V, 1.35 A, no faults, the link counts and source USB

#### Scenario: Throttled telemetry
- **WHEN** `--frames --telemetry-every 10` is given and twenty identical telemetry frames arrive
- **THEN** two telemetry lines are printed (the first and the eleventh)

#### Scenario: Fault appears while throttled
- **WHEN** a telemetry frame between two throttled lines reports a new fault
- **THEN** that frame is printed at once, naming the fault

#### Scenario: Rejected frame
- **WHEN** a frame with a bad CRC or an unknown version arrives
- **THEN** a line reports the rejection with the running count, and the following valid frame is still decoded

### Requirement: Observe only
Without `--keys` the monitor SHALL NOT write any byte to the port. It SHALL therefore never send a command frame, never report a connected controller and never send an emergency stop, and the robot SHALL see it as no operator source. `--help` SHALL state this.

#### Scenario: Watching the robot
- **WHEN** the monitor runs connected to the robot for any length of time without `--keys`
- **THEN** nothing is written to the port

### Requirement: Key forwarding
With `--keys` the monitor SHALL forward the bytes typed on standard input to the port, for the `--serial` build's keyboard commands. When standard input is a terminal on Linux or macOS it SHALL read keys one at a time without echo and without waiting for Enter, while Ctrl+C still ends the monitor; elsewhere it SHALL forward bytes as they are read. It SHALL forward only printable ASCII (0x20–0x7E), so a forwarded byte can never be a frame's start marker, and SHALL drop other bytes. Keys typed while no port is open SHALL be dropped with a note.

#### Scenario: Keyboard build
- **WHEN** the monitor runs with `--keys` against a `--serial` build and the user types `s`
- **THEN** the byte `s` is written to the port, and the robot's reply text is printed

#### Scenario: Non-printable input
- **WHEN** standard input delivers a byte outside printable ASCII
- **THEN** it is not written to the port
