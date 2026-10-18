# Design

## Context

- **Robot calibration today:** `kinematics::LEG_OFFSETS` holds per-leg (coxa, femur, tibia) offsets inherited from upstream, of about 75–93° for coxa and femur. `kinematics::angleToMicroseconds` maps 0–180° to a fixed 500–2500 µs.
- **The link** (`add-gamepad-bridge`): `core/protocol` version 1 (command and telemetry frames), `link_receiver` on the robot, and UART1 to the bridge.
- **USB on the robot:** USB stdio carries debug text (and the serial keys in `--serial` builds), with `PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000`, so a host that isn't reading never stalls the loop.

## Goals / Non-Goals

**Goals:**
- Everything that decides what a joint does (table, guards, rate limit, arbitration) is on the robot, hardware-free and host-tested. Front-ends are thin clients that request changes and show reports.
- One wire contract and one export format, each pinned by golden files, so a second implementation in another language can be verified mechanically.

**Non-Goals:**
- Persisting calibration on the robot.
- USB control in `--serial` builds: there the USB port carries the serial keys, and the USB link is disabled.

## Decisions

### D1. Layout

```
core/movement/joint_calibration.{h,cpp}   table type, clamping, rate-limited apply, angle -> pulse
core/movement/calibration_defaults.cpp    compiled defaults (the export target, fixed format)
core/protocol/                            version 2: calibration set/report, source-conflict flag
test/golden/calibration_defaults.cpp      golden copy, checked by every exporter
test/golden/protocol_v2.txt               golden frames: hex bytes + decoded fields per line
firmware/servo2040/src/usb_link.{h,cpp}   frames over USB stdio, sharing the port with printf
```

`kinematics::LEG_OFFSETS` is removed. Its values move into `calibration_defaults.cpp`, which keeps them as the initial defaults.

### D2. The calibration table

- `joint_calibration::Joint { float offsetDeg; uint16_t pulse0Us, pulse180Us; bool reversed; }`, 18 per table.
- The robot keeps two tables: **requested** (the latest accepted values, after clamping) and **applied** (what the IK and servo output use).
- `joint_calibration::step(applied, requested, dtS)` moves each applied value towards the requested one, by at most 60°/s for offsets and 667 µs/s for pulse end points (60°/s at the default 2000 µs per 180°). It runs every tick before `servo_output::apply`.
- **IK:** `solveLeg` takes the leg's three applied offsets. **Pulse:** `joint_calibration::toPulse(joint, angle)` clamps, mirrors if reversed, and interpolates between the end points. It replaces `angleToMicroseconds` in `servo_output`.
- **Guards** (in `link_receiver`, with the body mode and `allowOutput` passed in):
  - reject unless the body is in Calibrate and output is allowed;
  - clamp the offset to its default ±30° and each pulse to 400–2600 µs;
  - keep the end points at least 1000 µs apart, moving the requested 180° pulse away from the 0° pulse if needed;
  - treat a repeated request id on the same link as a retransmission: report again, change nothing.

### D3. Protocol version 2

`protocol::VERSION` becomes 2. A version-1 peer is rejected by the decoder, so mismatched firmwares degrade to "link lost" and a sleeping robot.

- **Type 3, calibration set** (front-end to robot), 9-byte payload: request id u8, joint u8 (0–17), offset centidegrees i16, pulse at 0° µs u16, pulse at 180° µs u16, flags u8 (bit 0 reversed).
- **Type 4, calibration report** (robot to front-ends), 128-byte payload: last request id u8, result u8 (0 accepted, 1 clamped, 2 rejected, 3 none yet), then 18 × (offset i16, pulse 0° u16, pulse 180° u16, flags u8).
- **Telemetry** keeps its size. Reserved byte 21 becomes flags: bit 0 source conflict, bit 1 USB in control, bit 2 bridge in control.
- `MAX_FRAME_SIZE` becomes 135. One report takes about 11.7 ms at 115200 baud on UART1; USB is far faster.
- **Rates:** a report after each processed set, and at 1 Hz in Calibrate, capped at 5 Hz per link.
- **Golden vectors:** a host test encodes a fixed set of frames of every type (including edge values and wrap-around) and compares them with `test/golden/protocol_v2.txt`. Regenerating the file is an explicit step, done when the protocol changes; the Go tests read the same file.

### D4. USB link alongside debug text

- **RX:** `usb_link::poll` drains up to 256 bytes per tick with `stdio_getchar_timeout_us(0)` into its own `protocol::Decoder`, and calls `idle()` after a tick without bytes, like UART1.
- **TX:** frames are written with `stdio_put_string(frame, len, false, false)` (no newline, no CR translation) from the main loop, the same thread that prints. Text and frames therefore never interleave inside each other.
- **Debug text** must stay printable ASCII. A review rule and a host-side check, the desktop app flagging any 0xA5 outside a frame, keep it so.
- **Dropped bytes:** with the 1 ms stdout timeout, a host that isn't reading loses whole writes. The decoder on the host resynchronises; nothing on the robot waits.
- **`--serial` builds:** `serial_commands` owns USB input, and `usb_link` is not compiled. The two command sources are a build choice, never mixed.

### D5. Source arbitration (`link_receiver`)

- One `Receiver` per link (bridge, USB). Each tick, `arbitrate(bridge, usb)` returns the input for the tick and the telemetry flags:
  - both connected and fresh → Sleep with the conflict flag;
  - else bridge connected and fresh → bridge;
  - else USB connected and fresh → USB;
  - else no source → Sleep.
- `requestSeq` is per source. A change of source counts as a new request only through that source's own counter, so a source switch never resumes motion after a fault by itself.
- E-stop: the OR of both links' flags. Fault clear: only from the source in control.

### D6. Compiled-defaults format

- `calibration_defaults.cpp`: a fixed header comment naming the format, then `const joint_calibration::Joint DEFAULTS[18] = {`, then one line per joint in output order: `{offset, pulse0, pulse180, reversed}, // leg N coxa|femur|tibia`, with the offset printed to two decimals. Then `};`.
- The golden copy in `test/golden/` equals the committed file (a host test checks this), so "export of the defaults reproduces the file" is testable by any exporter.

## Risks / Trade-offs

- [A calibration change moves a joint while someone holds the leg] → Calibrate only, 60°/s, ±30° of the defaults; the robot on a stand, and the e-stop.
- [Pulse end points that drive a servo into its stop] → The 400–2600 µs hard limit; the current reading shows a stall, and protection trips on overcurrent.
- [Debug text containing 0xA5 confuses the host decoder] → ASCII-only text; the CRC rejects any false frame anyway.
- [A forgotten USB host keeps control after the gamepad connects] → Both connected is a conflict that sleeps the robot. It is visible on both front-ends and the LEDs, so the operator releases one of them.
- [Mismatched firmware versions] → The version byte fails safe (link lost, robot asleep).

## Migration Plan

1. Core first: table, guards, arbitration, protocol v2 and the golden files, all host-tested. With the defaults, behaviour is unchanged (the existing IK and body tests pass).
2. Robot: runtime table, reports, TX queue, USB link.
3. Bridge: protocol v2 (it decodes reports but sends no calibration until `add-bridge-setup-portal`).
4. Flash both boards together. **Rollback:** both from the previous commit.
