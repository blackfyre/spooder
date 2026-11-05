# Design

## Context

`robot_tick::tick` counts ticks in `telemetryTick` (0 to 4, 20 ms each). Telemetry goes to every link on tick 0; `sendPose` (tick `POSE_TICK` = 2) and `sendDiagnostics` (tick `DIAGNOSTICS_TICK` = 4) go to `Link::Usb` only. `windowDiagnostics` builds the diagnostics for the window just ended and restarts the window as a side effect. Calibration reports go to each link on their own schedule (`link_receiver::reportDue`), on any tick, and their schedule advances only when the frame was sent.

The robot's bridge link (`firmware/servo2040/src/link.cpp`) transmits through a 256-byte whole-frame queue: `queueFrame` copies the frame with the UART interrupt masked, primes the 32-byte hardware FIFO, and the TX interrupt feeds the rest. It never waits; a frame that does not fit is refused whole. At 115,200 baud the line drains about 11.5 bytes per millisecond, about 230 bytes per 20 ms tick.

The bridge's decoder (`core/protocol`) knows every version 3 frame type, so pose and diagnostics frames decode as valid frames; `firmware/bridge/main/link.cpp` acts on telemetry and calibration reports only and ignores the rest, in both modes. Its link page shows the bridge decoder's accepted and rejected rates.

## Goals / Non-Goals

**Goals:**

- Pose and diagnostics on the bridge link at the USB rates, with no protocol change and no bridge behaviour change.
- No measurable cost to the control loop.

**Non-Goals:**

- Relaying the frames past the bridge (`add-wifi-drive`).
- Sending them only in a bridge mode that needs them (needs a protocol version 4 command field).
- Any change to the desktop calibrator, which reads USB only.

## Decisions

### D1. Same ticks, every link

- Tick 2 sends the pose frame, and tick 4 the diagnostics frame, to every link in the existing link order (the bridge, then USB), as `sendTelemetry` does. Each link's frame carries that link's own sequence number, which advances only when the frame was sent.
- The pose and the diagnostics are built once per tick and encoded per link. Building the diagnostics restarts the window, so building them per link would give the second link an empty window; both links get the same values.
- A refusal on one link does not stop the other link's send.
- Alternative rejected: separate bridge ticks (1 and 3). The USB ticks already keep the frames off the telemetry tick, and the bridge queue holds any of them beside a report (D2), so one schedule is simpler and easier to test.

### D2. The bridge transmit queue

- Worst case per tick on UART1: a calibration report (135 B) on the pose tick, 252 B, within the 256-byte queue when it starts the tick empty. The queue does start each pose and diagnostics tick empty in steady state: the most queued in one tick is 174 B (telemetry and a report), which drains in about 15 ms, inside one 20 ms tick.
- A `static_assert` in `link.cpp` states that a calibration report and a pose frame fit together, beside the existing report-and-telemetry one. The queue size stays 256.
- If a frame is still refused (a late tick, an earlier backlog), it is dropped whole: the pose and diagnostics have no retry, and a report waits for its next due tick, as now.

### D3. Loop-time cost

- TX is interrupt-driven, never polled: the tick's extra work is one encode (117 or 31 bytes) and one copy into the queue with the UART interrupt masked, plus up to 32 FIFO writes. The TX interrupt then runs at the FIFO's interrupt threshold, a few times per frame, for a few microseconds each, outside the tick's measured working time unless it preempts it.
- Host tests and the virtual robot cannot measure UART time; there the extra cost is the encode and a function call. The hardware task compares the diagnostics frame's mean and longest tick times on the board with and without the change.

### D4. The bridge

- No bridge code change. `link.cpp` gains a comment that pose and diagnostics frames arrive and are ignored until `add-wifi-drive`. The bridge decoder counts them as accepted; nothing on the bridge treats an accepted frame of another type as an error, and its rejected count is unaffected.
- The link page's bridge rate rises from about 10 to about 30 accepted per second (plus reports in Calibrate); `status-display`'s healthy-link scenario changes accordingly. The bridge's RX buffer (1,024 B) holds about half a second of the new stream.

## Assumptions

- The status-display spec's "about 10 accepted by the bridge" scenario must change to about 30: a third modified capability beyond the two the brief names, needed for the specifications to stay consistent.
- An older bridge (before this change) behaves identically: its decoder already knows types 5 and 6 (they date from protocol version 3), so it counts them as accepted and ignores them.
- Sending pose and diagnostics on the bridge link does not depend on whether a bridge is connected: an unplugged UART1 sends into nothing, as telemetry does now.
- In a `--serial` build there are no links and nothing changes.
- "No regression" in the loop time means the board's diagnostics mean and longest tick times stay within 50 µs of the previous firmware's, standing and walking: an extra encode and queue copy costs a few microseconds, so 50 µs (a quarter of one per cent of the 20 ms period) leaves room for tick-to-tick variation; the margin is a judgement, not a measured figure.

## Risks / Trade-offs

- [The bridge link carries about three times the frames] → about 17% of the line; the bridge's commands to the robot use the other direction and are unaffected.
- [A pose frame dropped on the bridge link while a report fills the queue] → harmless: the next comes 100 ms later.

## Follow-ups

- `add-wifi-drive`: relay pose and diagnostics to the handheld controller.
- If a later protocol version adds a bridge-mode field to the command, the robot could stop sending them to a bridge that does not relay them.
