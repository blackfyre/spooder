# Proposal

## Why

When protection stops the body after a fault, the robot keeps the body's last mode and keeps reporting it in telemetry. After the fault clears, while the robot waits for a new mode request, the bridge screen and the desktop calibrator show, for example, `WALK` with servo output off and no fault. The front-ends cannot tell a stopped body from a running one, and the operator is misled about what the robot is doing.

## What Changes

- The robot keeps an explicit run state for the body: not started (from power-up until the first start), running, or stopped (after a fault stopped it, until it restarts). Protection's supervisor owns it, in place of today's running/not-running boolean; the LED state, the gesture reported in telemetry and the console status line derive from it.
- Telemetry gains a flag, body stopped (bit 6 of the telemetry flags), set in every telemetry frame while the body is not running: before the first start and after a fault stop. It clears on the first telemetry frame after the body (re)starts. The frame layout does not change.
- **BREAKING**: the link protocol version goes from 3 to 4, so that a front-end that does not know the flag cannot show a stale mode as live. A robot and a bridge (or the desktop calibrator, or the handheld) of different versions discard each other's frames: the link looks lost and the robot sleeps. Both boards are flashed together, and the desktop calibrator and the handheld are rebuilt.
- The golden frame file becomes `test/golden/protocol_v4.txt`, with a telemetry vector carrying the new flag.
- Wherever the body mode is shown, a stopped body shows as stopped instead of its last mode: the bridge's drive page header and its setup screen, the desktop calibrator's header and its charts' mode lane, the console's decoded telemetry line, the WiFi drive probe's telemetry summary, and the handheld's screen.
- The handheld treats a telemetry frame reporting the body stopped as confirming a leave, as it already treats Sleep or servo output off.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `controller-link`: telemetry carries the body-stopped flag, and the protocol version is 4.
- `status-display`: a stopped body is shown as stopped, not by its last mode.
- `desktop-calibrator`: a stopped body is shown as stopped, not by its last mode.
- `handheld-controller` (introduced by the unarchived `add-handheld-controller`): a stopped body is shown as stopped and confirms a leave.

## Impact

- `core/protection/supervisor` (the run state) and `core/robot/robot_tick` (the flag, the LED input, the gesture guard, the status line); `core/protocol/protocol.h` (the flag and `VERSION`); `core/status/status_screen` (the header and the setup screen).
- The virtual robot's status line (`tools/virtual-robot`).
- `desktop/internal/protocol` (the version and the flag), `desktop/internal/session`, `desktop/internal/ui` (header, charts), `desktop/internal/console`, `desktop/internal/wifiprobe`; `desktop/handheld/internal/app` (the screen and the leave confirmation).
- The golden frame file and every test and document that names it; host tests in `test/test_supervisor`, `test/test_robot_tick`, `test/test_bench_sim`, `test/test_led_pattern`, `test/test_status_screen` and `test/test_protocol_golden`.
- Both boards must be flashed together; hardware confirmation on the boards.
