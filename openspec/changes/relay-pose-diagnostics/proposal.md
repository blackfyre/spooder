# Proposal

## Why

A planned handheld controller (a TrimUI Smart Pro on NextUI, reaching the robot through the bridge's planned WiFi drive mode) should show a live leg view and the robot's internals, not only telemetry. The robot sends the pose frame and the diagnostics frame on its USB link only, so nothing behind the bridge sees them. The bridge link has room: telemetry (39 B), pose (117 B) and diagnostics (31 B) at 10 Hz each come to about 1,870 B/s, about 17% of the 115,200-baud UART.

## What Changes

- The robot sends the pose frame and the diagnostics frame on the bridge link (UART1) as well as on USB, at the same rates (about ten times per second each), whole or not at all, on the same ticks as on USB: two and four ticks after telemetry, so neither shares a tick with telemetry or with each other.
- Each diagnostics window is reported once, with the same values on both links.
- They are sent whatever the bridge's mode: the robot cannot tell the bridge's modes apart without a new command field, which would need protocol version 4. Both frame types already exist in protocol version 3, so the protocol version and every existing frame stay unchanged, and the boards need not be flashed as a pair.
- The bridge decodes and discards them, in Bluetooth drive and in setup mode; a later change (`add-wifi-drive`) relays them. The bridge's own accepted-frame rate on its link page rises from about 10 to about 30 per second.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `pose-telemetry`: the pose frame is sent on both links, not on USB only.
- `diagnostics-telemetry`: the diagnostics frame is sent on both links, not on USB only.
- `status-display`: the link page's healthy-link figure for the bridge's accepted frames.

## Impact

- `core/robot/robot_tick` (the send schedule: pose and diagnostics on every link) and its host tests; comments in `core/protocol/protocol.h` and `link_receiver.h`.
- `firmware/servo2040/src/link.cpp`: the transmit queue's size guarantee (a pose frame and a calibration report fit together).
- `firmware/bridge/main/link.cpp`: no behaviour change (it already decodes and ignores frame types it does not use); a comment.
- The virtual robot needs no change: its bridge terminal carries whatever the tick sends.
- The `spooder-firmware-shape` skill, `core/CLAUDE.md`, the readme and the changelog; the desktop protocol's frame-type comments.
- Hardware confirmation on the boards.
