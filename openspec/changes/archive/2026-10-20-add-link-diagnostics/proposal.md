# Proposal

## Why

The robot sends each link only that link's own frame counts. From the USB port (the desktop calibrator, `mise run monitor`) there is no way to tell whether the bridge's frames arrive: "source none" means both "bridge with no gamepad" and "bridge link dead". Bring-up showed this gap: confirming the link needed the TTGO's screen or a paired gamepad.

## What Changes

- Every telemetry frame, on both links, also carries the bridge link's state: whether bridge frames arrive (a valid command frame, the bridge's 20 ms heartbeat, within the link-gap time), whether the bridge reports a connected gamepad, and the bridge link's valid and discarded frame counts.
- The fields use telemetry bytes that are reserved and zero in protocol version 3, so the version stays 3: older front-ends ignore them, and the robot's older firmware simply reports them as zero.
- The desktop calibrator shows a "Bridge" cell (up or down, gamepad, counts), and `mise run monitor --frames` prints the bridge state on each telemetry line.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `pose-telemetry`: telemetry gains the bridge link state (the USB host's view of the robot).
- `desktop-calibrator`: the Bridge cell.

## Impact

- `core/protocol/protocol.{h,cpp}` (telemetry bytes 23–26, flag bits 3 and 4), `core/robot/robot_tick` (filling them), golden vectors regenerated.
- `desktop/internal/protocol`, `internal/console`, `internal/session`, `internal/ui`.
- No bridge change. Flash the robot to get the fields; mixing with older robot firmware is harmless.
