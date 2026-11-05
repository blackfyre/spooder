# Proposal

## Why

Spooder should be drivable from a TrimUI Smart Pro handheld running NextUI, with its 1280×720 screen as feedback (the handheld app is a later change, `add-handheld-controller`). The handheld has WiFi but cannot act as a Bluetooth gamepad, and the robot has no radio, so the bridge must carry the handheld's commands. A feasibility spike on the device measured 1,000 of 1,000 pings answered by the bridge's setup-mode access point at 50 Hz, 5.1 ms on average and 34 ms at worst, with the handheld's WiFi power saving off.

## What Changes

- **WiFi drive, a new bridge mode.** The bridge runs a WPA2 access point instead of Bluetooth and accepts one handheld at a time. The handheld sends link-protocol command frames (same framing, version 3, CRC) in UDP datagrams; the bridge takes the desired state from them and builds its own command frames to the robot every 20 ms, as in Bluetooth drive. Every frame the robot sends (telemetry now; pose and diagnostics once `relay-pose-diagnostics` lands) goes to the handheld over UDP unchanged.
- **The bridge owns safety.** It reports no controller, so the robot sleeps, when no valid handheld frame has arrived for 150 ms. The GPIO35 emergency-stop button works as in every mode, ORed with the handheld's e-stop flag. The bridge keeps its own fault-clear, mode-request and gesture-request counters and advances them on each change of the handheld's: because the bridge's heartbeat never stops, the robot sees no link gap when the handheld app restarts, so the bridge applies the baseline-after-gap rule itself. The first handheld sender is locked in until its liveness lapses.
- **A latching mode switch** on GPIO32 (internal pull-up) selects Bluetooth drive (open) or WiFi drive (closed to ground) at boot. A broken wire or an unfitted switch keeps Bluetooth drive. Moving it while running shows on the screen that a restart is needed, and changes nothing else.
- **Setup mode is unchanged**: holding the left button during the boot logo enters it from either drive mode; calibration stays on the portal. The radio rule becomes: the bridge never runs WiFi and Bluetooth together.
- **The bridge screen** shows WiFi drive, the network name and password, whether a handheld is connected, and the switch notice.
- **A WiFi drive probe** for the desktop, so the bridge's UDP side can be checked from a laptop before the handheld app exists.

## Capabilities

### New Capabilities

- `wifi-drive`: the mode switch, the access point, the UDP command and relay protocol, handheld liveness, counter translation and the single-handheld rule.

### Modified Capabilities

- `setup-portal`: entering and leaving setup mode from either drive mode; never WiFi and Bluetooth together.
- `status-display`: the WiFi drive content of the pages, and the switch notice.

## Impact

- `firmware/bridge/`: a third boot path (WiFi drive) beside Bluetooth drive and setup mode, the access point and UDP task, the relay, the screen.
- `core/`: a new `core/wifi_drive/` (handheld controller, counter translation, liveness, the boot-mode decision and switch notice) with host tests; `core/setup/access_point` (the WiFi drive network name); `core/protocol` (the decoder hands out the raw bytes of a decoded frame; the wire format is unchanged); `core/status/status_screen`.
- `desktop/`: a small `spooder-wifi-probe` command reusing the Go link protocol.
- `tools/bridge-screenshots`: the WiFi drive pages.
- `firmware/bridge/CLAUDE.md`, the `spooder-firmware-shape` and `spooder-hardware-check` skills, readme and changelog.
- No protocol version change; the robot firmware is unchanged.
- Hardware: the switch and its wire to GPIO32 and ground; its place on the electronics mount belongs to `add-electronics-mount`.
