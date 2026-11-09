# Proposal

## Why

The pak takes any `wlan0` address in 192.168.4.0/24 as being on the bridge. The bridge's setup network (`spooder-XXXX`) hands out addresses in the same range, as many home and phone hotspot networks can, so the pak can start and send command frames on a network that is not the bridge's WiFi drive. Once running, it never checks the network again: if NextUI's WiFi moves `wlan0` to another network, the pak keeps sending to 192.168.4.1 there.

## What Changes

- **Start check by network name.** Before its address check, the pak reads the network `wlan0` is on (`iw dev wlan0 link`). It starts only on a network named `spooder-drive-` and four lower-case hex digits, the name the bridge's WiFi drive gives its access point (`core/setup/access_point`); on any other network, or none, it shows the join message naming that network, without renewing the lease. The address check and its one renewal stay.
- **Inactive off that network.** While it runs, the pak reads the network with the signal level, once a second. While `wlan0` is not on the network the pak started on, it sends no command frame, the header's link chip reads `OFF NETWORK` and the robot panel's notice (on the driving view, its title) names the network to rejoin. The bridge's liveness lapse then sleeps the robot, as when the handheld goes out of range. Back on that network, it sends again with Sleep requested, the controls held at that moment masked until released and the link's sequence numbers rebaselined, as after a resume from suspend. A network read that fails keeps the last verdict.
- Off the device (`--addr`) nothing changes: no check runs.
- No protocol, firmware or bridge change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `handheld-controller`: the joining requirement checks the network's name at the start and keeps the pak inactive off that network while it runs; the command datagram and leaving requirements gain that exception. The capability is added by `add-handheld-controller`, merged but not yet archived; this delta builds on its text.

## Impact

- `desktop/handheld/internal/wlan` (the name pattern, the start check, one `iw dev wlan0 link` read giving the network and the signal level), `internal/udplink` (a step can hold a tick unsent), `internal/app` (the expected network, the off-network state and its rejoin, the screen model), `cmd/spooder-handheld/screen.go` (the chip and the notice), their tests, `docs/handheld.md`, `desktop/CLAUDE.md`, the `spooder-hardware-check` skill's handheld checks and `CHANGELOG.md`.
