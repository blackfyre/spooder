# Design

## D1. The network's name, then the address

The start check reads `iw dev wlan0 link` once. A network name that does not match `^spooder-drive-[0-9a-f]{4}$` fails at once with the join message naming it (or naming none): a lease renewal cannot move `wlan0` to another network, so it would only add up to five DHCP tries before the same message. A matching name goes on to the existing address check with its one renewal, which still catches a stale lease from an earlier network. The pattern copies `access_point::driveSsid` (`spooder-drive-%02x%02x`, lower case); the setup network's `spooder-%02x%02x` does not match it.

## D2. Inactive means no frame sent

While `wlan0` is off the network the pak started on, `udplink` skips the tick's send: the sequence number does not advance and nothing is written. The step still runs, so the leave state machine, the e-stop toggle and the screen keep working, and a leave started off the network waits its 3 s and acts, as with no telemetry. No frame is better than a frame to 192.168.4.1 on a stranger's network, and the bridge already treats silence safely: its 150 ms liveness lapse sleeps the robot.

The pak compares with the network it started on, not with the pattern: another bridge's `spooder-drive-` network is another robot, and driving it after a network switch would be a surprise. Rejoining it needs a restart.

## D3. One read a second

The signal sample already runs `iw dev wlan0 link` once a second on the TrimUI, which has no `/proc/net/wireless`. It now always runs on the device and yields both the network name and the signal level; `/proc/net/wireless`, where present, still gives the level first. So the check adds no process on the TrimUI and a network change is seen within about a second. A read that fails (iw missing, timed out) says nothing about the network, so the last verdict stands; at the start, a failed read reads as no network and the join message shows.

## D4. Rejoining

Back on the expected network, the next tick does what a resume from suspend does: rebaseline the link, request Sleep and mask the held controls until released. The robot has slept through the lapse anyway; requesting Sleep makes the pak's requested state agree with it, so standing again is a deliberate press.
