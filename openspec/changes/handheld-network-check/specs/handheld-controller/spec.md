# handheld-controller delta

## MODIFIED Requirements

### Requirement: Joining the bridge
Before it reports a link, the handheld SHALL read the network `wlan0` is on (`iw dev wlan0 link`) and SHALL check that its name is `spooder-drive-` followed by four lower-case hex digits. If it is not, or `wlan0` is on no network, the handheld SHALL NOT renew the lease. If the name matches, it SHALL check that `wlan0` holds an IPv4 address in 192.168.4.0/24; if it does not, it SHALL renew the lease once (`udhcpc -i wlan0 -n -q -t 5`) and check again. If either check fails, it SHALL tell the user to join the `spooder-drive-XXXX` network (password on the bridge's link page) in NextUI's WiFi settings, naming the network `wlan0` is on if any, and SHALL send no command frame until both checks pass.

While it runs, the handheld SHALL read the network `wlan0` is on at least once a second. While `wlan0` is not on the network the handheld started on, it SHALL send no command frame, its header's link chip SHALL read `OFF NETWORK`, and its robot panel, or on the driving view the view's title, SHALL name the network to rejoin. A read that fails SHALL leave the last verdict unchanged. When `wlan0` is on that network again, the handheld SHALL send again with Sleep requested, every control held at that moment ignored until released, and the next robot frame taken as newest, as after a resume.

#### Scenario: Joined
- **WHEN** `wlan0` is on `spooder-drive-1a2b` and holds 192.168.4.2
- **THEN** the handheld starts its link without running `udhcpc`

#### Scenario: Stale lease
- **WHEN** `wlan0` is on `spooder-drive-1a2b` and holds 192.168.0.23, and the renewal gives 192.168.4.2
- **THEN** the handheld runs `udhcpc` once and then starts its link

#### Scenario: Not joined
- **WHEN** `wlan0` is on `spooder-drive-1a2b` and has no 192.168.4.x address after the renewal
- **THEN** the handheld asks the user to join the bridge's network and sends no command frame

#### Scenario: Another network in the bridge's range
- **WHEN** `wlan0` is on `spooder-1a2b` (the bridge's setup network) or `Home WiFi`, holding 192.168.4.2
- **THEN** the handheld runs no `udhcpc`, asks the user to join the bridge's network naming the network `wlan0` is on, and sends no command frame

#### Scenario: Leaving the network while running
- **WHEN** the handheld started on `spooder-drive-1a2b` and `wlan0` moves to `Home WiFi` or to no network
- **THEN** within about a second it sends no more command frames, the link chip reads `OFF NETWORK`, and the robot panel or the driving view's title names `spooder-drive-1a2b`

#### Scenario: Rejoining
- **WHEN** `wlan0` is back on `spooder-drive-1a2b` after the handheld went inactive, with Y held
- **THEN** the handheld sends again, the first frame requesting Sleep, and Y changes nothing until released and pressed again

#### Scenario: Another bridge
- **WHEN** the handheld started on `spooder-drive-1a2b` and `wlan0` moves to `spooder-drive-9f00`
- **THEN** it stays inactive

### Requirement: Command datagrams
The handheld SHALL send one link-protocol version 3 command frame per UDP datagram to 192.168.4.1, port 52040, every 20 ms, with the sequence number advancing by one per frame, except while `wlan0` is off the network it started on (Joining the bridge), when it SHALL send nothing and the sequence number SHALL NOT advance. It SHALL send every datagram of a run from one socket bound once, so from one source port. It SHALL send nothing during the first 150 ms after it starts.

#### Scenario: Rate and sequence
- **WHEN** the handheld runs for 1 s after its start-up wait
- **THEN** a listener on the bridge's address receives about 50 datagrams, each exactly one valid command frame, all from one source port, with sequence numbers rising by one

#### Scenario: Start-up wait
- **WHEN** the handheld starts
- **THEN** no datagram leaves it in the first 150 ms

#### Scenario: Held off the network
- **WHEN** the handheld goes off its network for five ticks between frames with sequence numbers 2 and 3
- **THEN** no datagram leaves it in those ticks, and the next frame has sequence number 3

### Requirement: Leaving safely
When Menu is pressed, or Power is pressed briefly or held, the handheld SHALL first request Sleep, keep sending command frames (none while `wlan0` is off the network it started on, Joining the bridge), and wait until two telemetry frames in a row, each newer by its sequence number than every robot frame taken before the press, report the body in Sleep or servo output off, for at most 3 s. While it waits, no control but Select + Start (the emergency stop) SHALL change what it requests. Only then SHALL it exit (Menu), run NextUI's suspend script (a short Power press) or power off (Power held 2 s). A second press during the wait SHALL NOT start another leave. With no telemetry arriving, it SHALL wait the full 3 s before it acts.

#### Scenario: Leave while walking
- **WHEN** Menu is pressed while the robot walks
- **THEN** the next command frame requests Sleep, and the pak exits only after two telemetry frames in a row report Sleep

#### Scenario: Delayed Sleep frame
- **WHEN** Menu is pressed while the robot walks, and a telemetry frame reporting Sleep then arrives that is older than the newest frame taken before the press, or newer but followed by telemetry reporting Walk
- **THEN** the pak does not exit on that frame

#### Scenario: Controls during the wait
- **WHEN** Y, Start or a stick is pressed while the handheld waits to leave
- **THEN** Sleep stays requested

#### Scenario: Robot does not confirm
- **WHEN** Menu is pressed and no telemetry reports Sleep or output off
- **THEN** the pak exits 3 s after the press

#### Scenario: Leave off the network
- **WHEN** Menu is pressed while `wlan0` is off the network the handheld started on
- **THEN** no command frame is sent, and the pak exits 3 s after the press

#### Scenario: Power pressed while standing
- **WHEN** Power is pressed briefly while the robot stands
- **THEN** Sleep is requested and confirmed before the suspend script runs

#### Scenario: Power held
- **WHEN** Power is held for 2 s
- **THEN** Sleep is requested and confirmed, or 3 s pass, before the handheld powers off
