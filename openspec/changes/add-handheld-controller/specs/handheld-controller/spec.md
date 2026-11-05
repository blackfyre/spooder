# Spec Delta

## Purpose

The handheld controller: a NextUI tool pak on a TrimUI Smart Pro that drives the robot through the bridge's WiFi drive and shows what the robot reports, and the rules for joining the bridge, leaving safely and building the pak.

## ADDED Requirements

### Requirement: Input mapping
The handheld SHALL map its controls to the same desired movement state as the bridge's gamepad mapping (gamepad-control: input mapping, stick modes and dead zone), with these handheld controls standing for the gamepad's: the sticks for the sticks, A, B, X and Y for `a`, `b`, `x` and `y`, L1 and R1 for `l1` and `r1`, L2 and R2 pressed beyond half travel for `l2` and `r2`, the D-pad for the D-pad and Start for `miscHome`. Select SHALL be a modifier with no function of its own; while it is held, Start and Y SHALL NOT act as `miscHome` and `y`, and a Start or Y pressed or held with Select SHALL NOT act until it is released, whichever button is released first. Select + Start (press) SHALL toggle the emergency stop. Select + Y held 1 s SHALL request a fault clear, once per hold, in place of the gamepad's `thumbL` + `thumbR` hold. The handheld SHALL NOT request Calibrate. Menu and Power SHALL NOT reach the mapping. A connected controller SHALL be reported in every command frame while the handheld drives. A run, and a return from suspend, SHALL start with Sleep requested, as the bridge holds it at a session's start, so that Y leaves Sleep for Stand.

#### Scenario: Same state as the gamepad
- **WHEN** the same sequence of stick positions and presses is given to the handheld and to the bridge's gamepad mapping
- **THEN** both produce the same mode, translate, rotate, height, speed, gait, stride, gesture and counters

#### Scenario: Start in Sleep
- **WHEN** the handheld starts driving
- **THEN** its command frames request Sleep, and a Y press then requests Stand with the mode-request counter advanced by one

#### Scenario: Stand and walk
- **WHEN** Start is pressed while the robot is requested to stand
- **THEN** the next command frame requests Walk and the mode-request counter has advanced by one

#### Scenario: Gesture layer on the analogue trigger
- **WHEN** L2 is pressed beyond half travel and A is pressed
- **THEN** the next command frame carries the say-hi gesture with the gesture-request counter advanced by one, and A does nothing else

#### Scenario: Emergency stop
- **WHEN** Select is held and Start is pressed
- **THEN** every following command frame signals the emergency stop until Select + Start is pressed again, and the mode does not change

#### Scenario: Select released first
- **WHEN** Select + Start or Select + Y is pressed and Select is released while Start or Y is still held
- **THEN** no mode request and no Sleep toggle follow, neither then nor when Start or Y is released

#### Scenario: Fault clear by hold
- **WHEN** Select and Y are held together for 1 s
- **THEN** the fault-clear counter advances by one, Sleep is not toggled, and holding longer advances it no further

#### Scenario: Short Select + Y
- **WHEN** Select and Y are held together and released after 0.5 s
- **THEN** no fault clear is requested and Sleep is not toggled

### Requirement: Command datagrams
The handheld SHALL send one link-protocol version 3 command frame per UDP datagram to 192.168.4.1, port 52040, every 20 ms, with the sequence number advancing by one per frame. It SHALL send every datagram of a run from one socket bound once, so from one source port. It SHALL send nothing during the first 150 ms after it starts.

#### Scenario: Rate and sequence
- **WHEN** the handheld runs for 1 s after its start-up wait
- **THEN** a listener on the bridge's address receives about 50 datagrams, each exactly one valid command frame, all from one source port, with sequence numbers rising by one

#### Scenario: Start-up wait
- **WHEN** the handheld starts
- **THEN** no datagram leaves it in the first 150 ms

### Requirement: Received frames
The handheld SHALL accept a datagram only when it comes from the bridge's address and port and holds exactly one valid robot frame (version, type, length and CRC), of a type the robot sends (telemetry, calibration report, pose or diagnostics). It SHALL drop every other datagram, counting as invalid each one from the bridge's address and port, and SHALL never act on the contents of a dropped datagram.

#### Scenario: Valid telemetry
- **WHEN** a datagram from 192.168.4.1:52040 holds exactly one valid telemetry frame
- **THEN** the screen shows its values

#### Scenario: Invalid datagram
- **WHEN** a datagram holds a frame with a bad CRC, a valid frame with one extra byte, two frames, a command frame or a calibration set frame
- **THEN** it is dropped, the invalid count rises by one, and nothing on the screen changes but that count

#### Scenario: Foreign sender
- **WHEN** a valid robot frame arrives from another address or port
- **THEN** it is dropped and nothing on the screen changes

### Requirement: Drive screen
The handheld SHALL show one live screen with, from the newest telemetry: the robot's body mode, gait, whether servo output is on, the faults by name, the gesture being played, the supply voltage and the current; from its own state: the requested mode, height, speed, gait, stride mode, stick mode and emergency stop. It SHALL show each frame type's values from the frame of that type with the newest sequence number, not replacing them with an older one, except that a type whose last shown frame was taken more than 500 ms ago SHALL be taken again from its next frame. It SHALL redraw at least 10 times a second. When no telemetry has arrived for 500 ms it SHALL mark the robot's values as stale.

#### Scenario: Fault by name
- **WHEN** telemetry reports the overcurrent fault
- **THEN** the screen names the overcurrent fault

#### Scenario: Older telemetry after a newer frame
- **WHEN** telemetry 10 is shown, then diagnostics 14, telemetry 13 and telemetry 9 arrive
- **THEN** the screen shows telemetry 13's values, not telemetry 9's

#### Scenario: One type lost while the others arrive
- **WHEN** telemetry 10 is shown, then no telemetry arrives for 600 ms while pose and diagnostics keep arriving, and telemetry 140 then arrives
- **THEN** the screen shows telemetry 140's values

#### Scenario: Link lost
- **WHEN** no robot frame has arrived for 500 ms
- **THEN** the robot's values are marked stale and the time since the last frame keeps rising

### Requirement: Link quality
The handheld SHALL show the WiFi link's quality: the signal level of `wlan0`, the robot frames received per type in the last second, the invalid datagrams, the lost robot frames (gaps in the robot's sequence numbers) and the late ones (a sequence number behind the newest), counted since start, and the time since the last valid frame. The first valid frame of a run, after a resume and after 500 ms without a valid frame SHALL set the newest sequence number without counting a frame lost or late. It SHALL NOT show a round-trip time.

#### Scenario: Loss counted from sequence numbers
- **WHEN** robot frames arrive with sequence numbers 10, 11, 14
- **THEN** the lost count rises by two

#### Scenario: Late frame
- **WHEN** a robot frame with sequence number 12 arrives after 14
- **THEN** the late count rises by one and the frame is not taken as newer than 14

#### Scenario: Sequence wrap
- **WHEN** robot frames arrive with sequence numbers 254, 255, 0, 1
- **THEN** no frame is counted lost or late

#### Scenario: Gap or robot restart
- **WHEN** robot frames stop for 500 ms or more, or the handheld resumes from suspend, and the next frame's sequence number is any value
- **THEN** that frame is taken as newest and nothing is counted lost or late

### Requirement: Joining the bridge
Before it reports a link, the handheld SHALL check that `wlan0` holds an IPv4 address in 192.168.4.0/24. If it does not, it SHALL renew the lease once (`udhcpc -i wlan0 -n -q -t 5`) and check again. If there is still none, it SHALL tell the user to join the `spooder-drive-XXXX` network (password on the bridge's link page) in NextUI's WiFi settings, naming the network `wlan0` is on if any, and SHALL send no command frame until the check passes.

#### Scenario: Joined
- **WHEN** `wlan0` holds 192.168.4.2
- **THEN** the handheld starts its link without running `udhcpc`

#### Scenario: Stale lease
- **WHEN** `wlan0` holds 192.168.0.23 and the renewal gives 192.168.4.2
- **THEN** the handheld runs `udhcpc` once and then starts its link

#### Scenario: Not joined
- **WHEN** `wlan0` has no 192.168.4.x address after the renewal
- **THEN** the handheld asks the user to join the bridge's network and sends no command frame

### Requirement: Leaving safely
When Menu is pressed, or Power is pressed briefly or held, the handheld SHALL first request Sleep, keep sending command frames, and wait until two telemetry frames in a row, each newer by its sequence number than every robot frame taken before the press, report the body in Sleep or servo output off, for at most 3 s. While it waits, no control but Select + Start (the emergency stop) SHALL change what it requests. Only then SHALL it exit (Menu), run NextUI's suspend script (a short Power press) or power off (Power held 2 s). A second press during the wait SHALL NOT start another leave. With no telemetry arriving, it SHALL wait the full 3 s before it acts.

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

#### Scenario: Power pressed while standing
- **WHEN** Power is pressed briefly while the robot stands
- **THEN** Sleep is requested and confirmed before the suspend script runs

#### Scenario: Power held
- **WHEN** Power is held for 2 s
- **THEN** Sleep is requested and confirmed, or 3 s pass, before the handheld powers off

### Requirement: WiFi power saving
At start the handheld SHALL read the WiFi power-saving setting of `wlan0` and turn it off. However the pak exits (Menu, a signal, a crash of the binary, or power-off through the pak), it SHALL restore the setting it read, and stop every process it started. It SHALL restore the setting before it runs the suspend script or powers off, and turn power saving off again when a suspend returns.

#### Scenario: Normal exit
- **WHEN** power saving was on at start and the user leaves with Menu
- **THEN** power saving is on again after the pak exits

#### Scenario: Suspend
- **WHEN** power saving was on at start and the user suspends with Power
- **THEN** power saving is on when the suspend script runs, and off again after it returns

#### Scenario: Binary crashes
- **WHEN** the binary exits with an error while power saving is off
- **THEN** the launch script restores the setting it read

### Requirement: Packaging
The handheld SHALL be packaged as a NextUI tool pak, a folder `Spooder.pak` to be copied to `Tools/tg5040/` on the SD card, holding `launch.sh`, the binary for linux/arm64 and the SDL2 add-on libraries the device lacks, and delivered as a zip of that folder. The binary SHALL need no GLIBC symbol version above 2.31. The same program SHALL also build for desktop Linux (amd64), where it uses the address given on its command line instead of the fixed bridge address and skips the WiFi checks.

#### Scenario: Device binary
- **WHEN** the device build finishes
- **THEN** the binary is an aarch64 ELF whose highest required GLIBC version is at most 2.31, and the zip holds `Spooder.pak/launch.sh` and the binary

#### Scenario: Desktop start against the virtual bridge
- **WHEN** the handheld's start-up is given the virtual bridge's address on the command line and no device flag
- **THEN** it runs no WiFi step and no address check, and drives the virtual robot as it would the robot through the bridge
