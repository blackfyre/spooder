# Spec Delta

## Purpose

WiFi drive: a bridge mode in which a WiFi handheld, instead of a Bluetooth gamepad, drives the robot through the bridge, with the bridge keeping the link's safety rules and relaying the robot's frames back to the handheld.

## ADDED Requirements

### Requirement: Drive mode switch
The bridge SHALL read a latching switch between GPIO32 and ground, with the internal pull-up, once at boot, before starting either radio. Open SHALL select Bluetooth drive and closed SHALL select WiFi drive, so that a broken wire or an unfitted switch selects Bluetooth drive. A boot into setup mode SHALL ignore the switch. The bridge SHALL keep the selected drive mode until it restarts; a change of the switch while it runs SHALL NOT restart it or change the drive mode.

#### Scenario: Switch open
- **WHEN** the bridge powers up with the switch open or not fitted
- **THEN** it starts Bluetooth drive, as before this change

#### Scenario: Switch closed
- **WHEN** the bridge powers up with the switch closed to ground
- **THEN** it starts WiFi drive and never starts Bluetooth

#### Scenario: Switch moved while running
- **WHEN** the switch is moved while the bridge runs Bluetooth drive
- **THEN** the bridge keeps running Bluetooth drive, and the screen shows that a restart is needed

### Requirement: WiFi drive access point
In WiFi drive the bridge SHALL run a WPA2 access point on channel 1 at address 192.168.4.1, serving DHCP to the station that joins, and SHALL accept one station at a time. Its network name SHALL be `spooder-drive-` followed by the last four hexadecimal digits of the bridge's MAC address, distinct from the setup-mode network. Its password SHALL be generated randomly the first time WiFi drive starts, kept on the bridge, separate from the setup-mode password, and never derivable from anything the bridge broadcasts.

#### Scenario: Handheld joins
- **WHEN** a handheld joins the WiFi drive network with the password shown on the screen
- **THEN** it receives an address by DHCP in the bridge's network

#### Scenario: Setup network not joined by mistake
- **WHEN** a handheld that has joined the WiFi drive network is near the bridge in setup mode
- **THEN** it does not join the setup network on its own, since the network names differ

### Requirement: Handheld command datagrams
In WiFi drive the bridge SHALL listen on UDP port 52040. A datagram SHALL count as a handheld frame only when it holds exactly one command frame of protocol version 3 with a valid CRC and nothing else; every other datagram SHALL be discarded and counted. The bridge SHALL take at most 8 datagrams per 20 ms command period and discard the rest unprocessed. Within one handheld session, a frame whose sequence number is not newer than that of the last accepted frame (a difference of 1 to 127, modulo 256) SHALL be discarded and counted, so that a late or repeated datagram changes nothing.

#### Scenario: Valid frame
- **WHEN** the handheld sends a datagram holding one valid version 3 command frame
- **THEN** the bridge accepts it as the handheld's newest desired state

#### Scenario: Corrupt or foreign datagram
- **WHEN** a datagram has a bad CRC, another protocol version, another frame type, or extra bytes
- **THEN** the bridge discards it, counts it, and its command frames to the robot do not change

#### Scenario: Reordered datagram
- **WHEN** a frame with sequence number 41 arrives after the frame with sequence number 42
- **THEN** the bridge discards the frame with sequence number 41

### Requirement: One handheld at a time
The bridge SHALL lock to the sender (address and port) of the first accepted handheld frame and discard, and count, datagrams from every other sender until the locked sender's liveness lapses. After a lapse, the next sender of a valid frame SHALL become the locked sender.

#### Scenario: Second sender ignored
- **WHEN** a second device sends valid command frames while the locked handheld is live
- **THEN** the bridge discards them, and its command frames follow the locked handheld only

#### Scenario: Handover after a lapse
- **WHEN** the locked handheld stops sending for longer than the liveness timeout, and another device then sends valid frames
- **THEN** the bridge locks to the other device

### Requirement: Handheld liveness
A handheld session SHALL be live while the newest accepted frame from the locked sender is no older than 150 ms. While live, the bridge's command frames SHALL report a connected controller exactly when the handheld's newest frame does. When the session lapses, the bridge SHALL report no controller and request Sleep, and SHALL keep requesting Sleep until the handheld makes a new mode request in a new session. The bridge SHALL send its own command frames every 20 ms whether or not a handheld is live.

#### Scenario: Handheld goes quiet
- **WHEN** the handheld's frames stop arriving
- **THEN** within 150 ms plus one command period the bridge's command frames report no controller, so the robot sleeps

#### Scenario: Occasional losses
- **WHEN** up to seven consecutive frames of a 50 Hz handheld are lost
- **THEN** the session stays live and the controller stays connected

#### Scenario: Handheld returns
- **WHEN** the handheld sends again after a lapse, with its mode set to Walk
- **THEN** the bridge's command frames request Sleep until the handheld's mode-request counter next changes

### Requirement: Counter translation
The bridge SHALL keep its own fault-clear, mode-request and gesture-request counters for WiFi drive. The first accepted frame of a session SHALL only set the baselines of the handheld's three counters. In later frames of the session, each change of a handheld counter from the previous accepted frame SHALL advance the bridge's matching counter by one: a mode-request change SHALL also set the requested mode to the frame's mode, and a gesture-request change SHALL also set the gesture to the frame's gesture. Between mode requests, the requested mode SHALL NOT change. The other fields of the desired state (translate, rotate, height, speed, gait and stride) SHALL follow the newest accepted frame while the session is live, and SHALL be zero or their defaults otherwise.

#### Scenario: Restarted handheld app
- **WHEN** the handheld app restarts, so that its first frame carries counters starting again from zero, while the robot still sees the bridge's heartbeat
- **THEN** the bridge's counters do not change, and the robot receives no fault clear, mode request or gesture request

#### Scenario: Clear requested
- **WHEN** the handheld's fault-clear counter changes from 3 to 4 within a session
- **THEN** the bridge's fault-clear counter advances by one in its next command frame

#### Scenario: Mode request
- **WHEN** the handheld's mode-request counter changes and its frame requests Stand
- **THEN** the bridge's next command frame requests Stand with an advanced mode-request counter

### Requirement: Emergency stop in WiFi drive
The bridge's command frames SHALL carry the emergency stop when the bridge's own GPIO35 toggle is active or the handheld's newest accepted frame carries its emergency-stop flag. The handheld's flag SHALL stay in effect through a liveness lapse until an accepted frame clears it.

#### Scenario: Handheld e-stop
- **WHEN** the handheld sends frames with the emergency-stop flag set
- **THEN** the bridge's command frames carry the emergency stop

#### Scenario: Bridge button in WiFi drive
- **WHEN** the GPIO35 button is pressed in WiFi drive while the handheld's flag is clear
- **THEN** the bridge's command frames carry the emergency stop, as in Bluetooth drive

#### Scenario: E-stop then lapse
- **WHEN** the handheld sets its emergency-stop flag and its frames then stop
- **THEN** the bridge's command frames keep carrying the emergency stop until an accepted frame without the flag arrives

### Requirement: Relay of robot frames
In WiFi drive, while a handheld session is live, the bridge SHALL send every valid frame it receives from the robot to the locked sender's address and port, one frame per datagram, byte for byte as received. It SHALL relay nothing while no session is live. It SHALL NOT send its own command frames to the handheld.

#### Scenario: Telemetry relayed
- **WHEN** the robot sends a telemetry frame while a handheld is live
- **THEN** the handheld receives a datagram holding exactly that frame's bytes

#### Scenario: No handheld
- **WHEN** no handheld session is live
- **THEN** the bridge sends no datagrams
