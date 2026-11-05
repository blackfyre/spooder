# Spec Delta

## ADDED Requirements

### Requirement: Virtual bridge in WiFi drive
On request the virtual robot SHALL run a virtual bridge in WiFi drive in place of the bridge terminal and the simulated bridge: it SHALL listen for UDP datagrams on a given address and port (default 127.0.0.1:52040), apply to them the bridge's own WiFi drive rules, compiled from the same source as the bridge firmware (datagram validation, the rate limit, the single-handheld lock, the sequence rule, liveness, counter translation, Sleep until a session's first mode request, and the e-stop flag), and feed one command frame per tick into the bridge link's decoder, built from the resulting desired state as the bridge builds it. It SHALL keep its receive backlog smaller than the datagrams it reads in one tick and read them in that tick, so that none is taken in a later tick than the one it arrived in, and its work per tick SHALL stay bounded however fast a sender sends. It SHALL send every frame the robot sends on the bridge link to the locked handheld, one frame per datagram, byte for byte, and nothing while no session is live. The `estop` command SHALL act as the bridge's GPIO35 button, ORed with the handheld's flag. It SHALL report each session start and end on standard output, with the robot time, and with each end the session's longest wall-clock interval between tick starts. It SHALL run only in real time, since the handheld's timing is wall-clock time.

#### Scenario: Handheld drives the virtual robot
- **WHEN** a program sends valid command frames at 50 Hz to the virtual bridge and changes the mode request to Stand
- **THEN** the virtual robot stands, and the program receives the robot's telemetry, pose and diagnostics frames from the virtual bridge's port

#### Scenario: Session starts in Sleep
- **WHEN** a program's first datagrams request Walk with no change of the mode-request counter
- **THEN** the robot stays in Sleep until the counter changes

#### Scenario: Handheld goes quiet
- **WHEN** the program stops sending for more than 150 ms
- **THEN** the virtual robot reports the session's end, the robot sleeps with no controller, and no more frames are sent to the program

#### Scenario: Flood then silence
- **WHEN** a program sends 500 valid datagrams at once and then stops
- **THEN** the virtual bridge takes at most 8 of them per 20 ms window, and the virtual robot reports the session's end within 190 ms of the last datagram

#### Scenario: Sustained flood
- **WHEN** the locked handheld sends valid datagrams as fast as it can for 1 s
- **THEN** the session's end reports no interval between tick starts longer than 40 ms, and the handheld keeps receiving telemetry at about 10 frames a second

#### Scenario: Second handheld
- **WHEN** a second program sends valid frames from another port while the first one's session is live
- **THEN** its frames are ignored and it receives nothing

#### Scenario: Not with a speed factor
- **WHEN** the virtual bridge is requested together with stepped mode or a speed factor
- **THEN** the virtual robot refuses to start and says why
