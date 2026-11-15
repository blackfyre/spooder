# handheld-controller delta

## MODIFIED Requirements

### Requirement: Leaving safely
When Menu is pressed, or Power is pressed briefly or held, the handheld SHALL first request Sleep, keep sending command frames (none while `wlan0` is off the network it started on, Joining the bridge), and wait until two telemetry frames in a row, each newer by its sequence number than every robot frame taken before the press, report the body in Sleep or servo output off, for at most 3 s. When the robot frames' sequence numbers are taken afresh during the wait (after a gap of silence, or on rejoining the network), every telemetry frame taken from then on SHALL count as newer than the frames taken before the press, and no robot frame received in that moment SHALL be lost to the ordering. While it waits, no control but Select + Start (the emergency stop) SHALL change what it requests. Only then SHALL it exit (Menu), run NextUI's suspend script (a short Power press) or power off (Power held 2 s). A second press during the wait SHALL NOT start another leave. With no telemetry arriving, it SHALL wait the full 3 s before it acts.

#### Scenario: Leave while walking
- **WHEN** Menu is pressed while the robot walks
- **THEN** the next command frame requests Sleep, and the pak exits only after two telemetry frames in a row report Sleep

#### Scenario: Delayed Sleep frame
- **WHEN** Menu is pressed while the robot walks, and a telemetry frame reporting Sleep then arrives that is older than the newest frame taken before the press, or newer but followed by telemetry reporting Walk
- **THEN** the pak does not exit on that frame

#### Scenario: Rejoin during a leave
- **WHEN** Menu is pressed, `wlan0` leaves the network the handheld started on and rejoins it while the handheld waits, and after the rejoin two telemetry frames in a row report Sleep with sequence numbers at or behind the newest frame taken before the press
- **THEN** the pak exits on the second of those frames, before 3 s have passed since the press

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
