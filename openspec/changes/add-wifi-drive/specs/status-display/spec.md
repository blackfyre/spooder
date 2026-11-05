# Spec Delta

## MODIFIED Requirements

### Requirement: Status content
The bridge screen SHALL show its status on pages (see Display pages). The drive page SHALL show:
- in Bluetooth drive, whether a gamepad is connected and, when the gamepad reports it, its battery level; in WiFi drive, that the bridge is in WiFi drive and whether a handheld is connected;
- the link state (receiving telemetry, or lost);
- the robot's body mode, and its gait, or the gesture while one plays;
- whether servo output is on;
- supply voltage and current, as values and as bars;
- the requested height and speed, as values and as bars, and the stride and stick modes.

Every page SHALL show a status strip that names, in words: an active emergency stop, every latched fault, a lost link, a source conflict or the USB link being in control, and a drive mode switch that no longer matches the running drive mode. With none of these, it SHALL show that there are no faults. Sensor readings SHALL be shown on the drive page once telemetry reports them as available.

#### Scenario: Fault on the robot
- **WHEN** telemetry reports a latched undervoltage fault
- **THEN** the status strip names the undervoltage fault on whichever page is shown, and the drive page shows the measured voltage

#### Scenario: Emergency stop on another page
- **WHEN** the emergency stop becomes active while the power page is shown
- **THEN** the status strip shows the emergency stop at the next refresh, without changing the page

#### Scenario: Gamepad without a battery report
- **WHEN** the connected gamepad does not report a battery level
- **THEN** the drive page shows the gamepad as connected and shows no battery level

#### Scenario: Handheld in WiFi drive
- **WHEN** the bridge is in WiFi drive and a handheld session is live
- **THEN** the drive page shows WiFi drive with the handheld connected, and no gamepad battery

### Requirement: Link page
The link page SHALL show, while the link is up: the frames the robot's decoder accepted and rejected per second on the bridge link, the frames the bridge's decoder accepted and rejected per second, the age of the newest telemetry, and which source the robot follows (the bridge, the USB link in control, or a conflict). In Bluetooth drive it SHALL also show whether a gamepad is connected and its battery level when reported. In WiFi drive it SHALL instead show the WiFi drive network name and password, and whether a handheld is connected, whether or not the link is up. The rates SHALL be computed over about one second.

#### Scenario: Healthy link
- **WHEN** the robot accepts every command frame and telemetry arrives at 10 Hz
- **THEN** the link page shows about 50 frames per second accepted by the robot and about 10 accepted by the bridge, none rejected

#### Scenario: Joining WiFi drive
- **WHEN** the bridge is in WiFi drive and no handheld has joined
- **THEN** the link page shows the network name and the password to join with, and no handheld connected

## ADDED Requirements

### Requirement: Drive mode switch notice
In Bluetooth drive and in WiFi drive, the bridge SHALL sample the drive mode switch while it runs. When the switch has stayed in the position that selects the other drive mode for at least 100 ms, the status strip SHALL name it in the warning tone with a request to restart, until the switch is back in the running drive mode's position for at least 100 ms. The notice SHALL NOT change the drive mode.

#### Scenario: Switch moved to WiFi drive
- **WHEN** the switch is closed while the bridge runs Bluetooth drive
- **THEN** the status strip asks for a restart to change the drive mode, and the bridge stays in Bluetooth drive

#### Scenario: Switch moved back
- **WHEN** the switch is returned to the running drive mode's position
- **THEN** the notice disappears from the status strip

#### Scenario: Contact bounce
- **WHEN** the switch reading changes for less than 100 ms
- **THEN** no notice is shown
