## MODIFIED Requirements

### Requirement: Explicit port
The app SHALL accept a serial port path on its command line (`--port PATH`). That port SHALL be listed whenever the path exists, even when the system's port enumeration does not report it (a pseudo-terminal, or a symbolic link to one), SHALL be preselected whenever it is listed and no other listed port is kept selected (the operator's pick, or while connected, the current selection), and SHALL be opened without setting modem control lines. Reconnection SHALL follow the path, so a new terminal behind the same link is found again. While the app is not connected, a port the operator picked SHALL stay selected, ahead of the path, while it is listed; once it is gone, the selection SHALL be a preselection again at once (the path, else a Servo 2040), so the path is preselected when it appears.

#### Scenario: Virtual robot
- **WHEN** the app is started with `--port build/virtual-robot/usb` while the virtual robot runs
- **THEN** that port is preselected, and connecting shows the virtual robot's log and telemetry

#### Scenario: Path absent
- **WHEN** the given path does not exist
- **THEN** it is not listed, and a connection to it waits until it appears

#### Scenario: Operator's pick kept
- **WHEN** the app is not connected, the given path exists, the operator picks another port, and the port list then changes with both still listed
- **THEN** the picked port stays selected

#### Scenario: Picked port unplugged
- **WHEN** the app is not connected, the operator has picked a port other than the given path and a Servo 2040, and that port is unplugged while the given path does not exist and a Servo 2040 is listed
- **THEN** the Servo 2040 is preselected at once, and once the path appears, the path is preselected
