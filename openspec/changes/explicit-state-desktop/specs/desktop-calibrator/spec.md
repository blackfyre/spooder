## MODIFIED Requirements

### Requirement: Explicit port
The app SHALL accept a serial port path on its command line (`--port PATH`). That port SHALL be listed and preselected whenever the path exists, even when the system's port enumeration does not report it (a pseudo-terminal, or a symbolic link to one), and SHALL be opened without setting modem control lines. Reconnection SHALL follow the path, so a new terminal behind the same link is found again. A port the operator picked SHALL stay selected only while it is listed; once it is gone, the selection SHALL be a preselection again, so the path is preselected when it appears.

#### Scenario: Virtual robot
- **WHEN** the app is started with `--port build/virtual-robot/usb` while the virtual robot runs
- **THEN** that port is preselected, and connecting shows the virtual robot's log and telemetry

#### Scenario: Path absent
- **WHEN** the given path does not exist
- **THEN** it is not listed, and a connection to it waits until it appears

#### Scenario: Picked port unplugged
- **WHEN** the app is not connected, the operator has picked another port, that port is unplugged while the given path does not exist, and the path then appears
- **THEN** the path is preselected
