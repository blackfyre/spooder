# Spec Delta

## ADDED Requirements

### Requirement: Explicit port
The app SHALL accept a serial port path on its command line (`--port PATH`). That port SHALL be listed and preselected whenever the path exists, even when the system's port enumeration does not report it (a pseudo-terminal, or a symbolic link to one), and SHALL be opened without setting modem control lines. Reconnection SHALL follow the path, so a new terminal behind the same link is found again.

#### Scenario: Virtual robot
- **WHEN** the app is started with `--port build/virtual-robot/usb` while the virtual robot runs
- **THEN** that port is preselected, and connecting shows the virtual robot's log and telemetry

#### Scenario: Path absent
- **WHEN** the given path does not exist
- **THEN** it is not listed, and a connection to it waits until it appears
