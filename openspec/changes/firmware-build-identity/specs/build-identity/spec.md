## ADDED Requirements

### Requirement: Build identity derived from git
Each build of the robot firmware, the virtual robot and the bridge firmware SHALL carry a build identity: the output of `git describe --always --dirty --abbrev=12` for the checkout being built, taken at build time. When the sources are not in a git checkout, or git is not available or fails, the identity SHALL be `unknown`. The identity SHALL depend only on the checkout's git state, not on the time, user or host of the build. A build whose identity has not changed SHALL NOT recompile any source because of the identity; a changed identity SHALL recompile only the source that holds it.

#### Scenario: Clean checkout
- **WHEN** the firmware is built from a checkout with no uncommitted changes to tracked files
- **THEN** its identity is the commit's describe output, without `-dirty`

#### Scenario: Uncommitted changes
- **WHEN** a tracked file has been edited and not committed
- **THEN** the identity ends in `-dirty`

#### Scenario: New commit without reconfiguring
- **WHEN** a commit is made and the firmware is rebuilt without reconfiguring the build directory
- **THEN** the new build carries the new commit's identity

#### Scenario: Outside a checkout
- **WHEN** the firmware is built from sources without git metadata
- **THEN** its identity is `unknown`

#### Scenario: Nothing changed
- **WHEN** the firmware is rebuilt with no change to the sources or the git state
- **THEN** no source is recompiled

### Requirement: Robot build banner
The robot SHALL print `spooder robot: build <identity>` as a line on its USB console on its first control tick, and again each time a USB host opens the port. The line SHALL contain only printable ASCII: any other byte of the identity SHALL be replaced by `?`, the identity SHALL be cut at 40 characters, and an empty identity SHALL be printed as `unknown`. The banner SHALL be printed before the servo profile line on the first tick. The virtual robot SHALL print the same banner, with its own build's identity, on its USB terminal, on its first tick and each time a program opens that terminal.

#### Scenario: Console opened after boot
- **WHEN** the robot has been running and a console or the desktop calibrator opens its USB port
- **THEN** the next text line it receives is `spooder robot: build <identity>`

#### Scenario: Identity with a non-printable byte
- **WHEN** the build identity contains a byte outside printable ASCII
- **THEN** the banner shows `?` in its place and no byte outside printable ASCII is written

#### Scenario: Virtual robot
- **WHEN** a program opens the virtual robot's USB terminal
- **THEN** it receives `spooder robot: build <identity>` with the virtual robot's build identity

### Requirement: Bridge build banner
The bridge SHALL print `spooder bridge: build <identity>` on its serial console at boot, in normal operation and in setup mode, with the same printable-ASCII rules as the robot's banner. In normal operation, the start-up log on its screen SHALL include the line `Build <identity>`; when the identity does not fit the line, the screen SHALL show `..` followed by the identity's end.

#### Scenario: Normal boot
- **WHEN** the bridge powers up in normal operation
- **THEN** the console shows `spooder bridge: build <identity>` and the start-up log shows `Build <identity>`

#### Scenario: Setup mode
- **WHEN** the bridge restarts into setup mode
- **THEN** the console shows `spooder bridge: build <identity>` before the setup-mode line

#### Scenario: Long identity on screen
- **WHEN** the identity is longer than the start-up log line allows
- **THEN** the line shows `Build ..` followed by the identity's last characters, including any `-dirty`
