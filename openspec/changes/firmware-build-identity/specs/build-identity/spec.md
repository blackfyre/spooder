## ADDED Requirements

### Requirement: Build identity derived from git
Each build of the robot firmware, the virtual robot and the bridge firmware SHALL carry a build identity: the output of `git describe --always --dirty --abbrev=12` for the checkout being built, taken at build time. When the sources are not in a git checkout, or git is not available or fails, the identity SHALL be `unknown`. The identity SHALL depend only on the checkout's git state, not on the time, user or host of the build. The identity SHALL be embedded so that any identity git can produce compiles, including a tag name containing `"`. A build whose identity has not changed SHALL NOT recompile any source because of the identity; a changed identity SHALL recompile only the source that holds it.

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

#### Scenario: Git missing or failing
- **WHEN** the `git` command is not found, or `git describe` exits with an error
- **THEN** the identity is `unknown` and the build succeeds

#### Scenario: Tag with a quote
- **WHEN** the checkout is described by an annotated tag whose name contains `"`
- **THEN** the build compiles and carries that describe output as its identity

#### Scenario: Nothing changed
- **WHEN** the firmware is rebuilt with no change to the sources or the git state
- **THEN** no source is recompiled

### Requirement: Robot build banner
The robot SHALL print `spooder robot: build <identity>` as a line on its USB console on its first control tick, and again on each control tick where it sees a USB host connected after a tick where none was. The connection SHALL be sampled once per control tick; a first tick with a host already connected SHALL print the banner once. The banner SHALL be the first line the robot logs on the tick it is printed. While the host stays connected the banner SHALL NOT be repeated. The line SHALL contain only printable ASCII: any other byte of the identity SHALL be replaced by `?`, the identity SHALL be cut at 40 characters, and an empty identity SHALL be printed as `unknown`. The banner SHALL be printed before the servo profile line on the first tick. The virtual robot SHALL print the same banner, with its own build's identity, on its USB terminal, on its first tick and on each tick where it sees that terminal opened after a tick where it was not.

#### Scenario: Console opened after boot
- **WHEN** the robot has been running with no USB host and a console or the desktop calibrator opens its USB port and keeps it open for more than one control tick
- **THEN** within one control tick the robot writes `spooder robot: build <identity>`, before any other text line of that tick

#### Scenario: First tick
- **WHEN** the robot runs its first control tick
- **THEN** it logs the banner, then the servo profile line

#### Scenario: Host connected from the first tick
- **WHEN** a USB host is already connected on the first control tick
- **THEN** the banner is printed once on that tick, not twice

#### Scenario: Host stays connected
- **WHEN** a USB host stays connected for many control ticks
- **THEN** no further banner is printed until it disconnects and connects again

#### Scenario: Empty identity
- **WHEN** the build identity is empty
- **THEN** the banner reads `spooder robot: build unknown`

#### Scenario: Long identity
- **WHEN** the build identity is longer than 40 characters
- **THEN** the banner shows its first 40 characters

#### Scenario: Identity with a non-printable byte
- **WHEN** the build identity contains a byte outside printable ASCII
- **THEN** the banner shows `?` in its place and no byte outside printable ASCII is written

#### Scenario: Virtual robot
- **WHEN** a program opens the virtual robot's USB terminal while no other program holds it
- **THEN** it receives `spooder robot: build <identity>` with the virtual robot's build identity

### Requirement: Bridge build banner
The bridge SHALL print `spooder bridge: build <identity>` on its serial console at boot, in normal operation and in setup mode, with the same printable-ASCII rules as the robot's banner. In normal operation, the first line of the start-up log on its screen SHALL be `Build <identity>`; when the identity does not fit the line, the screen SHALL show `..` followed by the identity's end.

#### Scenario: Normal boot
- **WHEN** the bridge powers up in normal operation
- **THEN** the console shows `spooder bridge: build <identity>` and the start-up log's first line is `Build <identity>`

#### Scenario: Setup mode
- **WHEN** the bridge restarts into setup mode
- **THEN** the console shows `spooder bridge: build <identity>` before the setup-mode line

#### Scenario: Long identity on screen
- **WHEN** the identity is longer than the start-up log line allows
- **THEN** the line shows `Build ..` followed by the identity's last characters, including any `-dirty`
