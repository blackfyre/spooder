# status-display Specification

## Purpose
Shows the operator the state of the controller, the link and the robot on the bridge's screen, so that faults and unsafe conditions are visible without a computer.

## Requirements

### Requirement: Status content
The bridge screen SHALL show its status on pages (see Display pages). The drive page SHALL show:
- whether a gamepad is connected and, when the gamepad reports it, its battery level;
- the link state (receiving telemetry, or lost);
- the robot's body mode, and its gait, or the gesture while one plays;
- whether servo output is on;
- supply voltage and current, as values and as bars;
- the requested height and speed, as values and as bars, and the stride and stick modes.

Every page SHALL show a status strip that names, in words: an active emergency stop, every latched fault, a lost link, and a source conflict or the USB link being in control. With none of these, it SHALL show that there are no faults. Sensor readings SHALL be shown on the drive page once telemetry reports them as available.

#### Scenario: Fault on the robot
- **WHEN** telemetry reports a latched undervoltage fault
- **THEN** the status strip names the undervoltage fault on whichever page is shown, and the drive page shows the measured voltage

#### Scenario: Emergency stop on another page
- **WHEN** the emergency stop becomes active while the power page is shown
- **THEN** the status strip shows the emergency stop at the next refresh, without changing the page

#### Scenario: Gamepad without a battery report
- **WHEN** the connected gamepad does not report a battery level
- **THEN** the drive page shows the gamepad as connected and shows no battery level

### Requirement: Link loss shown
If no valid telemetry frame has arrived for 1 second, the screen SHALL show that the robot link is lost, on every page, and SHALL stop showing robot values as current.

#### Scenario: Robot unplugged
- **WHEN** telemetry stops arriving
- **THEN** within about 1 second the status strip shows the robot link as lost, and the drive page shows no robot values

### Requirement: Display does not delay control
Screen updates SHALL NOT delay command frames beyond their 20 ms interval. The screen SHALL refresh at least twice a second, and redraw only the regions whose content changed.

#### Scenario: Busy screen
- **WHEN** many status values change at once
- **THEN** command frames continue to leave at least every 20 ms

#### Scenario: Nothing changes
- **WHEN** no shown value changes between two refreshes
- **THEN** nothing is redrawn

### Requirement: Start-up sequence
At power-up the screen SHALL show the Spooder logo, then a start-up log of the bridge's set-up steps, for no more than 3 seconds in total, and then the status screen. Command frames SHALL be sent throughout. If emergency stop becomes active during the sequence, the status screen SHALL be shown at once.

#### Scenario: Normal power-up
- **WHEN** the bridge powers up
- **THEN** the logo is shown, then the start-up log, and the status screen appears within 3 seconds

#### Scenario: Emergency stop during start-up
- **WHEN** the GPIO35 button is pressed while the logo is shown
- **THEN** the status screen is shown at once, with emergency stop active

### Requirement: Display pages
The status screen SHALL have three pages, in order: drive, power and link. It SHALL start on the drive page. After the start-up sequence, a short press of the left button (GPIO0) SHALL show the next page, from the link page back to the drive page, at the next refresh. A press SHALL count once it has been released, after being down for at least 30 ms and less than 1 s. Presses during the start-up sequence SHALL NOT change the page; holding the button during the logo still enters setup mode. The shown page SHALL be marked on screen.

#### Scenario: Cycling the pages
- **WHEN** the left button is pressed and released three times on the drive page
- **THEN** the power page, the link page and the drive page are shown in turn

#### Scenario: Bounce and long hold ignored
- **WHEN** the left button closes for under 30 ms, or is held for 1 s or longer
- **THEN** the page does not change

### Requirement: Power history page
The bridge SHALL keep the robot's supply voltage and current from telemetry for the last 60 seconds, one sample every 100 ms while the link is up, and none while it is lost. The power page SHALL show the voltage and the current over that window as traces, each with its newest value, the lowest voltage and the highest current in the window. Time without samples SHALL show as a gap. The page SHALL state that the traces are 10 Hz samples.

#### Scenario: Supply sag during a stand
- **WHEN** telemetry reported 5.65 V and 4.3 A twenty seconds ago, and 6.05 V and 0.4 A since
- **THEN** the power page shows a lowest voltage of 5.65 V and a highest current of 4.3 A

#### Scenario: Old samples leave the window
- **WHEN** the highest current was reported more than 60 seconds ago
- **THEN** it no longer counts as the window's highest current

#### Scenario: Link lost for a while
- **WHEN** the link was lost for 10 seconds within the window
- **THEN** the traces show a gap for that time

### Requirement: Link page
The link page SHALL show, while the link is up: the frames the robot's decoder accepted and rejected per second on the bridge link, the frames the bridge's decoder accepted and rejected per second, the age of the newest telemetry, and which source the robot follows (the bridge, the USB link in control, or a conflict). It SHALL also show whether a gamepad is connected and its battery level when reported. The rates SHALL be computed over about one second.

#### Scenario: Healthy link
- **WHEN** the robot accepts every command frame and telemetry arrives at 10 Hz
- **THEN** the link page shows about 50 frames per second accepted by the robot and about 10 accepted by the bridge, none rejected

### Requirement: Held extremes on the drive bars
The drive page's current bar SHALL mark the highest current reported recently, and its voltage bar the lowest voltage. A held extreme SHALL move at once to a reading at or beyond it, stay for 3 seconds after it was last reached, then move back towards the reported value at 2 A per second for the current and 0.5 V per second for the voltage, until it meets it. While the link is lost, no held extreme SHALL be shown, and holding SHALL start again from the first reading after it returns.

The current value SHALL be shown in the warning colour from a reading that exceeds the held highest current by more than 0.1 A, and the voltage value from a reading below the held lowest voltage by more than 0.05 V. While warning, every further reading beyond the held extreme SHALL keep the warning, which SHALL end 1 second after the last one. Readings within the margin SHALL NOT start a warning.

#### Scenario: New high current
- **WHEN** the current reads 0.4 A, then 4.3 A once, then 0.4 A again
- **THEN** the current value is shown in the warning colour from the 4.3 A reading for 1 second, and then in the normal colour

#### Scenario: Noise at a steady current
- **WHEN** the current wobbles by 0.05 A around a steady 0.4 A
- **THEN** the current value stays in the normal colour

#### Scenario: Deeper sag
- **WHEN** the held lowest voltage is 6.0 V and the voltage reads 5.6 V
- **THEN** the voltage value is shown in the warning colour

#### Scenario: Stand surge
- **WHEN** the current reads 4.3 A once and 0.4 A from then on
- **THEN** the current bar's mark stays at 4.3 A for 3 seconds, is at 2.3 A one second later, and meets the bar at 0.4 A

#### Scenario: Supply sag
- **WHEN** the voltage dips to 5.5 V and returns to 6.0 V
- **THEN** the voltage bar's mark stays at 5.5 V for 3 seconds, then rises to 6.0 V within a further second

### Requirement: Visual style
The status screen SHALL follow the Blackfyre design system's dark app palette: the ground in `bf-ink`, panels in `bf-ink-raised`, text in `bf-surface`, labels in `bf-ink-subtle`, good states in `bf-ok`, warnings in `bf-warn`, and the emergency stop and faults in `bf-accent` with `bf-ink` text. Labels SHALL be uppercase JetBrains Mono, and values Archivo. Every state shown in colour SHALL also be named in text.

#### Scenario: Fault strip
- **WHEN** a fault is latched
- **THEN** the status strip is filled in the accent colour with the fault's name in dark text
