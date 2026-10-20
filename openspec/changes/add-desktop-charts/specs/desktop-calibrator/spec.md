# Spec Delta

## ADDED Requirements

### Requirement: Charts
The app SHALL keep the last ten minutes of telemetry, diagnostics and pose on the current connection, and SHALL show them in a charts tab as stacked charts sharing one time axis:
- supply voltage, with the minimum, maximum and brown-out voltages;
- current and averaged current, with the trip and sustained currents;
- accepted frames per second on the robot's link, the bridge's link and the host link;
- rejected frames per second on the same three links;
- a timeline of the body mode, servo output and faults;
- the fastest joint speed of each joint kind (coxa, femur, tibia), over all legs, with the joint speed budget and the rated speed;
- the selected leg's servo angles, with the 0° and 180° limits;
- the budget's motion share;
- the control loop's longest and mean working time, with the 20 ms period;
- backstop hits per second.

Each chart SHALL have one value axis. Charts of two or more series SHALL have a legend, and reference lines SHALL be named. A window selector SHALL choose the last 30 seconds, 2 minutes or 10 minutes. Hovering a chart SHALL show the time and the values at that moment on every chart. Charts whose data the robot does not send (a robot without diagnostics, or no pose frames) SHALL say so instead of drawing nothing.

#### Scenario: Supply sag while walking
- **WHEN** the robot walks and the supply voltage dips
- **THEN** the voltage chart shows the dip against the minimum voltage, and the timeline shows Walk at that time

#### Scenario: Window
- **WHEN** the operator selects 30 s
- **THEN** every chart shows the last 30 seconds, and selecting 10 min shows up to ten minutes

#### Scenario: Hover
- **WHEN** the operator hovers a moment on any chart
- **THEN** every chart marks that moment and shows its values there

#### Scenario: New connection
- **WHEN** the app connects to a robot
- **THEN** the history starts empty

### Requirement: CSV export of the history
The charts tab SHALL offer to save the kept history as a CSV file: one row per telemetry sample, with its time, the telemetry values, the latest diagnostics and the joint speeds at that time, and a header row naming the columns and their units. The app SHALL write the file whole (written to a temporary file, then renamed), in the user's downloads directory (the home directory when there is none), and SHALL name the file it wrote.

#### Scenario: Save
- **WHEN** the operator saves the history after a minute of telemetry
- **THEN** a CSV file with a header row and about 600 rows is written, and the app shows its path
