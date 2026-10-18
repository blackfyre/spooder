# Spec Delta

## Purpose

A host program that runs the robot's own control logic and speaks the link protocol over pseudo-terminals, so front-ends can be exercised end to end without the Servo 2040.

## ADDED Requirements

### Requirement: The robot's own logic
The virtual robot SHALL run the same control tick as the robot firmware, compiled from the same source: link decoding and arbitration, power protection, the supervisor, calibration guards, reports and rate limit, the body state machine and kinematics, the status LED state and telemetry. It SHALL NOT reimplement any of these. Only the hardware is simulated: the servo outputs, the supply readings and the terminals.

#### Scenario: Same behaviour as the board
- **WHEN** a front-end sends the same frames to the virtual robot as to the Servo 2040, with the same supply readings
- **THEN** the virtual robot answers with the same telemetry, calibration reports and debug text, in the same order

### Requirement: USB terminal
The virtual robot SHALL open a pseudo-terminal that stands for the robot's USB port, print its path on start, and on request keep a symbolic link to it at a given path. Link frames and debug text SHALL share the terminal as on the board: each frame whole, text as printable ASCII lines, neither inside the other. While no program has the terminal open, output SHALL be dropped, as the board drops output without a USB host. Pulling the USB cable SHALL be simulable: the terminal closes (its link removed), and replugging opens a new one at the same link path.

#### Scenario: Calibrator connects
- **WHEN** the desktop calibrator opens the terminal's link path
- **THEN** it shows the robot's debug log and live telemetry, as with the Servo 2040

#### Scenario: Unplug and replug
- **WHEN** the operator unplugs and then replugs the virtual USB cable
- **THEN** the calibrator reports the connection lost, then reconnects through the same link path

### Requirement: Bridge terminal and simulated bridge
On request the virtual robot SHALL open a second pseudo-terminal for the bridge link (UART1). Without it, a simulated bridge MAY feed command frames into the bridge link's decoder: with or without a connected gamepad, and with or without the e-stop signalled. Its frames SHALL go through the same decoder and receiver as frames from a terminal.

#### Scenario: E-stop from the simulated bridge
- **WHEN** the operator signals the e-stop on the simulated bridge
- **THEN** the robot latches EMERGENCY_STOP within one tick and output goes off, whichever link is in control

#### Scenario: Gamepad on both links
- **WHEN** the simulated bridge reports a connected gamepad while the calibrator is in control
- **THEN** the robot reports a source conflict and sleeps

### Requirement: Timing modes
The virtual robot SHALL tick every 20 ms of virtual time. In real time it SHALL pace ticks to 20 ms of wall-clock time; with a speed factor it SHALL pace them that many times faster; in stepped mode it SHALL tick only when told to, a given number of ticks at once. Robot time SHALL be the virtual time in every mode.

#### Scenario: Stepped
- **WHEN** the virtual robot runs stepped and is told to step 5 ticks
- **THEN** robot time advances by exactly 100 ms and one telemetry frame is sent per link

### Requirement: Simulated supply and current
The supply voltage SHALL be settable. The current SHALL be simulated from the servo outputs: zero with output off, an idle draw per powered servo, and a draw that grows with the pulse change per tick, scaled by a settable load factor. A fixed current MAY be set instead, until the simulation is restored. Protection SHALL see these readings exactly as it sees the board's.

#### Scenario: Overvoltage
- **WHEN** the supply is set above the configured maximum voltage
- **THEN** protection latches OVERVOLTAGE, output goes off, and telemetry reports the fault

#### Scenario: Current grows with motion
- **WHEN** the body moves its legs with output on
- **THEN** the simulated current rises above the idle draw, and with a high enough load factor protection trips

### Requirement: Control commands and observation
The virtual robot SHALL read line commands on standard input: at least the supply voltage, a fixed or simulated current, the load factor, the simulated bridge (off, present, gamepad, e-stop), unplug and plug, stepping, and quit. It SHALL answer each command on standard output, and SHALL report on standard output, with the robot time, each change of the applied calibration and of the servo output state, so tests can check what the hardware would have done.

#### Scenario: Applied calibration follows at the rate limit
- **WHEN** a calibration change is accepted
- **THEN** the virtual robot reports the applied value moving towards it by at most 60° per second through the joint's calibration, until it arrives
