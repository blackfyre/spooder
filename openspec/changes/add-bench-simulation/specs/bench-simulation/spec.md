# Spec Delta

## Purpose

Lets the robot firmware on the Servo 2040 be driven end to end on the bench with USB only and no servo supply: movement, telemetry, pose frames and protection behave as on a powered robot, while no servo ever moves.

## ADDED Requirements

### Requirement: Simulation offered at start-up
The robot firmware SHALL offer bench simulation for a window of 3 s after start-up. Throughout the window the status LEDs SHALL show, alongside the normal start-up indication, one of two patterns, each distinct from every other indication: while no USB host has enumerated the board (or while it has suspended it), that the window is open but a press would be refused; once one has, that simulation is available. The robot SHALL run in live mode from its first control tick, and offering simulation SHALL NOT delay any control tick. The window SHALL close early, for the rest of the boot, once servo output has been on.

#### Scenario: Normal start-up
- **WHEN** the robot starts and the USER button is not pressed
- **THEN** the LEDs show that simulation is available for 3 s, the robot is in live mode from its first tick, and after 3 s the indication stops and the robot stays in live mode

#### Scenario: Available once USB is ready
- **WHEN** the robot starts on USB and the host enumerates the board 1 s after start-up
- **THEN** for that first second the LEDs show the window open with USB not ready, and from then until 3 s after start-up that simulation is available

#### Scenario: Output already on
- **WHEN** servo output has switched on within the window
- **THEN** the window closes at once and a later press within the 3 s does not enter simulation

### Requirement: Entry into simulation
A press of the board's USER button (GPIO23) inside the window SHALL enter simulation only if all of these hold at the press:
- the button was seen released and then pressed inside the window (a button already held when the window opens does not count until it has been released);
- the servo supply voltage has read below 1 V on every control tick from start-up until the press;
- servo output is off;
- a USB host has enumerated the board and has not suspended it.

A charger or a cable with no host SHALL read as no USB host. Only the first press in the window SHALL be decided; after it the window closes, whether simulation was entered or refused.

#### Scenario: Entry on the bench
- **WHEN** the board runs from USB only, enumerated by a computer, and the USER button is pressed and released 1 s after start-up
- **THEN** the robot enters simulation

#### Scenario: Stuck button
- **WHEN** the USER button reads pressed from the first tick to the end of the window
- **THEN** the robot stays in live mode

#### Scenario: Supply present
- **WHEN** the servo supply has read 1 V or more on any tick since start-up and the USER button is then pressed in the window
- **THEN** entry is refused and the robot stays in live mode

#### Scenario: Servo rail fed from USB
- **WHEN** the board's USB-to-servo-rail trace is uncut, so the servo supply reads about 5 V, and the USER button is pressed in the window
- **THEN** entry is refused

#### Scenario: No USB host
- **WHEN** the board is powered from a charger, or its USB host has not enumerated it or has suspended it, and the USER button is pressed in the window
- **THEN** entry is refused

#### Scenario: Press after the window
- **WHEN** the USER button is pressed 3 s or more after start-up
- **THEN** nothing changes

### Requirement: Refused entry shown
When entry is refused, the status LEDs SHALL show, for 3 s, why: the supply was present, or no USB host was attached, each with its own pattern distinct from every other indication. The supply reason SHALL take precedence when both apply. The robot SHALL stay in live mode.

#### Scenario: Supply still draining
- **WHEN** the servo supply was disconnected just before a reset, so its capacitor still reads above 1 V, and the USER button is pressed in the window
- **THEN** the LEDs show the supply reason for 3 s and the robot stays in live mode

### Requirement: Simulation lasts until restart
Once entered, simulation SHALL last until the next restart, whatever its cause (power, RESET, the watchdog, a 1200-baud touch). It SHALL be held in RAM only: nothing that survives a restart, including the watchdog scratch registers and flash, SHALL record it. Losing the USB host during simulation SHALL NOT end it.

#### Scenario: USB unplugged in simulation
- **WHEN** the robot is in simulation and its USB host disconnects or suspends it
- **THEN** the robot stays in simulation and no servo produces a pulse

#### Scenario: Restart
- **WHEN** the robot restarts after simulation
- **THEN** it starts in live mode and offers simulation again

### Requirement: No servo pulse in simulation
In simulation, no servo output SHALL produce a pulse, whatever the body, protection or any link requests. Servo output SHALL be forced off when simulation is entered, and the robot SHALL NOT be able to switch it on again until restart. The body, the servo-output rules (switch-on in groups of two legs, inhibit, stop) and the output rate backstop SHALL act on a servo model instead, so movement, telemetry and pose frames behave as on a powered robot.

#### Scenario: Walking in simulation
- **WHEN** the robot is in simulation and a link requests Stand and then Walk
- **THEN** the body stands and walks, pose frames report the modelled pulses and output as on, and no servo output produces a pulse

#### Scenario: Calibration in simulation
- **WHEN** a calibration change is accepted in Calibrate during simulation
- **THEN** the modelled pulses follow it and no servo output produces a pulse

### Requirement: Modelled supply in simulation
In simulation, power protection SHALL evaluate the readings of the virtual robot's supply model instead of the measured supply: a steady 6.0 V, and a current that is zero with the modelled output off, an idle draw per powered servo and a draw that grows with the pulse change per tick. Protection SHALL apply its rules to those readings unchanged, including the 500 ms settling, the emergency stop and fault clearing. Telemetry and the debug output SHALL report the modelled readings.

#### Scenario: Supply accepted in simulation
- **WHEN** the robot enters simulation
- **THEN** protection accepts the modelled supply after 500 ms and the body starts on the next new request

#### Scenario: Emergency stop in simulation
- **WHEN** a link signals the emergency stop in simulation and later sends a fault clear and a new request
- **THEN** the emergency-stop fault latches, the modelled output goes off, and after the clear and the request the body restarts through Initialise and Stand

### Requirement: Supply appearing in simulation
In simulation the firmware SHALL keep measuring the real servo supply. If it reads above the brown-out voltage on any tick, the simulation supply fault SHALL latch: it inhibits output, stops the body, is reported on the debug output, in telemetry and on the LEDs (with an indication distinct from every other fault), and a fault clear SHALL NOT clear it until restart. The robot SHALL stay in simulation.

#### Scenario: Servo supply connected during simulation
- **WHEN** a 6 V servo supply is connected while the robot walks in simulation
- **THEN** the simulation supply fault latches, the body stops, telemetry reports the fault, and no servo output produces a pulse

#### Scenario: Clear refused
- **WHEN** the simulation supply fault is latched and a fault clear arrives
- **THEN** the fault stays latched

### Requirement: Simulation reported
Every telemetry frame sent in simulation SHALL set the simulation flag (bit 5 of the telemetry flags byte); telemetry in live mode SHALL leave it clear. In simulation, the telemetry's servo-output field and the pose frame's output flag SHALL report the modelled output. The status line on the debug output SHALL name simulation while it is active, and the robot SHALL report entering simulation and a refused entry, with its reason, once on the debug output. The link protocol version SHALL stay 3: receivers ignore flag and fault bits they do not know. The virtual robot SHALL report the flag clear.

#### Scenario: Telemetry in simulation
- **WHEN** the robot is in simulation and walking
- **THEN** each telemetry frame has the simulation flag set and reports output as on

#### Scenario: Older receiver
- **WHEN** a receiver that does not know the simulation flag decodes a telemetry frame with it set
- **THEN** it decodes the frame and shows the other fields as before
