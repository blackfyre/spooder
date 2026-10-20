# Spec Delta

## Purpose

Protects the servos, the Servo 2040 and the power supply from damage. It monitors the servo supply current and voltage and stops the robot safely when either goes out of range.

## ADDED Requirements

### Requirement: Supply monitoring
The firmware SHALL measure the total servo supply current and the servo supply voltage through the Servo 2040's current- and voltage-sense inputs, and evaluate them at least once every 20 ms. The limits SHALL be named, build-time constants: a trip current, a sustained current with its time window, a minimum voltage with its debounce time, a brown-out voltage and a maximum voltage. The limits SHALL be chosen for the servos and supply fitted. A build with no limits configured SHALL NOT compile.

#### Scenario: Readings available for diagnostics
- **WHEN** the firmware is running
- **THEN** the latest current and voltage readings and any latched fault are reported on the debug output at least once per second

### Requirement: Output held off until the supply is valid
After power-on, and whenever servo output has been inhibited, servo output SHALL stay off until the supply voltage has been continuously between the minimum and maximum voltage for 500 ms.

#### Scenario: Powered from USB only
- **WHEN** the board is powered from USB with no servo supply connected
- **THEN** servo output stays off and the absent supply is reported

#### Scenario: Supply connected
- **WHEN** a servo supply within range is connected and held steady for 500 ms
- **THEN** servo output is allowed and the boot sequence proceeds

### Requirement: Overcurrent trip
If the supply current exceeds the trip current, servo output SHALL be inhibited within one 20 ms evaluation period, and an overcurrent fault SHALL be latched.

#### Scenario: Stalled servos draw too much current
- **WHEN** the measured current rises above the trip current
- **THEN** servo output is switched off within 20 ms and the overcurrent fault is latched

### Requirement: Sustained high current
If the supply current averaged over the sustained-current window exceeds the sustained current but stays below the trip current, movement SHALL be put into Sleep, and a sustained-current fault SHALL be latched. Sleep folds the legs and then switches output off.

#### Scenario: Prolonged heavy load
- **WHEN** the average current over the window stays above the sustained current
- **THEN** the body enters Sleep and the sustained-current fault is latched

#### Scenario: Short peak is tolerated
- **WHEN** the current briefly exceeds the sustained current for less than the window, and never exceeds the trip current
- **THEN** no fault is latched

### Requirement: Supply voltage out of range
If the supply voltage stays below the minimum voltage for the debounce time, movement SHALL be put into Sleep and an undervoltage fault SHALL be latched. If the voltage falls below the brown-out voltage, or rises above the maximum voltage, servo output SHALL be inhibited within one evaluation period, and the corresponding fault SHALL be latched.

#### Scenario: Battery running low
- **WHEN** the voltage stays below the minimum voltage for the debounce time
- **THEN** the body enters Sleep and the undervoltage fault is latched

#### Scenario: Transient sag
- **WHEN** the voltage dips below the minimum voltage for less than the debounce time, and never below the brown-out voltage
- **THEN** no fault is latched

#### Scenario: Supply too high
- **WHEN** the voltage rises above the maximum voltage
- **THEN** servo output is switched off within 20 ms and the overvoltage fault is latched

### Requirement: Fault latching and clearing
A latched fault SHALL persist until a clear request is received and the condition that caused it is no longer present. While any fault is latched, every movement command SHALL be treated as a Sleep request, and faults that inhibit output SHALL keep output inhibited. Clearing a fault SHALL NOT move the legs by itself: movement resumes only after a new movement request made since the fault latched. Repeating the request that was in effect when the fault latched does not count. After a fault that only forced Sleep, the body resumes through the normal Sleep wake path. A fault that inhibited output SHALL stop movement altogether, and the body SHALL restart through the boot sequence (Initialise, then Stand) once output is allowed again and a new request has been made.

#### Scenario: Clear while the condition persists
- **WHEN** a clear request arrives while the voltage is still below the minimum
- **THEN** the undervoltage fault stays latched

#### Scenario: Clear after recovery
- **WHEN** the condition has gone and a clear request arrives
- **THEN** the fault is cleared, and the body stays asleep until a new non-Sleep request arrives

#### Scenario: Restart after an output-inhibiting fault
- **WHEN** an overcurrent fault stopped movement, has been cleared, the supply has settled, and a new request arrives
- **THEN** the body restarts from Initialise, then Stand, and then follows the request

#### Scenario: Fault before the supply is first accepted
- **WHEN** an overvoltage fault latches before movement has ever started and is then cleared, with no new request
- **THEN** the body does not start and no servo moves

### Requirement: Fault indication
The board's RGB LEDs SHALL show whether the robot is healthy, waiting for a valid supply, or has a latched fault. Each fault type SHALL have a distinct indication. Every fault SHALL also be reported once on the debug output when it latches, with the reading that caused it.

#### Scenario: Fault shown
- **WHEN** an overcurrent fault latches
- **THEN** the LEDs show the overcurrent indication, and the debug output reports the fault and the measured current
