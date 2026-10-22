# Spec Delta

## MODIFIED Requirements

### Requirement: Supply monitoring
The firmware SHALL measure the total servo supply current and the servo supply voltage through the Servo 2040's current- and voltage-sense inputs, and evaluate them at least once every 20 ms. The limits SHALL be named, build-time constants: a trip current with its persistence time, a ceiling current, a sustained current with its time window, a minimum voltage with its debounce time, a brown-out voltage and a maximum voltage. The limits SHALL be chosen for the servos and supply fitted. A build with no limits configured SHALL NOT compile, nor SHALL a build whose sustained, trip and ceiling currents are not in increasing order.

#### Scenario: Readings available for diagnostics
- **WHEN** the firmware is running
- **THEN** the latest current and voltage readings and any latched fault are reported on the debug output at least once per second

### Requirement: Overcurrent trip
If every reading of the supply current has been above the trip current for at least the persistence time, counted from the first reading of that unbroken run, servo output SHALL be inhibited on that evaluation, and an overcurrent fault SHALL be latched. A reading at or below the trip current SHALL end the run. If a single reading exceeds the ceiling current, servo output SHALL be inhibited within one 20 ms evaluation period, and the overcurrent fault SHALL be latched. For clearing, the overcurrent condition is present while the current is above the trip current.

#### Scenario: Stalled servos draw too much current
- **WHEN** the measured current stays above the trip current, below the ceiling, for the persistence time
- **THEN** servo output is switched off on the evaluation that completes the persistence time and the overcurrent fault is latched

#### Scenario: Start-up surge is tolerated
- **WHEN** the current rises above the trip current, below the ceiling, for less than the persistence time, as when all the servos start together
- **THEN** no fault is latched and output stays on

#### Scenario: Current above the ceiling
- **WHEN** a single reading is above the ceiling current
- **THEN** servo output is switched off within 20 ms and the overcurrent fault is latched
