# Spec Delta

## MODIFIED Requirements

### Requirement: Supply monitoring
The firmware SHALL measure the total servo supply current and the servo supply voltage through the Servo 2040's current- and voltage-sense inputs, and evaluate them at least once every 20 ms. The limits SHALL be named, build-time constants: a trip current with its persistence time, a ceiling current, a sustained current with its time window, a minimum voltage with its debounce time, a brown-out voltage and a maximum voltage. The limits SHALL be chosen for the servos and supply fitted. A build with no limits configured SHALL NOT compile, nor SHALL a build whose sustained, trip and ceiling currents are not in increasing order. In bench simulation, protection SHALL evaluate the modelled supply readings instead (as the `bench-simulation` capability states), and the measured voltage SHALL still be read every tick for the simulation supply fault.

#### Scenario: Readings available for diagnostics
- **WHEN** the firmware is running
- **THEN** the latest current and voltage readings and any latched fault are reported on the debug output at least once per second

#### Scenario: Readings in bench simulation
- **WHEN** the robot is in bench simulation
- **THEN** the debug output reports the modelled current and voltage readings, and protection applies its limits to them

### Requirement: Output held off until the supply is valid
After power-on, and whenever servo output has been inhibited, servo output SHALL stay off until the supply voltage has been continuously between the minimum and maximum voltage for 500 ms. In bench simulation the rule SHALL apply to the modelled supply and the modelled output; the real servo output SHALL stay off whatever the modelled supply reads.

#### Scenario: Powered from USB only
- **WHEN** the board is powered from USB with no servo supply connected and is not in bench simulation
- **THEN** servo output stays off and the absent supply is reported

#### Scenario: Supply connected
- **WHEN** a servo supply within range is connected and held steady for 500 ms
- **THEN** servo output is allowed and the boot sequence proceeds

#### Scenario: Powered from USB only in bench simulation
- **WHEN** the board is powered from USB only and is in bench simulation
- **THEN** the modelled output is allowed 500 ms after entry, and the real servo output stays off
