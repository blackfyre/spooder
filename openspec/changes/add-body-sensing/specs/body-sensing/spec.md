# Spec Delta

## Purpose

Measures the robot body's distance to the ground and its attitude (pitch and roll), and makes those readings available with guaranteed rates and explicit validity. Movement can later rely on them, and operators can see them.

## ADDED Requirements

### Requirement: Sensor availability
At start-up the firmware SHALL detect each sensor by its identification register. It SHALL probe the LSM6DSO at both of its possible addresses (0x6A and 0x6B) and the VL6180X at 0x29. A sensor that isn't found SHALL be reported unavailable, and the robot SHALL run normally without it. A sensor that fails 5 consecutive reads during operation SHALL be marked unavailable and re-initialised at most once per second until it responds again.

#### Scenario: Sensor not fitted
- **WHEN** the robot starts without a VL6180X connected
- **THEN** the distance reading is reported unavailable and movement and power protection run normally

#### Scenario: Sensor recovers
- **WHEN** a sensor's cable is reconnected after it was marked unavailable
- **THEN** within about 1 second it is re-initialised and its readings become available again

### Requirement: Ground distance
While the VL6180X is available, the firmware SHALL measure the distance to the ground at least 10 times per second, in millimetres. Each reading SHALL carry its status, and readings the sensor reports as invalid or out of range SHALL be marked invalid rather than reported as distances.

#### Scenario: Valid reading
- **WHEN** the sensor faces a surface 80 mm away
- **THEN** a distance of about 80 mm is reported as valid at least 10 times per second

#### Scenario: Nothing in range
- **WHEN** no surface is within the sensor's range
- **THEN** the reading is reported as out of range, not as a distance

### Requirement: Body attitude
While the LSM6DSO is available, the firmware SHALL sample acceleration and rotation rate at least 100 times per second, and estimate body pitch and roll. It SHALL estimate the gyroscope bias at start-up while the robot is stationary, before servo output is first enabled. At rest on a level surface, pitch and roll SHALL read within ±2° of zero, and the estimate SHALL not drift by more than 2° over 10 minutes at rest. The estimate SHALL follow a tilt within 0.5 seconds.

#### Scenario: Level at rest
- **WHEN** the robot stands still on a level surface
- **THEN** pitch and roll read within ±2° of zero

#### Scenario: Tilt followed
- **WHEN** the body is tilted by 10° about its roll axis and held
- **THEN** within 0.5 seconds roll reads within ±2° of 10°

### Requirement: Isolation from movement
Sensor input and output SHALL NOT delay a movement step or a power-protection evaluation. Movement and protection SHALL read the newest available sensor snapshot without waiting. A snapshot SHALL always be internally consistent: no value may be mixed from different sample times.

#### Scenario: Bus stalls
- **WHEN** the sensor I²C bus is held low or a sensor stops responding
- **THEN** movement steps and power-protection evaluations continue every 20 ms, and the affected readings become unavailable

### Requirement: Readings in telemetry
Each telemetry frame SHALL carry the newest ground distance with its status, and the newest pitch and roll. Each SHALL be marked available or unavailable.

#### Scenario: Readings shown
- **WHEN** both sensors are available
- **THEN** telemetry reports the distance, pitch and roll as available, and the bridge screen shows them
