# Proposal

## Why

The long-term goal is for a VL6180X distance sensor and an LSM6DSO IMU to help movement: ground distance for foot placement and body height, and body attitude for levelling. Before any behaviour can depend on them, the robot needs reliable, rate-guaranteed readings. A missing or failing sensor must never disturb the 20 ms movement loop or the power protection.

## What Changes

- Connect the VL6180X and the LSM6DSO to the Servo 2040's I²C1 on GPIO26 (SDA) and GPIO27 (SCL), through the J6 header, with the LSM6DSO's interrupt on GPIO28.
- Read the ground distance continuously from the VL6180X, and acceleration and rotation from the LSM6DSO. Estimate body pitch and roll, with gyro bias calibrated at boot.
- Run sensor I/O on the RP2040's second core, and publish a snapshot to core 0. Sensor stalls or bus errors cannot delay movement or protection.
- Detect missing or failing sensors and report each as unavailable. The robot keeps running without them.
- Fill the reserved sensor fields of the telemetry frame, which the bridge screen already displays once available.
- No movement behaviour uses the readings yet. That is a later change.

## Capabilities

### New Capabilities

- `body-sensing`: Acquisition of ground distance and body attitude: the sensor bus, rates, availability and failure handling, attitude estimation, and isolation from the movement loop.

### Modified Capabilities

None. The telemetry fields were reserved by `add-gamepad-bridge`'s `controller-link`, and this change only fills them.

## Impact

- **Depends on:**
  - `port-movement-to-servo2040`, for the platform, the core 0 loop and the J6 allocation;
  - `add-gamepad-bridge`, for the telemetry sensor block.
- **Code:**
  - New `firmware/servo2040/src/sensors/` for the I²C1 bus, the VL6180X and LSM6DSO drivers, and the core 1 loop.
  - New `core/attitude/` for the hardware-free complementary filter and bias calibration, with host tests.
  - The ST `lsm6dso-pid` driver (BSD-3) is added as a submodule at v3.2.2.
- **Hardware:**
  - Sensor breakouts are soldered to J6: pin 10 GPIO26 SDA, pin 9 GPIO27 SCL, pin 8 GPIO28 INT, pin 15 3V3, pin 16 GND.
  - The breakouts must provide I²C pull-ups, or 4.7 kΩ resistors to 3V3 are fitted, because the board has none on these pins.
  - The LSM6DSO's address depends on the breakout's SA0 strap (0x6A or 0x6B), and the firmware probes both.
- **Concurrency:** introduces the first core 1 use, and the first data shared between cores.
