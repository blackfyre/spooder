# Tasks

## 1. Hardware-free core (host-tested)

- [ ] 1.1 Create `core/attitude/` with a complementary filter (dt-derived α, τ ≈ 0.5 s), gyro bias estimation and a stationarity check (design D6). Verify host tests for static level and tilted angles within 0.5°, a 10° step followed within 0.5 s, drift under 2° over a simulated 10 minutes with synthetic bias and noise, and rejection of a non-stationary bias window.
- [ ] 1.2 Create the `SensorSnapshot` struct and a seqlock publish/read helper in `core/`, using portable atomics and barriers behind a small platform shim. Verify host tests for normal publish and read, and for a simulated torn read being retried.

## 2. Drivers on core 1 (Servo 2040)

- [ ] 2.1 Add `external/lsm6dso-pid` (tag v3.2.2) as a submodule. Bring up I²C1 on GPIO26 (SDA) and GPIO27 (SCL) at 400 kHz, with timeout-based transfers and 9-clock bus recovery. Verify the build.
- [ ] 2.2 Implement the VL6180X driver: model-ID probe, mandatory and recommended settings, continuous ranging at about 20 Hz, and status mapping to valid, out-of-range or error. Verify the build.
- [ ] 2.3 Implement the LSM6DSO driver through `lsm6dso-pid`: probe 0x6A then 0x6B, 208 Hz accelerometer and gyro, INT1 data-ready on GPIO28 with a polling fallback. Verify the build.
- [ ] 2.4 Implement `sensorsMain` on core 1: an availability state per sensor (5 failures marks it unavailable, then a 1 s re-init period), the bias window, attitude updates, and snapshot publishing. Launch it after core 0 set-up, behind the `SPOODER_SENSORS` build flag (on by default). Verify the build with the flag on and off.

## 3. Integration

- [ ] 3.1 On core 0, read the snapshot each tick, fill the telemetry sensor block with its availability bits, and report it once per second on the debug output. Core 1 does no I/O other than I²C. Verify `mise run build` and `mise run test` with no warnings.

## 4. Verification and documentation

- [ ] 4.1 Run `mise run test`, `mise run build` and `mise run build-bridge` from a clean checkout. Verify all three pass.
- [ ] 4.2 Bring up the hardware (requires the user at the hardware), recording results in this change:
  - detection with each sensor absent and present;
  - a cable pull and recovery within about 1 s;
  - SDA/SCL rise time at 400 kHz, falling back to 100 kHz if it exceeds 300 ns;
  - ToF against a ruler at 30, 80 and 150 mm;
  - pitch and roll against an inclinometer, level and at 10°;
  - drift over 10 minutes at rest;
  - the movement tick period unchanged with sensors running, using a GPIO toggle on a scope;
  - readings shown on the bridge screen.
- [ ] 4.3 Update `CLAUDE.md` and `spooder-firmware-shape` (core 1 ownership of I²C1, the snapshot boundary, the J6 sensor pins and pull-up requirement) and `spooder-hardware-check` (sensor wiring and checks). Verify that every symbol and path mentioned exists (`grep`).
