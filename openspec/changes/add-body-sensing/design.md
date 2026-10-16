# Design

## Context

- **Depends on `port-movement-to-servo2040`:**
  - core 0 runs the 20 ms loop: power read, protection, `movementTick`, servo output;
  - ServoCluster's DMA IRQ is on core 0;
  - J6 gives GPIO26/27/28. The board has no pull-ups on them (schematic sheet 1).
- **Depends on `add-gamepad-bridge`:** telemetry already has a sensor block, holding an availability bitmask, ToF mm and status, and pitch and roll in centidegrees.
- **GPIO26/27 can be I²C1** (RP2040 pin mux: even pins are SDA, odd are SCL, and the controller is (n/2) mod 2). GPIO28 is free for the IMU interrupt.
- **Drivers:**
  - **VL6180X:** ST's application note sequence. It needs the private register "mandatory settings" load after reset, then single-shot or continuous ranging, and reports a 0–255 mm range with a status register.
  - **LSM6DSO:** the ST platform-independent `lsm6dso-pid` driver (BSD-3, v3.2.2), which needs only read and write register callbacks. Its `WHO_AM_I` is 0x6C.

## Goals / Non-Goals

**Goals:**
- Readings with guaranteed rates and explicit validity.
- No possible effect on core 0 timing.
- Attitude estimation that can be host-tested.

**Non-Goals:**
- Using the readings in movement. That is a later change; it will probably add a `locomotion` requirement for levelling or height hold.
- Magnetometer or yaw estimation.
- Hot-plugging beyond periodic re-initialisation.

## Decisions

### D1. Sensor I/O on core 1

- `multicore_launch_core1(sensorsMain)` starts after core 0 has set up ServoCluster and the power monitor, so no cluster allocation overlaps with core 1 (pimoroni's `pwm_alloc` warning).
- Core 1 owns I²C1 exclusively. Nothing on core 0 touches it.
- **Why core 1:**
  - VL6180X ranging takes milliseconds;
  - I²C errors and timeouts can block for longer;
  - the LSM6DSO needs 100 Hz or more, which is out of step with the 20 ms loop.
  
  Doing this on core 0 would risk movement and protection timing.
- **Alternatives considered:**
  - Interleaving non-blocking I²C on core 0 with a state machine. Rejected: it is more complex, and it shares core 0's time budget.
  - DMA-driven I²C. Rejected: unnecessary at these rates.

### D2. Snapshot handoff

- Core 1 publishes a `SensorSnapshot` struct: availability flags, ToF mm and status, pitch, roll, and the sample time in ms.
- Publishing uses a seqlock. The writer increments a sequence counter, writes, then increments it again, all under `__dmb()` barriers. The reader retries while the counter is odd or changed.
- Core 0 reads it each tick, without waiting. The single-writer, single-reader pattern needs no lock, so neither core can block the other.
- **Rejected alternative: the pico-sdk `queue_t`.** It is fine, but a queue carries a history the robot doesn't want. Only the newest value matters.

### D3. Bus

- I²C1 at 400 kHz, with GPIO26 SDA and GPIO27 SCL.
- Every transfer uses `i2c_*_timeout_us`, so a stuck bus returns an error instead of hanging.
- After 5 consecutive failures, a sensor is marked unavailable. Its re-initialisation is retried at most once per second.
- If SDA is held low at boot or after errors, bus recovery clocks out up to 9 SCL pulses.

### D4. VL6180X

- Probe the model ID at 0x000 (expect 0xB4), then load the mandatory private settings and the recommended public settings.
- Use continuous ranging with a period of about 50 ms (20 Hz, above the required 10 Hz).
- Poll the result-ready register from the core 1 loop.
- Map the range status code to valid, out-of-range or error.

### D5. LSM6DSO

- Probe `WHO_AM_I` at 0x6A, then at 0x6B, and remember the address that answers.
- Configure the accelerometer at 208 Hz ±4 g, and the gyro at 208 Hz ±500 dps.
- The data-ready interrupt on INT1 goes to GPIO28. Core 1 waits on it with a timeout, or falls back to polling if the INT line isn't wired.
- `lsm6dso-pid` is used through read and write callbacks over I²C1. The submodule is `external/lsm6dso-pid` at v3.2.2.

### D6. Attitude estimation (hardware-free, `core/attitude`)

- **Filter:** a complementary filter, `angle = α·(angle + gyro·dt) + (1−α)·accel_angle`, with a time constant of about 0.5 s (α derived from dt).
- **Gyro bias:** averaged over 2 s while stationary, before servo output is first allowed. The power gate's 500 ms wait plus initialisation gives time for this. Readings during the bias window are flagged as not yet available.
- **Stationarity check:** the bias estimate is accepted only if the gyro variance is below a threshold. Otherwise it is retried.
- **Rejected alternative: a Kalman or Madgwick filter.** Unnecessary for pitch and roll at these accuracy targets, and the complementary filter is easier to verify on host.

### D7. Testing

- **Host tests for `core/attitude`:**
  - level and tilted static input giving the right angles within 0.5°;
  - a step tilt followed within 0.5 s;
  - bias removal giving drift under 2° over a simulated 10 minutes, using synthetic gyro bias and noise;
  - stationarity rejection.
- **Seqlock:** host-tested with a single-threaded interleaving simulation, covering a torn read being retried.
- **Hardware checks:**
  - detection with and without each sensor;
  - a cable pull and recovery;
  - ToF against a ruler;
  - level and tilt against a phone inclinometer;
  - the movement tick period unchanged with sensors running, checked with a GPIO toggle on a scope.

## Risks / Trade-offs

- [Breakout pull-ups are unknown and may be too weak or too strong at 400 kHz] → Check rise time during bring-up, and drop to 100 kHz if needed. The rates above have ample margin at 100 kHz.
- [Vibration from walking corrupts the accelerometer's attitude estimate] → The complementary filter weights the gyro over short times, and the accuracy targets apply at rest. Tuning while walking belongs to the later movement-assist change.
- [Core 1 floating point and newlib re-entrancy] → Core 1 uses only `core/attitude` maths and the pico-sdk I²C. It makes no `printf`; debug output goes through a core 0 report of the snapshot.
- [A sensor stuck mid-transfer] → Timeouts, then bus recovery, then periodic re-initialisation.

## Migration Plan

1. Land `core/attitude` with its tests, then the drivers, then the core 1 loop and the snapshot, then the telemetry fill.
2. Bring up one sensor at a time on the bench, then mount both and verify against a ruler and an inclinometer.
3. **Rollback:** a build flag `SPOODER_SENSORS=0` skips starting core 1. Telemetry then reports both sensors unavailable.
