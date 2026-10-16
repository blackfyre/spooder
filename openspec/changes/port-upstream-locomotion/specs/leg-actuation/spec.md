# Spec Delta

## Purpose

Defines how a foot target for one leg becomes servo output: the inverse kinematics, which targets are accepted, and how servo output is switched on and off.

## ADDED Requirements

### Requirement: Foot target to joint angles
Each leg SHALL convert a foot target, given in the leg's own frame (origin at the coxa pivot, x outward, z up, millimetres), into coxa, femur and tibia angles using the leg's link lengths (41, 116 and 183 mm) and that leg's calibration offsets.

#### Scenario: Calibration pose
- **WHEN** a leg is given the target (224, 0, 116) with zero offsets
- **THEN** the coxa angle is 0°, the femur angle is 90° and the tibia angle is 90°, each within 0.5°

### Requirement: Target validation
A leg SHALL reject a foot target that is not finite or that lies further than 340 mm from the coxa pivot. A rejected target SHALL NOT change the leg's recorded position and SHALL NOT produce servo output. Rejections SHALL be reported on the debug output without repeating the same report on every step.

#### Scenario: Unreachable target
- **WHEN** a leg is given a target 360 mm from its coxa pivot
- **THEN** the leg keeps its previous recorded position and its servos receive no new pulse

#### Scenario: Non-finite target
- **WHEN** a leg is given a target containing NaN
- **THEN** the target is rejected in the same way as an unreachable one

### Requirement: Joint angle to servo pulse
Each joint angle SHALL be converted to a servo pulse of 500 µs at 0° to 2500 µs at 180°, linearly, and written to that joint's channel. Legs 1–3 SHALL use the PCA9685 at I²C address 0x40, channels 0–8, and legs 4–6 the PCA9685 at 0x41, channels 0–8, in coxa, femur, tibia order. Angles outside 0–180° SHALL be clamped.

#### Scenario: Channel mapping
- **WHEN** leg 5's femur angle is set
- **THEN** the pulse is written to channel 4 of the driver at 0x41

### Requirement: Servo frame rate
Both servo drivers SHALL generate pulses at a 50 Hz frame rate (20 ms period), so that every pulse in the 500–2500 µs range fits within one frame. The output pulse width SHALL be within 2 % of the requested width.

#### Scenario: Driver start-up
- **WHEN** the servo drivers are initialised
- **THEN** both run at 50 Hz before any pulse is written

#### Scenario: Longest pulse fits the frame
- **WHEN** a 2500 µs pulse is requested
- **THEN** the channel outputs a pulse of 2450–2550 µs once per 20 ms frame

### Requirement: Servo output enable and disable
Servo output SHALL be switchable off for all eighteen channels together, leaving the joints unpowered. While output is off, a new accepted foot target SHALL switch output back on before its pulses are written.

#### Scenario: Disable outputs
- **WHEN** servo output is switched off
- **THEN** no channel on either driver produces a pulse

#### Scenario: Implicit wake
- **WHEN** servo output is off and a leg accepts a new foot target
- **THEN** output is switched on and the leg's pulses are written
