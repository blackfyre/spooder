# Spec Delta

## MODIFIED Requirements

### Requirement: Joint angle to servo pulse
Each joint angle SHALL be converted to a servo pulse of 500 µs at 0° to 2500 µs at 180°, linearly, and output on that joint's servo channel. Angles outside 0–180° SHALL be clamped. The eighteen joints SHALL map to the Servo 2040's servo outputs 1–18 (GPIO0–17) in leg order, legs 1–6, and within each leg in coxa, femur, tibia order. Leg *n*'s coxa is therefore output 3n−2, its femur 3n−1 and its tibia 3n.

#### Scenario: Channel mapping
- **WHEN** leg 5's femur angle is set
- **THEN** the pulse is output on servo output 14 (GPIO13)

#### Scenario: Angle clamping
- **WHEN** a joint angle of 200° is requested
- **THEN** the joint's output pulse is 2500 µs

### Requirement: Servo frame rate
All eighteen servo outputs SHALL generate pulses at a 50 Hz frame rate (20 ms period), so that every pulse in the 500–2500 µs range fits within one frame. The output pulse width SHALL be within 2 % of the requested width, with timing derived from the board's crystal and no per-board oscillator calibration. Pulses for all joints written in one movement step SHALL take effect together, in the same frame.

#### Scenario: Driver start-up
- **WHEN** servo output is initialised
- **THEN** all eighteen outputs run at 50 Hz before any pulse is produced

#### Scenario: Longest pulse fits the frame
- **WHEN** a 2500 µs pulse is requested
- **THEN** the output produces a pulse of 2450–2550 µs once per 20 ms frame

#### Scenario: One step, one frame
- **WHEN** a movement step changes the angles of several joints
- **THEN** all of the new pulses first appear in the same frame

### Requirement: Servo output enable and disable
Servo output SHALL be switchable off for all eighteen outputs together, leaving the joints unpowered. While output is off, a new accepted foot target SHALL switch output back on before its pulses are produced, unless power protection is inhibiting output. While output is inhibited, output SHALL stay off whatever targets are accepted.

#### Scenario: Disable outputs
- **WHEN** servo output is switched off
- **THEN** no servo output produces a pulse

#### Scenario: Implicit wake
- **WHEN** servo output is off, power protection is not inhibiting output, and a leg accepts a new foot target
- **THEN** output is switched on and the leg's pulses are produced

#### Scenario: Inhibited output stays off
- **WHEN** power protection is inhibiting output and a leg accepts a new foot target
- **THEN** no servo output produces a pulse
