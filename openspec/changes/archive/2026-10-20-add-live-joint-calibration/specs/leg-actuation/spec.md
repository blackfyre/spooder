# Spec Delta

## MODIFIED Requirements

### Requirement: Foot target to joint angles
Each leg SHALL convert a foot target, given in the leg's own frame (origin at the coxa pivot, x outward, z up, millimetres), into coxa, femur and tibia angles using the leg's link lengths (41, 116 and 183 mm) and each joint's calibration offset from the joint calibration table.

#### Scenario: Calibration pose
- **WHEN** a leg is given the target (224, 0, 116) with zero offsets
- **THEN** the coxa angle is 0°, the femur angle is 90° and the tibia angle is 90°, each within 0.5°

#### Scenario: Offset applied
- **WHEN** a leg's femur offset is 5° and the leg is given the calibration-pose target
- **THEN** the femur angle is 95°, within 0.5°

### Requirement: Joint angle to servo pulse
Each joint angle SHALL be clamped to 0–180°, mirrored to 180° minus the angle when the joint's direction is reversed, and converted linearly to a pulse between the joint's calibrated pulses for 0° and 180° (by default 500 µs and 2500 µs). The pulse SHALL be output on that joint's servo channel. The eighteen joints SHALL map to the Servo 2040's servo outputs 1–18 (GPIO0–17) in leg order, legs 1–6, and within each leg in coxa, femur, tibia order. Leg *n*'s coxa is therefore output 3n−2, its femur 3n−1 and its tibia 3n.

#### Scenario: Channel mapping
- **WHEN** leg 5's femur angle is set
- **THEN** the pulse is output on servo output 14 (GPIO13)

#### Scenario: Angle clamping
- **WHEN** a joint angle of 200° is requested with the default calibration
- **THEN** the joint's output pulse is 2500 µs

#### Scenario: Calibrated pulse range
- **WHEN** a joint calibrated to 600 µs at 0° and 2400 µs at 180° is set to 90°
- **THEN** its output pulse is 1500 µs

#### Scenario: Reversed joint
- **WHEN** a reversed joint with the default pulse range is set to 30°
- **THEN** its output pulse is 2167 µs, the pulse for 150°
