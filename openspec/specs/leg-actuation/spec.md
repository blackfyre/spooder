# leg-actuation Specification

## Purpose
Defines how a foot target for one leg becomes servo output: the inverse kinematics, which targets are accepted, and how servo output is switched on and off.

## Requirements

### Requirement: Foot target to joint angles
Each leg SHALL convert a foot target, given in the leg's own frame (origin at the coxa pivot, x outward, z up, millimetres), into coxa, femur and tibia angles using the leg's link lengths (41, 116 and 183 mm) and each joint's calibration offset from the joint calibration table.

#### Scenario: Calibration pose
- **WHEN** a leg is given the target (224, 0, 116) with zero offsets
- **THEN** the coxa angle is 0°, the femur angle is 90° and the tibia angle is 90°, each within 0.5°

#### Scenario: Offset applied
- **WHEN** a leg's femur offset is 5° and the leg is given the calibration-pose target
- **THEN** the femur angle is 95°, within 0.5°

### Requirement: Target validation
A leg SHALL reject a foot target that is not finite, or that lies further from the femur joint than the femur and tibia reach (299 mm) less a 4 mm margin, where the knee would be nearly straight and the joint angles change without bound. A rejected target SHALL NOT change the leg's recorded position and SHALL NOT produce servo output. Rejections SHALL be reported on the debug output without repeating the same report on every step.

#### Scenario: Unreachable target
- **WHEN** a leg is given a target 360 mm from its coxa pivot
- **THEN** the leg keeps its previous recorded position and its servos receive no new pulse

#### Scenario: Nearly straight knee
- **WHEN** a leg is given the target (200, 0, −250), 296 mm from its femur joint and within 340 mm of its coxa pivot
- **THEN** the target is rejected in the same way as an unreachable one

#### Scenario: Non-finite target
- **WHEN** a leg is given a target containing NaN
- **THEN** the target is rejected in the same way as an unreachable one

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

### Requirement: Servo profile
The firmware SHALL be built for one servo profile, chosen by name at build time. The default SHALL be the MG996R. A profile SHALL record the servo's rated no-load speed at its nominal supply voltage and the share of that speed movement may use under load. The joint speed budget is the product of the two. Building for a profile name that is not in the table SHALL fail. The robot SHALL print the profile name and its budget on the USB console on its first control tick. The virtual robot SHALL be built for a profile in the same way.

#### Scenario: Default profile
- **WHEN** the firmware is built without choosing a profile
- **THEN** it is built for the MG996R, and the start-up console names the MG996R and its budget

#### Scenario: Unknown profile
- **WHEN** the firmware is built for a profile name that is not in the table
- **THEN** the build fails and names the known profiles

#### Scenario: Another profile
- **WHEN** a second profile with a different budget is added to the table and chosen at build time
- **THEN** movement and the output rate backstop use that profile's budget, and no other source change is needed

### Requirement: Output rate backstop
Between consecutive control ticks, the servo output SHALL change no joint's commanded servo angle by more than the servo profile's rated no-load speed multiplied by the elapsed time. Initialise's poses, step changes by design, SHALL pass unlimited. A joint held back SHALL continue towards its latest commanded angle on the following ticks. This limit SHALL NOT apply to the first pulse written to a joint after servo output is switched on, because the servo's position is unknown then. Each tick in which the limit holds back at least one joint SHALL be counted. When the count has risen since the last status report, the robot SHALL print a separate line on the USB console with the count and the joint it held back most. The existing status line SHALL be unchanged.

#### Scenario: Normal movement does not engage the backstop
- **WHEN** the body walks in any gait, steering or not, at a speed dial up to 44 and any body height, then stands and sleeps
- **THEN** the backstop holds back no joint and the count stays at zero

#### Scenario: Fast command is held back
- **WHEN** a joint's commanded angle jumps by 25° in one tick under a rated speed of 325°/s
- **THEN** the joint's written angle moves by at most 6.5° in that 20 ms tick, reaches the commanded angle over the following ticks, and the count rises

#### Scenario: Output switched on
- **WHEN** servo output is switched on after being off and a joint receives its first pulse
- **THEN** that pulse is written at the commanded angle without the limit

#### Scenario: Reported on the console
- **WHEN** the count has risen since the last status report
- **THEN** a printable-ASCII line on the USB console names the count and the leg and joint that was held back most

### Requirement: Feet fitted to the servo travel
Outside Calibrate, a leg SHALL move each accepted foot target, as little as needed, so that every joint's servo angle (IK angle plus calibration offset) lies within 0–180°, and SHALL record the moved foot. The direction of the foot SHALL be brought into the coxa's travel; its distance from the femur joint into the distances the tibia's travel allows, along the same elevation; and its elevation about the femur joint into the femur's travel, at that distance. A target within travel SHALL be solved unchanged. The fit SHALL be continuous in the target, so a path crossing a limit follows it instead of jumping. In Calibrate the joints SHALL be solved unfitted, so each shows its calibration as it is.

#### Scenario: Lift past the femur's travel
- **WHEN** a leg with a femur offset of 89° is given targets rising straight up in 1 mm steps from (190, 0, −60)
- **THEN** every solved angle lies within 0–180°, the femur holds at 180° where the unfitted angle would exceed it, and the fitted foot moves less than 2 mm per step

#### Scenario: Target within travel
- **WHEN** a target's unfitted angles all lie within 0–180°
- **THEN** the target is unchanged and its angles equal the unfitted ones

#### Scenario: Every pose within travel with the compiled defaults
- **WHEN** the body boots, stands, walks in every gait at the speed dial 0, 44 or 100, height 0, 50 or 100, with and without steering, then stands and sleeps
- **THEN** no joint's angle leaves 0–180° in any step

#### Scenario: Calibrate is not fitted
- **WHEN** the body is in Calibrate
- **THEN** the joint angles are the unfitted IK angles plus offsets
