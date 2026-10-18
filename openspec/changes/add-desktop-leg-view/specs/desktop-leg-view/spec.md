# Spec Delta

## Purpose

Draws the robot's legs in the desktop calibrator as the robot commands them, so that calibration offsets can be judged by comparing the drawing with the real leg.

## ADDED Requirements

### Requirement: Leg drawing from the reported pose
The calibrator SHALL draw the legs from the latest pose frame, using the reported commanded angles, the link lengths (coxa 41 mm, femur 116 mm, tibia 183 mm) and the robot's angle conventions, so that the drawn foot is where the robot's inverse kinematics placed it. It SHALL show:
- the selected leg from the side, in the leg's vertical plane, with its coxa angle in a top view;
- all six legs from above on a schematic body, with the leg numbers and the assumed front marked.

#### Scenario: Calibration pose
- **WHEN** the robot reports the calibration pose (commanded 0°, 90°, 90°)
- **THEN** the side view shows the coxa straight out, the femur pointing straight up and the tibia horizontal, outward, with the foot 224 mm out and 116 mm above the coxa pivot

#### Scenario: Leg never set
- **WHEN** the pose reports pulse 0 for all three joints of a leg
- **THEN** that leg is shown as not set rather than drawn

### Requirement: Joint details on the drawing
The calibrator SHALL show each joint's pulse width and servo angle next to it on the side view, SHALL mark a joint whose servo angle is within 5° of 0° or 180° as near its limit and one outside 0–180° as at its limit, and SHALL highlight the leg and joint selected in the calibration editor.

#### Scenario: Joint at its limit
- **WHEN** the robot reports a servo angle of 183° for leg 2's femur
- **THEN** that joint is marked as at its limit in the side view and in the top view

#### Scenario: Editor selection
- **WHEN** the operator selects leg 4's tibia in the editor
- **THEN** the side view shows leg 4 with its tibia highlighted

### Requirement: No stale pose
When no pose frame has arrived for 1 second, or none has arrived since connecting, the calibrator SHALL show that there is no pose instead of drawing geometry.

#### Scenario: Pose frames stop
- **WHEN** pose frames stop arriving for more than 1 second
- **THEN** the legs pane shows "no pose" and no leg geometry

### Requirement: Leg view appearance
The leg view SHALL use the calibrator's Blackfyre look in both the light and dark palettes.

#### Scenario: Dark mode
- **WHEN** the system is set to a dark appearance
- **THEN** the leg view is drawn in the Blackfyre dark palette with legible labels
