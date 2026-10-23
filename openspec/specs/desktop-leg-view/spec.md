# desktop-leg-view Specification

## Purpose
Draws the robot's legs in the desktop calibrator as the robot commands them, so that calibration offsets can be judged by comparing the drawing with the real leg.

## Requirements

### Requirement: Leg drawing from the reported pose
The calibrator SHALL draw the legs from the latest pose frame, using the reported commanded angles, the link lengths (coxa 41 mm, femur 116 mm, tibia 183 mm) and the robot's angle conventions, so that the drawn foot is where the robot's inverse kinematics placed it. It SHALL show:
- the selected leg from the side, in the leg's vertical plane, with its coxa angle in a top view;
- all six legs from above on a schematic body, with the leg numbers and the front (legs 1 and 6) marked.

#### Scenario: Calibration pose
- **WHEN** the robot reports the calibration pose (commanded 0°, 90°, 90°)
- **THEN** the side view shows the coxa straight out, the femur pointing straight up and the tibia horizontal, outward, with the foot 224 mm out and 116 mm above the coxa pivot

#### Scenario: Leg never set
- **WHEN** the pose reports pulse 0 for all three joints of a leg
- **THEN** that leg is shown as not set rather than drawn

### Requirement: Joint details on the drawing
The calibrator SHALL show each joint's pulse width next to it on the side view, and the selected leg's commanded angles, servo angles and pulses in a list beside the drawings. It SHALL mark a joint whose servo angle is within 5° of 0° or 180° as near its limit and one outside 0–180° as at its limit, and SHALL highlight the leg and joint selected in the calibration editor. Legs SHALL be numbered as in the editor.

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

### Requirement: Four-view window
The calibrator SHALL offer, from its top bar, a separate window that draws the whole robot from the latest pose frame in four views:
- top, with the front up;
- front, seen from ahead, so the robot's left is on the right;
- left, seen from the robot's left, with the front to the right;
- isometric, from the front left and above.

Each view SHALL draw the schematic body as a plate, every leg from its coxa pivot through the coxa end and the knee to the foot, and the ground at the lowest set foot when that foot is below the body (in the front, left and isometric views). The geometry SHALL be that of the leg drawing: the reported commanded angles, the link lengths and the legs' mounts. The three orthographic views SHALL share one scale, and the isometric view SHALL draw nearer legs over farther ones. The selected leg SHALL be highlighted, joints near or at their servo limits SHALL be ringed as in the leg drawing, and the legs SHALL be numbered as in the editor.

The window SHALL show why there is no pose under the same conditions as the leg drawing. It SHALL follow new pose frames without input from the main window. Opening it while it is open SHALL bring nothing new up. Closing it SHALL leave the main window running, and closing the main window SHALL close it.

#### Scenario: Calibration pose
- **WHEN** the robot reports the calibration pose (coxa 0°, femur 90°, tibia 90° on every leg)
- **THEN** the front and left views show each femur vertical and each tibia horizontal, outward from its knee, and the top view shows every leg straight out from its mount

#### Scenario: Standing
- **WHEN** the robot stands with every foot at the same height
- **THEN** the front, left and isometric views show all six feet on the ground line

#### Scenario: Shared scale
- **WHEN** the window is resized
- **THEN** a 100 mm segment has the same length in the top, front and left views

#### Scenario: No pose
- **WHEN** no pose frame has arrived for 1 second
- **THEN** the window says so instead of drawing geometry

#### Scenario: Opened twice
- **WHEN** the operator presses the top bar's button while the window is open
- **THEN** no second window opens

### Requirement: Estimated centre of gravity
The four-view window SHALL show the robot's estimated centre of gravity for the latest pose: a static estimate from point masses for the body, the battery when fitted, and each leg's servos and links, placed by the same geometry as the drawing. It SHALL:
- mark the centre of gravity in every view, and in the front, left and isometric views drop a line from it to the ground when the ground is drawn;
- draw, in the top view, the support polygon: the convex hull of the feet within 10 mm of the lowest foot, when there are at least three;
- show the centre's position in the body frame, the estimated total mass and the support margin, the distance of the centre's ground projection inside the polygon, marked as fine from 20 mm, as a warning from 0 to 20 mm and as a danger outside;
- offer a control that fits or removes the battery (2S 5000 mAh) in the centre body, with the battery removed whenever the window opens.

The estimate SHALL match the robot's hardware-free model (`core/movement/centre_of_gravity`), pinned by shared golden vectors. With a leg never set, the window SHALL show no estimate.

#### Scenario: Symmetric stand
- **WHEN** the robot stands with every foot at the same height and distance from its pivot
- **THEN** the centre of gravity is marked on the body's centre line, and the margin is shown as fine

#### Scenario: Lifted leg
- **WHEN** one front leg is raised and the other five feet stay on the ground
- **THEN** the support polygon has five corners, and the centre of gravity is marked where the raised leg's mass has moved it

#### Scenario: Battery fitted
- **WHEN** the operator fits the battery
- **THEN** the shown mass rises by the battery's mass and the centre of gravity moves towards the battery's position

#### Scenario: Too few feet down
- **WHEN** fewer than three feet are within 10 mm of the lowest
- **THEN** no support polygon is drawn and the margin is shown as unavailable
