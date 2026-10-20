# Spec Delta

## ADDED Requirements

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
