# Spec Delta

## ADDED Requirements

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
