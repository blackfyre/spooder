# gamepad-control Specification

## Purpose
How the gamepad bridge turns a Bluetooth gamepad's input into the commands it sends the robot.

## Requirements

### Requirement: Stick modes
The bridge SHALL offer two stick modes, steer and crab, toggled by `b` (one toggle per press) and kept across a controller reconnect like the other settings. Steer SHALL be the default. This requirement supersedes the stick rows of the input mapping table.
- In steer mode, left stick Y SHALL set the forward or backward translate, left stick X SHALL steer, and the translate's sideways part SHALL be zero. While the left stick points backward by more than 10 %, the robot SHALL reverse and steering SHALL be mirrored as a car's is: steering left while reversing turns the body clockwise. Between 0 and 10 % backward there SHALL be no reverse and no mirroring, so that a stick held sideways turns the body on the spot without flipping direction.
- In crab mode, the left stick SHALL set the translate vector and the right stick X the rotate amount, as before.
- In both modes, right stick X SHALL add to the rotate amount, and the sum SHALL be clamped to −100..100. A stick pushed right SHALL turn the body clockwise, and left anticlockwise.
- The status screen SHALL show the stick mode.

#### Scenario: Steering while walking
- **WHEN** steer mode is on and the left stick points forward and to the left
- **THEN** the command carries a forward translate with no sideways part and a rotate amount that turns the body anticlockwise

#### Scenario: Steering while reversing
- **WHEN** steer mode is on and the left stick points backward and to the left
- **THEN** the command carries a backward translate and a rotate amount that turns the body clockwise

#### Scenario: Crab mode keeps the old layout
- **WHEN** crab mode is on and the left stick points to the right
- **THEN** the command carries a sideways translate and no rotation
