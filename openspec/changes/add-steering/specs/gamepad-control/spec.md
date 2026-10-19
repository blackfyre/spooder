## ADDED Requirements

### Requirement: Stick modes
The bridge SHALL offer two stick modes, steer and crab, toggled by `b` (one toggle per press) and kept across a controller reconnect like the other settings. Steer SHALL be the default. This requirement supersedes the stick rows of the input mapping table.
- In steer mode, left stick Y SHALL set the forward or backward translate, left stick X SHALL steer, and the translate's sideways part SHALL be zero. While the left stick points backward, steering SHALL be mirrored as a car's is: steering left while reversing turns the body clockwise.
- In crab mode, the left stick SHALL set the translate vector and the right stick X the rotate amount, as before.
- In both modes, right stick X SHALL add to the rotate amount, and the sum SHALL be clamped to −100..100.
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
