## ADDED Requirements

### Requirement: Gesture playback
The robot SHALL play a requested gesture from Stand, in a dedicated Perform mode, and return to Stand when it ends. The gestures SHALL be say hi, wiggle, dance, attack and ball. A gesture request SHALL be acted on only when the body is standing with no transition in progress; otherwise it SHALL be ignored. Each request SHALL play its gesture once.

#### Scenario: Say hi from Stand
- **WHEN** the body stands and say hi is requested
- **THEN** the body shifts its weight, lifts a front leg, waves it, puts it down and returns to Stand

#### Scenario: Request while walking
- **WHEN** the body is walking and a gesture is requested
- **THEN** the request is ignored and walking continues

### Requirement: Ball holds until another request
The ball gesture SHALL lower the body onto the ground and fold the legs in, then hold that pose. Any other mode request SHALL first unfold the legs and stand the body up, then act on the request.

#### Scenario: Leaving the ball
- **WHEN** the body is curled in the ball pose and Walk is requested
- **THEN** the body unfolds into Stand, then walks

### Requirement: Interrupting a gesture
Sleep, a stale command, the emergency stop and power faults SHALL interrupt a gesture at once, exactly as they interrupt walking. Another mode request SHALL end the gesture at its next keyframe and return through Stand.

#### Scenario: Emergency stop mid-gesture
- **WHEN** the robot is playing dance and the emergency stop is signalled
- **THEN** servo output stops at once and the robot reports the fault

### Requirement: Safe poses
Every pose a gesture reaches SHALL be reachable and within each servo's travel at every body height, and gestures that lift legs SHALL keep the body's centre inside the support area of the planted feet.

#### Scenario: Every keyframe checked
- **WHEN** each gesture is played in simulation at the lowest and highest body heights
- **THEN** no target is rejected, no joint leaves its range and the stability margin holds for every lifted-leg keyframe
