## ADDED Requirements

### Requirement: Gesture layer
While L2 is held, the bridge SHALL map `a`, `b`, `x`, `y` and R2 to gesture requests (say hi, wiggle, dance, attack and ball, in that order), one request per press, and SHALL suppress those buttons' normal functions. Releasing L2 SHALL restore them.

#### Scenario: Wiggle on the layer
- **WHEN** L2 is held and `b` is pressed
- **THEN** one wiggle request is sent and the stick mode is not toggled
