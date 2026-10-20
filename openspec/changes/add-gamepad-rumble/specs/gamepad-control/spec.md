## ADDED Requirements

### Requirement: Rumble feedback
The bridge SHALL rumble the connected gamepad when a protection fault newly latches on the robot, when its own emergency stop switches on or off, when the robot link is lost or regained while a gamepad is connected, and when a gesture request is not acted on within 0.6 s, with a distinct pattern per event. Faults and stops present when the gamepad connects SHALL NOT rumble. A more important event SHALL replace a lighter rumble in progress, a lighter event during a more important rumble SHALL play after it, and the same event SHALL NOT repeat within 1 s.

#### Scenario: Fault while driving
- **WHEN** the robot latches an overcurrent fault while the gamepad drives it
- **THEN** the gamepad rumbles long and strong once

#### Scenario: Existing fault on connect
- **WHEN** a gamepad connects while a fault is already latched
- **THEN** the gamepad does not rumble for it

#### Scenario: Refused gesture
- **WHEN** a gesture is requested while the robot walks
- **THEN** the gamepad gives a light tap about 0.6 s later
