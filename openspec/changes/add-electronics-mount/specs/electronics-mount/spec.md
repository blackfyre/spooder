# Spec Delta

## Purpose

The printed mount that holds, protects and cools Spooder's electronics on the frame top. It stays clear of the legs, comes open for calibrating and flashing, and prints without support.

## ADDED Requirements

### Requirement: Generated and checked parts
The mount's parts SHALL be generated from named measurements by a build script. The script SHALL check the parts and SHALL write print files only when every check passes. Parts SHALL be exported in the frame top's coordinates, so they load into an assembly with the frame's reference files. Every measurement not taken from a reference file or a measured part SHALL be listed in the documentation as a value to check before printing, with the name of the value to change.

#### Scenario: A check fails
- **WHEN** any check fails
- **THEN** the build reports each failure and writes no print files

#### Scenario: All checks pass
- **WHEN** every check passes
- **THEN** the build writes each distinct part once, as a print file and a CAD file, each in the frame top's coordinates

### Requirement: Fit to the frame and the legs
Above the frame top, every part SHALL stay inside the region the legs never enter: within 78.8 mm of the body's centre, or within the front and rear tabs (|x| ≤ 24 mm, |y| ≤ 79 mm). No part SHALL reach below the frame top's recessed floor. The deck SHALL fasten to the frame top's eight M3 heat-set inserts, with its screw holes on the measured insert positions.

#### Scenario: Leg clearance
- **WHEN** the parts are checked
- **THEN** no point of any part above the frame top lies outside the keep-clear region

#### Scenario: Deck on the inserts
- **WHEN** the deck is checked
- **THEN** each of the eight insert positions has a clear M3 hole through the deck

### Requirement: What the mount holds
The mount SHALL hold the Servo 2040, the T-Display, the SZBK07 converter, one LM2596 module, the XT60 socket, the inline fuse holder and the Schottky diode, each located with clearance and retained so it cannot shift in use. The LM2596 SHALL be screwed down through its mounting holes. The Schottky diode, carried inline on the LM2596's output lead, SHALL be tied down at a tie point on that lead's route. No held part SHALL overlap a printed part or another held part. The cover SHALL enclose every held part except the T-Display's screen and buttons, which SHALL show through the lid. The XT60 socket's mating face SHALL be reachable from outside with the cover on, and SHALL bear the force of plugging and unplugging without moving.

#### Scenario: No collisions
- **WHEN** the parts and the held parts' outlines are checked
- **THEN** no held part overlaps a printed part or another held part, and each lies inside the cover

#### Scenario: Battery plugged in from outside
- **WHEN** the battery's XT60 plug is pushed into the socket and pulled out with the cover on
- **THEN** the socket does not move in its holder

### Requirement: Sensor provisions
The mount SHALL reserve:
- a flat pad with two screw bosses for the LSM6DSO IMU breakout that `add-body-sensing` reads, within 30 mm of the body's centre, with at least 10 mm between the breakout's outline and the SZBK07's outline, and with a route to the Servo 2040's J6 header;
- a pass-through for four wires from the inside of the mount to below the frame top, for a distance sensor under the body;
- an entry for six end-stop signal wires and a shared ground, with a route to the Servo 2040's sensor header.

#### Scenario: IMU pad
- **WHEN** the deck is checked
- **THEN** the IMU pad lies within 30 mm of the body's centre, its breakout outline overlaps no held part, and at least 10 mm separates that outline from the SZBK07's outline

#### Scenario: Pass-through over open frame
- **WHEN** the pass-through is checked against the frame top's model
- **THEN** it opens onto free space below the deck

#### Scenario: End-stop entry
- **WHEN** the end-stop entry is checked
- **THEN** it takes seven wires, and its reserved route reaches the Servo 2040's sensor header from the side the header takes its wires from

### Requirement: Cable routes
Each cable run SHALL have a reserved route that no printed or held part enters, except the Schottky diode, which lies inside the route of the LM2596's output to J6 and nowhere else. That route SHALL be wide enough along its length to carry the diode in its heat-shrink. The runs are:
- XT60 to the fuse and the SZBK07's input;
- the SZBK07's output to the Servo 2040's EXT terminal;
- the fused supply to the LM2596;
- the LM2596's output to J6 (through the Schottky diode) and to the T-Display's pins;
- the T-Display's link wires to J6;
- the 18 servo leads from below the deck to their headers;
- the sensor wires.

Each route SHALL reach its terminal from the side that terminal takes its wires from. Every cable-tie hole and wire slot through the deck SHALL open onto at least 1.5 mm of free space under the deck, measured on the frame top's model.

#### Scenario: Routes kept clear
- **WHEN** the routes are checked
- **THEN** no printed or held part other than the Schottky diode overlaps any reserved route, and the diode lies wholly inside the route of the LM2596's output to J6 and overlaps no other route

#### Scenario: Ties over open frame
- **WHEN** each tie hole and slot is checked against the frame top's model
- **THEN** the frame leaves at least 1.5 mm below the deck along it

### Requirement: Access and service
The cover SHALL be fastened with M3 screws into heat-set inserts. It SHALL lift off once the screws are out, with no wire connecting it to the deck. Every screw SHALL have a straight path for a screwdriver. With the cover off, the Servo 2040's USB-C port and the T-Display SHALL be reachable for calibrating and flashing, and the robot SHALL be able to run.

#### Scenario: Opening the cover
- **WHEN** the cover's screws are removed
- **THEN** the cover lifts straight off, and nothing has to be unplugged

#### Scenario: Running with the cover off
- **WHEN** the cover is off and the battery is plugged in
- **THEN** a USB-C cable plugs into the Servo 2040 without touching any part, the T-Display's screen and buttons are uncovered, and the robot runs

#### Scenario: Screwdriver access
- **WHEN** each screw position is checked
- **THEN** a 4 mm cylinder along the screw's axis, out to the outside, meets no part

### Requirement: Cooling
The cover SHALL have an air inlet low on one side and an outlet high on the opposite side, with the SZBK07 in the path between them. The outlet SHALL take a 30 mm fan blowing outwards, and the inlet SHALL take an optional 30 mm fan blowing inwards. Each fan SHALL sit in a holder on the deck behind its vent, so the cover carries no fan. Both vents SHALL work passively when no fan is fitted: a reserved air channel, whose free cross-section is nowhere smaller than the inlet's net open area, SHALL run from the inlet past the SZBK07's heatsinks to the outlet. The lid SHALL have no openings above the SZBK07, whose heatsinks are live.

#### Scenario: No fans fitted
- **WHEN** the mount is assembled without fans and the air channel is checked
- **THEN** no printed or held part enters the channel from the inlet, past the SZBK07, to the outlet, and the channel's cross-section is nowhere smaller than the inlet's net open area

#### Scenario: Exhaust fan fitted
- **WHEN** a 30 x 30 x 10 mm fan is fitted at the outlet
- **THEN** it seats in its deck holder without colliding with any part, blows out of the cover, and the cover still lifts off with nothing to unplug

#### Scenario: Converter stays cool
- **WHEN** the robot walks for 10 minutes with the cover on, at a room temperature of 20 to 25 °C
- **THEN** the SZBK07's heatsink reads at most 70 °C, with the exhaust fan fitted if it reads more without

#### Scenario: Nothing falls onto the heatsinks
- **WHEN** the lid is checked
- **THEN** it has no opening above the SZBK07's footprint

### Requirement: Protection
Bumpers at the corners SHALL stand proud of the cover's sides and of the lid, so that in a knock or a rollover they meet the ground or obstacle first. The T-Display's screen and buttons SHALL sit below the bumpers' tops. The bumpers SHALL be separate parts that can be replaced without taking the mount apart, and SHALL flex under impact when printed in PETG. The cover is not sealed against water.

#### Scenario: Rollover
- **WHEN** the robot rests upside down on a flat surface
- **THEN** it rests on the bumpers, and the screen and buttons do not touch the surface

#### Scenario: Replacing a bumper
- **WHEN** a bumper is broken
- **THEN** it can be removed and its replacement fitted with the cover in place

### Requirement: Printability
Every part SHALL print in PETG on a 256 x 256 mm bed, in the print orientation the documentation states, without support:
- no face SHALL look down more steeply than 45° unless it is at most 3 mm wide;
- every hole running horizontally as printed SHALL be a teardrop pointing up as printed;
- every part SHALL be a single solid.

#### Scenario: Overhang check
- **WHEN** each part is checked in its print orientation
- **THEN** no face needs support

#### Scenario: Bed size
- **WHEN** each part is checked
- **THEN** it fits within 256 x 256 x 256 mm

### Requirement: Colours
The cover SHALL be produced as a multi-part print file, with flush inlays on the lid's outer face in a second colour. Each other part SHALL be a single colour. The documentation SHALL state each part's colour: the cover black with yellow inlays, the bumpers yellow, the deck black, and the small inside parts white.

#### Scenario: Multi-colour cover
- **WHEN** the cover's print file is opened in the slicer
- **THEN** it holds the body and the inlays as separate parts that the slicer can assign to different filaments, with the inlays flush with the lid's outer face
