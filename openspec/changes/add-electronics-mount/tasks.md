# Tasks

## 1. Measurements and reference data

- [x] 1.0 Build from nominal values until the parts are measured: the SZBK07 at 60 × 54 × 28 mm with its terminals on the two short ends, the LM2596 at 45 × 20 × 14 mm with its holes on the diagonal, a 30 × 30 × 10 mm fan with 24 mm hole spacing, a 20 × 15 mm LSM6DSO breakout, and the Schottky diode as a DO-41 body of 5.2 × 2.7 mm in 6 mm heat-shrink. Each is a named constant, listed in the README's "Measure before printing" table with its source. Verify the table lists each one.
- [ ] 1.1 Measure the parts (requires the user with calipers):
  - the SZBK07's outline, which ends its terminals are on, and its heatsinks' height;
  - the LM2596's outline, its mounting-hole positions and its pad positions;
  - a 30 × 30 × 10 mm fan's hole spacing and its wire exit;
  - the LSM6DSO breakout's outline and holes, once it is found (until then its nominal value stays in the table);
  - the Schottky diode's body in its heat-shrink;
  - a servo plug seated on a header pin, with its lead bent back.

  Record them in the README's measurements table. Verify every entry has a value and a source.
- [x] 1.2 Collect the reference data from the repository's files:
  - from the Servo 2040 drawing: the EXT terminal, the sensor header, J6 and the servo header;
  - from the T-Display STEP model: the bare spots on its underside;
  - from the frame top STL: open space and inserts.

  Verify each value is a named constant with its source in a comment.

## 2. Generator and checks

- [x] 2.1 Create `hardware/spooder-electronics-mount/` with a build script and preview renderer (build123d, numpy, trimesh and matplotlib pinned inline, run with `uv`), wired to `mise run cad` and `mise run cad-preview`. Verify both tasks run on an empty part set.
- [x] 2.2 Implement the checks:
  - one solid per part;
  - the keep-clear region, nothing below the frame floor, the 256 mm bed;
  - the insert holes;
  - overlaps between parts and held parts;
  - enclosure;
  - reserved routes, with the Schottky diode allowed only inside the LM2596's output route;
  - frame clearance under ties and slots;
  - overhangs in each print orientation, including teardrops;
  - screwdriver paths;
  - the air channel from the inlet past the SZBK07 to the outlet, clear and with no cross-section smaller than the inlet's net open area.

  The build writes no files when any check fails. Verify each check fails on a deliberately bad shape and passes on a good one.
- [x] 2.3 Implement the export: each distinct part once as STL (the cover as 3MF) and once as STEP, in the frame top's coordinates. Verify a test that counts the files per distinct part, and that re-imports each STEP and finds the deck's holes on the eight insert positions and each part's bounding box equal to the built part's.

## 3. Deck and layout

- [x] 3.1 Lay out the held parts by their connections (design: "Layout follows the wiring"), with the deck fastened to the eight frame inserts, and the retention for each part. The LM2596 is screwed down; the XT60 holder takes the force of plugging and unplugging; the Schottky diode has a tie point on the LM2596's output route. Verify the fit, overlap and enclosure checks pass, and send a render.
- [x] 3.2 Add the reserved cable routes, the tie holes and the servo lead slots. Verify the route and frame-clearance checks pass.
- [x] 3.3 Add the sensor provisions:
  - the IMU pad with two bosses, within 30 mm of the centre and at least 10 mm from the SZBK07's outline;
  - the 4-wire pass-through to below the frame top;
  - the 7-wire end-stop entry, routed to the sensor header from its wire side.

  Verify a check for the IMU pad's position and separation, the route check for the end-stop route, and the frame-clearance check pass.

## 4. Cover

- [x] 4.1 Model the faceted cover: 45° corner faces, a chamfered top edge, recessed wall detail, and the window over the T-Display's screen and buttons, bevelled at 45°. Verify the overhang check passes lid-down and the screen sits below the lid's top.
- [x] 4.2 Add the fastening: M3 screws into heat-set inserts in the deck's posts, with teardrop holes and straight screwdriver paths. Verify the screwdriver-path and overhang checks pass.
- [x] 4.3 Add the cooling: an inlet low on one side, an outlet high on the opposite side, the SZBK07 between them, and no lid openings above it. Fit a 30 mm exhaust fan position (blowing out) and an optional 30 mm intake, each in a holder on the deck behind its vent. Verify checks that each fan position seats a fan without collisions, that the cover carries no fan, that the air channel is clear, and that the lid has no opening over the SZBK07.
- [x] 4.4 Add the flush yellow inlays on the lid (the name and accents), and export the cover as a 3MF with the body and the inlays as separate parts. Verify the 3MF opens in Bambu Studio with two assignable parts, and the inlays are flush.

## 5. Bumpers and holders

- [x] 5.1 Model the rails on the cover's corners and the PETG leaf-spring bumpers that slide onto them. They stand proud of the body and the lid and stay inside the keep-clear region. Verify the keep-clear and overhang checks pass, and a check that the bumpers' tops are above the lid and the screen.
- [x] 5.2 Model the T-Display holder on the deck: supported on the board's bare underside, located by lips, held down by the lid, and printable in its own orientation. Verify the holder's contact check, the overhang check and the screwdriver-path check pass.
- [x] 5.3 Model the remaining holders in white: the LM2596 standoffs, the IMU bosses and the fan holders. Verify all checks pass.

## 6. Documentation

- [x] 6.1 Write the README: parts and colours, print orientation and settings for each part, the hardware list, the assembly order, the cable routes, cooling, and the measurements to check. Verify every printed file and every constant named in "Measure before printing" appears in it.
- [x] 6.2 Update the hardware-check skill's power wiring, which already feeds both boards 5 V on their pins from a logic buck: name the LM2596 as that buck, state its dropout near an empty pack (the power protection watches the servo rail, not the pack, so it does not cover this), and add the fan supply. Update `CLAUDE.md`'s hardware list and `CHANGELOG.md`. Verify the Mermaid diagram matches the text.

## 7. Verification

- [ ] 7.1 Run `mise run cad` and `mise run cad-preview` from a clean checkout. Verify all checks pass, the export test passes, and the images are regenerated. Get a Codex review of physical and assembly problems, and answer every finding.
- [ ] 7.2 Test prints (requires the user at the printer):
  - a bumper on a rail section: it slides, holds and flexes back;
  - a deck post with an insert: it holds an M3 screw at hand torque;
  - a fan in its holder;
  - the XT60 holder with the socket: plugged and unplugged ten times, the socket does not move.

  Record the results here.
- [ ] 7.3 Fit check on the robot (requires the user). Record the results here:
  - every part seats, and the cover lifts off with nothing to unplug;
  - with the cover off, a USB-C cable plugs into the Servo 2040 and the robot runs;
  - a bumper is removed and refitted with the cover in place;
  - the robot rests on its bumpers upside down with the screen and buttons clear;
  - with a bench supply at 6.4 V on the XT60 and the fans running, the 5 V at J6 pin 17 is measured and both boards run;
  - the SZBK07's heatsink, read with a thermocouple or IR thermometer after a 10-minute walk at full speed on a flat floor at 20–25 °C, cover on: at most 70 °C without the exhaust fan, or with it if not.
