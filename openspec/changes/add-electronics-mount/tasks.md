# Tasks

## 1. Measurements and reference data

- [ ] 1.0 Build from nominal values until the parts are measured: the SZBK07 at 60 × 54 × 28 mm with its terminals on the two short ends, the LM2596 at 45 × 20 × 14 mm with its holes on the diagonal, a 30 × 30 × 10 mm fan with 24 mm hole spacing, and a 20 × 15 mm IMU breakout. Each is a named constant, listed in the README's "Measure before printing" table with its source. Verify the table lists each one.
- [ ] 1.1 Measure the parts (requires the user with calipers):
  - the SZBK07's outline, which ends its terminals are on, and its heatsinks' height;
  - the LM2596's outline, its mounting-hole positions and its pad positions;
  - a 30 × 30 × 10 mm fan's hole spacing and its wire exit;
  - the IMU breakout's outline and holes (LSM6DSO, or the ADXL345 to hand);
  - a servo plug seated on a header pin, with its lead bent back.

  Record them in the README's measurements table. Verify every entry has a value and a source.
- [ ] 1.2 Collect the reference data from the repository's files:
  - from the Servo 2040 drawing: the EXT terminal, the sensor header, J6 and the servo header;
  - from the T-Display STEP model: the bare spots on its underside;
  - from the frame top STL: open space and inserts.

  Verify each value is a named constant with its source in a comment.

## 2. Generator and checks

- [ ] 2.1 Replace `hardware/spooder-electronics-mount/` with a new build script and preview renderer (build123d, numpy, trimesh and matplotlib pinned inline, run with `uv`), wired to `mise run cad` and `mise run cad-preview`. Verify both tasks run on an empty part set.
- [ ] 2.2 Implement the checks:
  - one solid per part;
  - the keep-clear region, nothing below the frame floor, the 256 mm bed;
  - the insert holes;
  - overlaps between parts and held parts;
  - enclosure;
  - reserved routes;
  - frame clearance under ties and slots;
  - overhangs in each print orientation, including teardrops;
  - screwdriver paths.

  The build writes no files when any check fails. Verify each check fails on a deliberately bad shape and passes on a good one.

## 3. Deck and layout

- [ ] 3.1 Lay out the held parts by their connections (design: "Layout follows the wiring"), with the deck fastened to the eight frame inserts, and the retention for each part. The LM2596 is screwed down; the XT60 holder takes the force of plugging and unplugging. Verify the fit, overlap and enclosure checks pass, and send a render.
- [ ] 3.2 Add the reserved cable routes, the tie holes and the servo lead slots. Verify the route and frame-clearance checks pass.
- [ ] 3.3 Add the sensor provisions:
  - the IMU pad with two bosses, within 30 mm of the centre and away from the SZBK07;
  - the 4-wire pass-through to below the frame top;
  - the end-stop wire entry, routed to the sensor header.

  Verify a check for the IMU pad's position and the frame-clearance check pass.

## 4. Cover

- [ ] 4.1 Model the faceted cover: 45° corner faces, a chamfered top edge, recessed wall detail, and the window over the T-Display's screen and buttons, bevelled at 45°. Verify the overhang check passes lid-down and the screen sits below the lid's top.
- [ ] 4.2 Add the fastening: M3 screws into heat-set inserts in the deck's posts, with teardrop holes and straight screwdriver paths. Verify the screwdriver-path and overhang checks pass.
- [ ] 4.3 Add the cooling: an inlet low on one side, an outlet high on the opposite side, the SZBK07 between them, and no lid openings above it. Fit a 30 mm exhaust fan position (blowing out) and an optional 30 mm intake. Verify checks that each fan position seats a fan without collisions, and that the lid has no opening over the SZBK07.
- [ ] 4.4 Add the flush yellow inlays on the lid (the name and accents), and export the cover as a 3MF with the body and the inlays as separate parts. Verify the 3MF opens in Bambu Studio with two assignable parts, and the inlays are flush.

## 5. Bumpers and holders

- [ ] 5.1 Model the rails on the cover's corners and the PETG leaf-spring bumpers that slide onto them. They stand proud of the body and the lid and stay inside the keep-clear region. Verify the keep-clear and overhang checks pass, and a check that the bumpers' tops are above the lid and the screen.
- [ ] 5.2 Model the T-Display holder on the deck: supported on the board's bare underside, located by lips, held down by the lid, and printable in its own orientation. Verify the holder's contact check, the overhang check and the screwdriver-path check pass.
- [ ] 5.3 Model the remaining holders in white: the LM2596 standoffs, the IMU bosses and, if the measurements favour it, the fan holder. Verify all checks pass.

## 6. Documentation

- [ ] 6.1 Write the README: parts and colours, print orientation and settings for each part, the hardware list, the assembly order, the cable routes, cooling, and the measurements to check. Verify every printed file and every constant named in "Measure before printing" appears in it.
- [ ] 6.2 Update the hardware-check skill's power wiring: the T-Display and the Servo 2040 both fed 5 V on their pins from the LM2596 (J6 pin 17 through the Schottky diode), the LM2596's dropout near an empty pack, and the fan supply. Update `CLAUDE.md`'s hardware list and `CHANGELOG.md`. Verify the Mermaid diagram matches the text.

## 7. Verification

- [ ] 7.1 Run `mise run cad` and `mise run cad-preview` from a clean checkout. Verify all checks pass and the images are regenerated. Get a Codex review of physical and assembly problems, and answer every finding.
- [ ] 7.2 Test prints (requires the user at the printer):
  - a bumper on a rail section: it slides, holds and flexes back;
  - a deck post with an insert: it holds an M3 screw at hand torque;
  - a fan in its position.

  Record the results here.
- [ ] 7.3 Fit check on the robot (requires the user): every part seats, the cover lifts off with nothing to unplug, the robot rests on its bumpers upside down with the screen clear, and the SZBK07's temperature after a 10-minute walk, with and without the exhaust fan. Record the results here.
