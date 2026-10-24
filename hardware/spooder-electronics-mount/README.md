# Spooder electronics mount

Spooder's own printed mount for its electronics on the frame top: a deck screwed to the frame's inserts, a cover, corner bumpers and small holders. It is generated as code and checked before export (the `spooder-printed-parts` skill); the OpenSpec change `add-electronics-mount` describes it.

The sizes are nominal until the bought parts are measured: read "Measure before printing" below before printing anything.

| File | What it is |
|---|---|
| `measurements.py` | every size, as a named constant with its source in a comment |
| `reference_check.py` | re-derives the reference data in `measurements.py` from the frame top STL, the T-Display STEP model and the Servo 2040 drawing, and fails on any mismatch |
| `build.py` | generates the parts, runs every check, and only when all pass writes the print and CAD files, re-reads them, and only then moves them into `print/` and `step/` |
| `layout.py` | the layout: where each held part sits, the deck with its stops, standoffs, the XT60's cradle, the cover's posts and the holders' pockets, the spacers under it, the cover with the bumpers' rails, the bumpers, the loose holders (the LM2596's standoffs, the IMU's pad, the fans' holders, the T-Display's holder), the cable routes with the terminals they reach, the slots, tie holes, wire entry and pass-through, the vents, the fan positions, the air channel and the screws |
| `model.py` | the part set's model (printed parts, held parts, cable routes and their terminals, deck openings, screws, the air channel, vents, fan positions, the IMU's pad) and geometry helpers such as `box()`, `turned()` and `teardrop()` |
| `checks.py` | the checks, each rule's number named with its source |
| `export.py` | the export, and the re-reading of what it wrote |
| `preview.py` | renders the assembly on the frame top to `images/`: open (`assembly.png`, `top.png`) and closed (`closed.png`) |
| `test_build.py` | shows each check failing on a bad shape and passing on a good one, tests the export, and tests the layout as built |
| `test_measurements.py` | checks that every constant has a source and that the table below lists each nominal value |

The scripts pin their dependencies inline (build123d, numpy, trimesh, matplotlib and, for the reference check, pillow), locked in `<script>.lock` by `uv lock --script`:

```sh
mise run cad          # reference check, unit tests, then build.py: check and write
mise run cad-preview  # images/assembly.png, images/top.png and images/closed.png
uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --check  # check, write nothing
```

## Parts and colours

Everything prints in PETG. Each file holds one copy of its part, in the robot's orientation (the frame top's coordinates): turn it to its print orientation in the slicer. `test_build.py` checks this table against the built part set.

| File | Part | Copies | Colour | Print orientation |
|---|---|---|---|---|
| `print/deck.stl` | the deck: plate, tabs to the ring's inserts, the Servo 2040's standoffs, the SZBK07's stops, the XT60's cradle, the cover's posts and the holders' pockets | 1 | black | flat, its underside on the bed, as in the file |
| `print/spacer.stl` | a spacer under the deck at a floor insert | 4 | white | on its end, as in the file |
| `print/cover.3mf` | the cover, with the inlays as its second part | 1 | black, inlays yellow | lid-down: turn it over |
| `print/bumper.stl` | a corner guard (bumper) | 4 | yellow | on its top: turn it over, the slots' sloped ends facing up |
| `print/lm2596-standoff.stl` | a standoff under the LM2596 | 2 | white | on its end, as in the file |
| `print/imu-pad.stl` | the LSM6DSO's pad with its two bosses | 1 | white | flat, as in the file |
| `print/intake-fan-holder.stl` | the intake fan's holder | 1 | white | standing, its foot on the bed, as in the file |
| `print/exhaust-fan-holder-left.stl` | the exhaust fan's left column and cheek | 1 | white | standing, its foot on the bed, as in the file |
| `print/exhaust-fan-holder-right.stl` | the exhaust fan's right column and cheek | 1 | white | standing, its foot on the bed, as in the file |
| `print/t-display-holder.stl` | the T-Display's holder, a portal over the LM2596 | 1 | white | on its back: its rear face (−y) on the bed |

The CAD files are `step/<part>.step`, the same names. The overhang check uses each part's print orientation above; the cover's 3MF is one object of two parts, so assign the yellow filament to the inlays in the slicer.

## Printing

Starting settings for a Bambu P1P with an AMS and a 0.4 mm nozzle (assumed: not yet tried on test prints, task 7.2):
- **Every part:** PETG at the filament's own profile, 0.2 mm layers (the cover's inlays are 0.6 mm, three layers), **no supports**: every part passes the 45° overhang check in its orientation, and its horizontal holes are teardrops. A smooth or textured PEI plate with a glue stick as a release agent for PETG.
- **Deck:** 3 walls, 25 % infill. The cover's posts are 2 mm walls round their insert holes, so they print solid.
- **Cover:** 3 walls, 15 % infill, a 5 mm brim against warping (tall PETG walls on an open frame), and no draughts round the printer. The inlays print in the first three layers, so the purge is small; set the prime tower's volume to the slicer's minimum.
- **Bumpers:** 3 walls and 0 % infill, with 4 top and bottom layers, so each prints as a hollow shell that gives in a knock. Print the four at once.
- **Holders and spacers:** 3 walls, 30 % infill.

## Hardware

Besides the electronics themselves (the SZBK07, one LM2596 module, the XT60 socket, the inline 15 A ATO fuse holder, a 1N5819 Schottky diode in 6 mm heat-shrink, the Servo 2040, the T-Display, and later the LSM6DSO breakout):

| Item | Count | Where |
|---|---|---|
| M3 heat-set insert, Ø4.4 hole × 5 mm | 3 | the cover's posts on the deck (the frame top has its own eight) |
| M3 × 6 button head (ISO 7380) | 4 | the deck to the ring's inserts, in counterbores |
| M3 × 10 button head | 4 | the deck through the spacers to the floor's inserts |
| M3 × 10 button head | 3 | the cover, down through the lid into the posts' inserts |
| M2.5 × 6 self-tapping screw for plastics | 4 | the Servo 2040 onto the deck's standoffs |
| M2.5 × 6 self-tapping screw for plastics | 2 | the LM2596 onto its standoffs (its 3 mm holes do not pass an M3) |
| M2 × 5 self-tapping screw for plastics | 2 | the LSM6DSO breakout onto its pad's bosses, when fitted |
| M2 × 6 self-tapping screw for plastics | 4 | each bumper's screwed post into its rail |
| cable tie, at most 3 mm wide and 1.8 mm thick | 4 | the diode, the XT60 and the fuse holder (two), through the deck's tie holes (3.4 × 2.0 mm) |
| 3010 fan, 5 V, 2-pin (optional) | up to 2 | the exhaust position (blowing out) and the intake position (blowing in) |

The screw lengths follow from the stack: the deck's counterbores leave 1 mm of plate under each head, the floor's spacers are 5.75 mm, the lid is 2.4 mm over a 5 mm insert, and each self-tapping screw passes a 1.6 mm board into a pilot 5 mm deep (the Servo 2040's standoffs), 4 mm deep (the IMU's bosses) or down the whole standoff (the LM2596's), so no screw is longer than its pilot. Check them against the measured parts.

## Assembly order

The order follows what covers what: the floor screws lie under the SZBK07 and the board, the loose holders are kept in their pockets by what goes over them, and the LM2596 is adjusted before its holder covers it.

1. Press the three M3 inserts into the deck's cover posts with a soldering iron.
2. Set the SZBK07 to 6.0 V and the LM2596 to 5.0 V on the bench (the `spooder-hardware-check` skill), and solder the harness: the XT60, the fuse holder and the leads to the SZBK07's input and the LM2596's input; the LM2596's output leads to J6 pin 17 (through the diode in its heat-shrink) and to the T-Display's 5V pin, and their ground to J6; the fan leads, if fitted, to the LM2596's output pads.
3. Stand the four spacers on the frame top's floor inserts, lay the deck on the ring and the spacers, and drive the eight deck screws: M3 × 6 at the ring, M3 × 10 through the spacers.
4. Drop the loose holders into their pockets: the IMU's pad, the LM2596's two standoffs, the intake fan's holder and the exhaust fan's two holders.
5. Screw the LSM6DSO breakout onto its pad (when fitted), with its wires out of its rear edge towards J6.
6. Seat the SZBK07 between its blocks and ribs, its input end left.
7. Screw the Servo 2040 onto its standoffs over the IMU, the servo leads down through the two lead slots in front of it.
8. Push the XT60 into its cradle against its back stops and tie it down; tie the fuse holder down at both ends.
9. Screw the LM2596 onto its standoffs, and tie the diode down through its pair of holes in front of the module.
10. Lay the wires in their routes (below) and connect the screw terminals and J6.
11. Fit the T-Display's holder over the LM2596 and lay the T-Display on its pads between the lips, screen up.
12. Fit the fans, if any, into their holders, the exhaust blowing out and the intake blowing in.
13. Lower the cover straight down onto the deck's rim and drive its three M3 × 10 screws through the lid.
14. Slide each bumper down onto its corner's rails until its slots stop on the wedges, and drive its M2 × 6 screw through the post with the screw hole into the rail.

Opening for calibrating or flashing is the reverse of step 13 only: the bumpers stay on the cover's rails and lift off with it, nothing is unplugged, and the Servo 2040's USB-C port faces the left with room for a cable. Unplug the battery at the XT60 first: there is no intrusion switch.

## Cable routes

Each run has a reserved route that no part enters (`routes()` in `layout.py`), reaching each terminal from the side it takes its wires from. Wire gauges follow the hardware-check skill.

| Route | Wires | Path |
|---|---|---|
| `xt60 to fuse and szbk07 input` | 14–16 AWG | from the XT60's solder cups along the left side through the fuse holder to the SZBK07's input end |
| `szbk07 output to ext` | 14–16 AWG | from the SZBK07's output end up the corridor on the right to the Servo 2040's EXT terminal, from its right edge |
| `fused supply to lm2596` | 22–24 AWG | from the fuse holder's output beside the SZBK07's input end, along the band between the SZBK07 and the board, and up the corridor to the LM2596's input pads |
| `lm2596 output` | 22–24 AWG | from the LM2596's output pads to the T-Display's 5V pin and, through the Schottky diode tied down in front of the module, under the board to J6's 5V pin; the only route a held part (the diode) lies in |
| `t-display link to j6` | GND, TX, RX | from the T-Display's header row down the corridor and the band behind the board to J6 |
| `servo leads` | 18 servo plugs | from the servo header in front of the board down through the two lead slots to the open frame below |
| `end-stop wires` | 7 (six end-stops and their ground) | up through the end-stop entry in the corridor to the sensor header, from its right edge |
| `imu to j6` | 4 | from the LSM6DSO's rear edge under the board into the band, and up to J6 |
| `pass-through to j6` | 4 (the VL6180X) | up through the pass-through in the band to J6 |
| `usb-c cable` | a USB-C plug | out of the Servo 2040's left edge over the deck's rim; reserved with the cover off only |
| `battery plug` | the mated XT60 plug | outside the left wall's notch, within the legs' keep-clear radius |

The fans' leads (2 × 26 AWG each) have no route of their own: the exhaust fan's run to the LM2596's output pads beside it, the intake fan's along the `fused supply to lm2596` route.

## Cooling

The mount is passive and ready for fans. Air comes in through the grille low in the left wall behind the XT60 (336 mm² open), crosses over the SZBK07's heatsinks and leaves through the grille high in the front-right corner face, so warm air rising and the exhaust fan both pull the same way; the lid has no opening over the SZBK07. The build keeps that channel clear, with no cross-section smaller than the inlet's open area.

- **Exhaust fan** (30 mm, blowing out): fit it first if the SZBK07 runs hot. Its holders stand above the board's front-right corner behind the outlet.
- **Intake fan** (30 mm, blowing in, optional): on the deck behind the inlet.
- **Supply:** both run on 5 V from the LM2596's output pads, before the Schottky diode, so their current does not pass the diode on its way to J6. A 3010 fan draws about 0.1 A; the LM2596's 5 V rail carries both boards and both fans well within its rating.
- **Limit:** after a 10-minute walk at full speed, cover on, the SZBK07's heatsink should stay at or below 70 °C (task 7.3), a margin below PETG's softening near 80 °C.

## Layout

The layout follows the wiring (`layout.py`, and `images/top.png` from above, the front at the top):
- **Deck:** a 3 mm plate on the frame top's ring, its footprint the cover's (130 × 124 mm with 45° corner faces), with tabs forward and back to the ring's four inserts. It is screwed to all eight inserts with M3 button heads in counterbores; under it, a printed spacer at each of the four floor inserts (`spacer`, four copies) bridges the 5.75 mm down to the floor.
- **SZBK07** across the rear, its input end (left) towards the XT60 and the fuse, its output end (right) towards a corridor up the right-hand side to the Servo 2040's EXT terminal. Blocks at its rear corners and low ribs at its front corners locate it; the cover is to hold it down.
- **Servo 2040** in front of it, unturned, on 9 mm standoffs (M2.5 self-tapping), with the **LSM6DSO** breakout under it on its pad (a plate with two bosses, M2 self-tapping) near the body's centre. Its EXT terminal and sensor header face the corridor, J6 the band behind it, the servo header the two lead slots in front of it over the frame's open front, and its USB-C port the left, where a cable has room with the cover off.
- **LM2596** on the right beyond the corridor, screwed (M2.5 self-tapping) onto two standoffs that raise it 13.5 mm off the deck, with the **T-Display** above it on its holder, so the screen sits beside the SZBK07 rather than over it. Its wires are soldered to its pads, so their routes rise only 3.8 mm (`LM2596_WIRE_HEIGHT`) above its board's underside. The **Schottky diode** lies in the LM2596's output route in front of the module, tied down through a pair of holes; that route runs under the board to J6's 5V pin.
- **XT60** in the left wall, in a cradle on the deck: side walls that reach out to the wall as a hood for the plug, back stops either side of its solder cups for the push of plugging in, and a cable tie through the deck over its body for the pull of unplugging. It is set 9 mm in from the cover's outer face, so the battery's plug in it ends within the legs' keep-clear radius. The **fuse holder** lies behind it along the left side, tied down at both ends.
- **Sensor provisions:** the end-stop wires come up through an entry in the corridor beside the sensor header; the distance sensor's four wires go down through a pass-through in the band to the open frame below.
- **Cover** (`images/closed.png`): a faceted shell, 130 × 124 × 45.6 mm, standing on the deck's rim and printed lid-down. It is styled as a rugged sci-fi case. Its corners are 45° faces, its top edge a 3 mm 45° chamfer, and a 0.8 mm groove runs round its walls; above the groove each side wall carries a raised armour plate, 1.6 mm proud with 45° edges all round (the inlet's grille is cut through the left one). The lid has a window over the T-Display's screen and buttons, bevelled at 45° to the top except at the side button's outer end, and no opening over the SZBK07; blocks under it come within 0.5 mm of the SZBK07's corners, nearer than its 3 mm ribs are tall, so the cover holds it down, and another block comes within 2 mm of the intake fan's top, less than its holder's 10 mm lips, so the fan cannot lift out over them upside down. The left wall is notched over the XT60 for the battery's plug. The name, three chevrons and hazard stripes along the rear edge are flush yellow inlays, 0.6 mm (three layers) deep; panel lines 1 mm wide and hex bolt heads are engraved 0.6 mm into the lid.
- **T-Display** raised on its holder until its header pins clear the lid, so the screen sits 9.5 mm below the lid's top. The holder is a portal over the LM2596: a wall between the module and the cover's right wall, a wall behind the module's input route, and a plate over the module, FIT under the board's underside parts. Pads rise from the plate into the board's three bare spots and bear on its underside there; they locate it sideways, and a lip beyond each end of the board (the rear one a block) locates it lengthwise, 2 mm above its underside. It is not screwed: the cover, 0.3 mm over the board's header pins by the right wall (1.2 mm under the flat of the lid), holds the board and the holder down. It prints on its back (rear face down).
- **Loose holders** (white): the LM2596's standoffs, the IMU's pad, the fans' holders and the T-Display's holder sit in 1.5 mm pockets in the deck (0.2 mm clearance) and are kept there by what lies over them: the T-Display's holder over the LM2596, the Servo 2040 over the IMU, the cover over the T-Display's pins, and the lid 0.5 mm over the fan holders' cheeks, with the fans fitted or not. The Servo 2040's standoffs stay part of the deck, which takes its screws.
- **Bumpers** (yellow): one part, fitted at each of the four corner faces. Each corner face carries two vertical dovetail rails near its ends, clear of the outlet's grille, their tops 45° wedges flush with the top chamfer. A bumper is a corner guard, 5 mm thick, that slides down onto them until its slots' sloped ends rest on the wedges, and is held there by an M2 × 6 self-tapping screw through one post into its rail (into the cover's wall behind it); with the screw out it lifts off upwards with the cover in place. It has a post over each end of the corner face, round its rail and past the face's end, ridged across in V grooves; above the inlet's grille the posts wrap 6 mm onto the side walls, and above the outlet's grille a cap joins them over the top chamfer, 3.5 mm above the lid (above the cover's screw heads). The middle of the face stays open for the outlet. It prints hollow, so its shell gives in a knock. It prints on its top.
- **Fastening:** three M3 screws down through the lid into heat-set inserts (Ø4.4 × 5) in posts on the deck, at the front-left, rear-left and rear-right; the front-right corner holds the exhaust fan, and no post fits beside it clear of the routes. The holes are vertical, so round, and each screw has a straight path from above.
- **Cooling:** the inlet is a grille of seven 2 × 24 mm slots low in the left wall behind the XT60 (336 mm² open), the outlet seven 2 × 26 mm slots high in the front-right corner face. A 30 mm intake fan position (optional, blowing in) stands on the deck behind the inlet, between lips and a cheek, with the XT60 cradle's wall at its front end; the exhaust position (blowing out) stands above the board's front-right corner on two columns, with cheeks at its ends that reach in over its inner face as lips (the cover's wall stops it sliding out); the lips' undersides rise at 45° from the cheeks, so they print upright without hanging in the air. Both holders are separate white parts. The passive air channel runs from the inlet over the fuse holder, across over the SZBK07's heatsinks, forward over the board, in front of the T-Display and into the corner to the outlet.

## What the build checks

Every check runs on what the part set declares; an empty set passes. When any check fails, the build names every failure and writes nothing.

- **One solid** per part, and distinct part names.
- **Fit:** nothing outside the legs' keep-clear region (r ≤ 78.8 mm, or the tabs |x| ≤ 24, |y| ≤ 79), enforced from the frame floor up; nothing below the frame floor (z 20); each part within the 256 mm bed in its print orientation, at some turn about the vertical.
- **Insert holes:** a clear M3 hole (Ø3.4) through the deck on each of the eight frame inserts, with material round it for the screw head.
- **Overlaps** between printed parts, their copies and inlays, and held parts.
- **Enclosure:** rays along ±x, ±y and +z from every held part's surface meet the cover, a vent's grille, another printed part or another held part (not a fan: both are optional), except from the regions declared exposed (the T-Display's screen and buttons, the XT60's mating face), which must meet nothing of the cover; the board's face round the screen in the window is exempt.
- **Routes:** no printed or held part enters a reserved cable route, except the Schottky diode, wholly inside the LM2596's output route and in no other; every box of that route is at least the diode's sleeve (6 mm) across. A route reserved only with the cover off (the USB-C cable) is not checked against the cover.
- **Route ends:** each route holds the space next to each terminal it ends at, on the side the terminal takes its wires from (the EXT terminal and the sensor header from +x, J6 from the board's rear edge, the T-Display's header rows from below), or the space above a wire entry in the deck.
- **Frame clearance:** each tie hole, slot and pass-through runs clear through the deck and has at least 1.5 mm of free space above the frame top's model under it.
- **Sensor provisions:** the LSM6DSO breakout within 30 mm of the centre, at least 10 mm from the SZBK07's outline, overlapping no held part, on two bosses with pilot holes, with a route to J6; the pass-through over open frame; the end-stop entry and the pass-through each taking their wires (7 and 4) side by side, the entry with a route to the Servo 2040.
- **Overhangs** in each part's print orientation: no face looks down more steeply than 45° unless it lies on the bed or is planar, at most 3 mm wide, and nowhere more than 3 mm from where it hangs from material (an edge with a face going down from it: a wall, a column, a 45° face), so a bridge passes and an arm held at one end does not; a curved downward face never passes, so a horizontal hole must be a teardrop.
- **Screwdriver paths:** a Ø4 cylinder from each screw head outwards along its axis meets no part fitted before the screw is driven.
- **Air channel:** connected boxes from the space behind the inlet's grille past the SZBK07 to the space behind the outlet's, entered by no part (the fans left out, as the passive case has none), whose cross-section along each box and across each junction is nowhere smaller than the inlet's net open area. Each grille's open area is measured through the cover (its narrowest section across the wall, from exact sections between every depth where the wall's shape changes, including the narrowest point of a section that narrows between them, so a slot shut at any depth, however thin the skin, counts as shut): it must be the area the layout declares and no smaller than the inlet's. A box may be turned about the vertical (the run into the front-right corner); where two boxes in differently turned frames meet, the junction's cross-section across a flow is the shared region's widest section across it.
- **Fans:** each fan position holds its held fan, supported under both ends and located at both end faces by the deck's holders, with the cover no nearer than 0.1 mm (it carries no fan); moved 1 mm across its width or inwards it meets its holder, and outwards its holder or the cover's wall; at least a quarter of the rays from its outer face pass out through its vent and past the bumpers; the exhaust faces out of the box; a part set with an air channel has an exhaust position.
- **Bumpers:** in every horizontal direction (5° apart) and upwards, the bumpers reach at least 1 mm beyond the cover, every held part and the cover's screw heads (not their own), so their tops are above the lid and the screen; each is held by a screw whose head lies on it and whose shank reaches into the cover; each, lowered 0.5 mm, meets the cover (its stop), moved 0.1 mm sideways meets nothing (it slides on its rails), and raised by 0.5 to 80 mm meets nothing (with its screw out, it comes off with the cover in place); each half of it, pulled off its corner face, meets the cover within 5 mm (each rail keeps its end on).
- **Kept in place:** each loose holder rises less than its pocket's depth before something fixed stops it: rays straight up from every face of it that looks up measure how far it rises to what lies over it; a loose holder or a held part one carries, met on the way, is pushed up and rises only as far as it can itself (the LM2596's standoffs push the module, the T-Display's holder and the board up to the cover).
- **Support:** the T-Display's holder bears on the board's underside (at least 2 mm² in each bare spot where the holder comes within 0.1 mm under the declared face and the board within 0.1 mm over it) and nowhere else under it, and the board moved 1 mm along x or y meets the holder.
- **Cover:** every screen that must show sits below the cover's top; the cover comes nearer above a held-down part than its stops are tall (the SZBK07, the T-Display, the fans); the lid has no opening above the SZBK07 (rays up from its top, 1 mm apart, all meet the cover); the cover lifts straight off with its bumpers: nothing else lies above any face of it that looks up (rays up from them), and raised by 0.5 to 80 mm it overlaps nothing else.

## Output

In the frame top's coordinates, each distinct part once (the four bumpers are one file):
- `print/<part>.stl`, or `print/cover.3mf`: one object whose two parts are the body and the inlays, for the AMS (Bambu Studio 2.8.2 loads it as one object of two parts, each assignable to a filament). The files are in the robot's orientation: lay the cover lid-down in the slicer;
- `step/<part>.step`.

The build deletes files there that belong to no current part, and re-reads what it wrote: one print file and one STEP file per part, each with the built part's bounding box, the 3MF one object of two parts, and the deck's eight insert holes found again in its STEP. The files are byte-for-byte repeatable.

## Measure before printing

These values are nominal: the parts have not been measured, nor the wire sides checked on the board, yet. Measure each with calipers before printing, write the result in the last column as the value in backticks followed by how and when it was measured in brackets (for example `` `(59.6, 53.2, 27.4)` (calipers, 2026-11-02) ``), and change the constant in `measurements.py` to match; `test_measurements.py` checks that the two agree, and the checks then re-validate the parts. A value the layout takes as given rather than sizing a part from (the SZBK07's terminals on its short ends, its heatsinks within its outline, the holes big enough for their screws, the diode inside its sleeve) is checked by `layout.measurement_failures()`, and the build fails on one that no longer holds: the layout must then be redrawn. The fans' holes and wire exit are recorded only (`REFERENCE_ONLY`): the holders hold the fans by their frames and the leads are routed by hand, which the fan holder's test print checks. Lengths are millimetres.

| Constant | Nominal value | Source | Measured |
|---|---|---|---|
| `SZBK07_SIZE` | `(60.0, 54.0, 28.0)` | reseller listings (about 60 × 53–54 × 27–28 by batch), the larger figures | not yet |
| `SZBK07_TERMINALS_ON_SHORT_ENDS` | `True` | reseller photos: input terminals on one short end, output on the other | not yet |
| `SZBK07_HEATSINK_TOP` | `28.0` | taken as the full outline height | not yet |
| `LM2596_SIZE` | `(45.0, 20.0, 14.0)` | reseller listings of the common blue module | not yet |
| `LM2596_HOLES` | `((7.0, 2.5), (38.0, 17.5))` | reseller photos: two holes on the diagonal, from the module's corner | not yet |
| `LM2596_HOLE_D` | `3.0` | M3 clearance on the common module | not yet |
| `LM2596_PAD_INSET` | `2.5` | reseller photos: input pads at one end, output pads at the other | not yet |
| `FAN_SIZE` | `(30.0, 30.0, 10.0)` | the 3010 fan size | not yet |
| `FAN_HOLE_SPACING` | `24.0` | the usual 3010 hole spacing | not yet |
| `FAN_HOLE_D` | `3.0` | the usual 3010 hole | not yet |
| `FAN_WIRE_EXIT_AT_CORNER` | `True` | assumed: the leads leave at one corner | not yet |
| `LSM6DSO_SIZE` | `(20.0, 15.0)` | assumed: no breakout has been chosen yet | not yet |
| `LSM6DSO_HOLE_SPACING` | `15.0` | assumed: two holes along the length | not yet |
| `LSM6DSO_HOLE_D` | `2.2` | assumed: M2 | not yet |
| `SCHOTTKY_BODY` | `(5.2, 2.7)` | the DO-41 outline (a 1N5819) | not yet |
| `SCHOTTKY_SLEEVE_D` | `6.0` | the heat-shrink's size before shrinking | not yet |
| `SCHOTTKY_SLEEVE_LENGTH` | `20.0` | assumed: the body and both solder joints | not yet |
| `SERVO_PLUG_HEIGHT` | `20.0` | assumed: a 2.54 mm servo plug about 14 tall on a header pin, plus its lead bent back | not yet |
| `SZBK07_TERMINAL_SPAN` | `30.0` | assumed: reseller photos, a 4-way block about 25 mm long in the middle of each short end | not yet |
| `XT60_SIZE` | `(16.0, 15.5, 8.2)` | assumed: the Amass XT60 outline, about 16 × 15.5 × 8.2 | not yet |
| `XT60_CUP_LENGTH` | `7.0` | assumed: the solder cups behind the housing | not yet |
| `XT60_CUP_SECTION` | `(10.0, 6.0)` | assumed: the two cups side by side | not yet |
| `XT60_PLUG_LENGTH` | `20.0` | assumed: the battery's plug beyond the socket's face when mated, with its solder joints | not yet |
| `FUSE_HOLDER_SIZE` | `(35.0, 14.0, 10.0)` | assumed: an inline ATO blade-fuse holder, between its lead exits | not yet |
| `S2040_PCB_THICKNESS` | `1.6` | a standard 1.6 mm board | not yet |
| `S2040_UNDERSIDE_LEADS` | `2.0` | assumed: the through-hole leads below the board | not yet |
| `USB_C_PLUG` | `(12.5, 7.0, 30.0)` | assumed: a common USB-C cable's overmould, about 12 × 6.5 × 30 | not yet |
| `LSM6DSO_HEIGHT` | `3.0` | assumed: the breakout's board and parts on its bosses | not yet |
| `SERVO_PLUG_SECTION` | `(8.0, 4.5)` | assumed: a 3-way 2.54 mm servo plug with its key, which must pass through a lead slot | not yet |
| `SENSOR_WIRE_D` | `1.3` | assumed: 26 AWG hook-up wire with its insulation | not yet |
| `LM2596_WIRE_HEIGHT` | `3.8` | assumed: the 1.6 mm board and 22 AWG wire soldered on its pads; over 3.87 the LM2596's wire routes meet the exhaust fan holder's right lip (`EXH_LIP_Z` in `layout.py`) | not yet |
| `MODULE_PCB_THICKNESS` | `1.6` | assumed: a standard 1.6 mm board, for the LM2596 module and the LSM6DSO breakout under their screw heads | not yet |
| `S2040_HEIGHT` | `12.0` | Pimoroni's product page (62 × 42 × 12 mm with its connectors); the drawing does not give it | not yet |
| `S2040_EXT_WIRE_SIDE` | `'+x'` | the `spooder-printed-parts` skill: the EXT terminal takes its wires from the board's right edge (the drawing does not show its opening) | not yet |
| `S2040_SENSOR_WIRE_SIDE` | `'+x'` | assumed: the sensor header's leads leave towards the board's right edge, the nearest edge clear of other headers | not yet |

Also measure, with no constant of their own yet: the LM2596's pad positions and the fan's wire exit.

## Reference data

Taken from the repository's reference files, each constant with its source in `measurements.py`:
- **Frame top** (`hardware/aecert-hexapod-v1/stl/Frame Top.stl`): the ring at z 25.75, the floor at z 20, the underside at z 16; the eight M3 insert holes (Ø4.4, the ring's blind to z 19.75, the floor's through); and the open rectangles with the battery bay below them (`FRAME_OPEN`).
- **Servo 2040** (`hardware/pimoroni-servo2040/servo-2040-drawing.png`): the servo header (its dimensions run to each group's centre; the dimensioned row is "+", with "−" above it and the signal row below), the EXT terminal, the sensor header, J6, the mounting holes and the USB-C connector, in the board's own frame. The pads' positions are checked against the drawing's pixels; which row or column carries which signal is read from its labels.
- **T-Display** (`hardware/lilygo-t-display/3d_file/T-Display.step`): the PCB, the screen and buttons on one face, the header rows and how far their pins stand out (8.5 mm from the PCB's back face), the tallest part on the other face (3.4 mm) and the bare spots on that face where a holder can bear (`TD_BARE`). The model has no USB-C connector.
