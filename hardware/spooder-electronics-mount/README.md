# Spooder electronics mount

Spooder's own printed mount for its electronics on the frame top: a deck screwed to the frame's inserts, a cover, corner bumpers and small holders. It is generated as code and checked before export (the `spooder-printed-parts` skill); the OpenSpec change `add-electronics-mount` describes it.

This folder is being built: the generator, its checks and the layout are in place, with the deck and its spacers and the cover; the bumpers and the remaining holders follow.

| File | What it is |
|---|---|
| `measurements.py` | every size, as a named constant with its source in a comment |
| `reference_check.py` | re-derives the reference data in `measurements.py` from the frame top STL, the T-Display STEP model and the Servo 2040 drawing, and fails on any mismatch |
| `build.py` | generates the parts, runs every check, and only when all pass writes the print and CAD files and re-reads them |
| `layout.py` | the layout: where each held part sits, the deck with its stops, standoffs, bosses, the XT60's cradle, the cover's posts and the fans' holders, the spacers under it, the cover, the cable routes with the terminals they reach, the slots, tie holes, wire entry and pass-through, the vents, the fan positions, the air channel and the screws |
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

### Layout

The layout follows the wiring (`layout.py`, and `images/top.png` from above, the front at the top):
- **Deck:** a 3 mm plate on the frame top's ring, its footprint the cover's (130 × 124 mm with 45° corner faces), with tabs forward and back to the ring's four inserts. It is screwed to all eight inserts with M3 button heads in counterbores; under it, a printed spacer at each of the four floor inserts (`spacer`, four copies) bridges the 5.75 mm down to the floor.
- **SZBK07** across the rear, its input end (left) towards the XT60 and the fuse, its output end (right) towards a corridor up the right-hand side to the Servo 2040's EXT terminal. Blocks at its rear corners and low ribs at its front corners locate it; the cover is to hold it down.
- **Servo 2040** in front of it, unturned, on 9 mm standoffs (M2.5 self-tapping), with the **LSM6DSO** breakout under it on two bosses near the body's centre. Its EXT terminal and sensor header face the corridor, J6 the band behind it, the servo header the two lead slots in front of it over the frame's open front, and its USB-C port the left, where a cable has room with the cover off.
- **LM2596** on the right beyond the corridor, screwed down on two bosses (M2.5 self-tapping), with the **T-Display** above it on a holder (task 5.2), so the screen sits beside the SZBK07 rather than over it. The **Schottky diode** lies in the LM2596's output route in front of the module, tied down through a pair of holes; that route runs under the board to J6's 5V pin.
- **XT60** in the left wall, in a cradle on the deck: side walls that reach out to the wall as a hood for the plug, back stops either side of its solder cups for the push of plugging in, and a cable tie through the deck over its body for the pull of unplugging. It is set 9 mm in from the cover's outer face, so the battery's plug in it ends within the legs' keep-clear radius. The **fuse holder** lies behind it along the left side, tied down at both ends.
- **Sensor provisions:** the end-stop wires come up through an entry in the corridor beside the sensor header; the distance sensor's four wires go down through a pass-through in the band to the open frame below.
- **Cover** (`images/closed.png`): a faceted shell, 130 × 124 × 45.6 mm, standing on the deck's rim and printed lid-down. Its corners are 45° faces, its top edge a 3 mm 45° chamfer, and a 0.8 mm groove runs round its walls. The lid has a window over the T-Display's screen and buttons, bevelled at 45° to the top except at the side button's outer end, and no opening over the SZBK07; blocks under it come within 0.5 mm of the SZBK07's corners, nearer than its 3 mm ribs are tall, so the cover holds it down. The left wall is notched over the XT60 for the battery's plug. The name and three chevrons are flush yellow inlays, 0.6 mm (three layers) deep.
- **T-Display** raised on its holder (task 5.2) until its header pins clear the lid, so the screen sits 9.5 mm below the lid's top.
- **Fastening:** three M3 screws down through the lid into heat-set inserts (Ø4.4 × 5) in posts on the deck, at the front-left, rear-left and rear-right; the front-right corner holds the exhaust fan, and no post fits beside it clear of the routes. The holes are vertical, so round, and each screw has a straight path from above.
- **Cooling:** the inlet is a grille of seven 2 × 24 mm slots low in the left wall behind the XT60 (336 mm² open), the outlet seven 2 × 26 mm slots high in the front-right corner face. A 30 mm intake fan position (optional, blowing in) stands on the deck behind the inlet, between lips and a cheek, with the XT60 cradle's wall at its front end; the exhaust position (blowing out) stands above the board's front-right corner on two columns, with cheeks at its ends that reach in over its inner face as lips (the cover's wall stops it sliding out). Both holders are part of the deck for now (task 5.3). The passive air channel runs from the inlet over the fuse holder, across over the SZBK07's heatsinks, forward over the board, in front of the T-Display and into the corner to the outlet.

### What the build checks

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
- **Overhangs** in each part's print orientation: no face looks down more steeply than 45° unless it lies on the bed or is a planar bridge at most 3 mm wide; a curved downward face never passes, so a horizontal hole must be a teardrop.
- **Screwdriver paths:** a Ø4 cylinder from each screw head outwards along its axis meets no part fitted before the screw is driven.
- **Air channel:** connected boxes from the space behind the inlet's grille past the SZBK07 to the space behind the outlet's, entered by no part (the fans left out, as the passive case has none), whose cross-section along each box and across each junction is nowhere smaller than the inlet's net open area. A box may be turned about the vertical (the run into the front-right corner); where two boxes in differently turned frames meet, the junction's cross-section across a flow is the shared region's widest section across it.
- **Fans:** each fan position holds its held fan, supported under both ends and located at both end faces by the deck's holders, with the cover no nearer than 0.1 mm (it carries no fan); moved 1 mm across its width or inwards it meets its holder, and outwards its holder or the cover's wall; at least a quarter of the rays from its outer face pass out through its vent; the exhaust faces out of the box; a part set with an air channel has an exhaust position.
- **Cover:** every screen that must show sits below the cover's top; the cover comes nearer above a held-down part than its stops are tall (the SZBK07); the lid has no opening above the SZBK07 (rays up from its top, 1 mm apart, all meet the cover); the cover lifts straight off: nothing lies above any face of it that looks up (rays up from them), and raised by 0.5 to 80 mm it overlaps nothing.

### Output

In the frame top's coordinates, each distinct part once (the four bumpers are one file):
- `print/<part>.stl`, or `print/cover.3mf`: one object whose two parts are the body and the inlays, for the AMS (Bambu Studio 2.8.2 loads it as one object of two parts, each assignable to a filament). The files are in the robot's orientation: lay the cover lid-down in the slicer;
- `step/<part>.step`.

The build deletes files there that belong to no current part, and re-reads what it wrote: one print file and one STEP file per part, each with the built part's bounding box, the 3MF one object of two parts, and the deck's eight insert holes found again in its STEP. The files are byte-for-byte repeatable.

## Measure before printing

These values are nominal: the parts have not been measured, nor the wire sides checked on the board, yet. Measure each with calipers before printing, write the result in the last column as the value in backticks followed by how and when it was measured in brackets (for example `` `(59.6, 53.2, 27.4)` (calipers, 2026-11-02) ``), and change the constant in `measurements.py` to match; `test_measurements.py` checks that the two agree, and the checks then re-validate the parts. Lengths are millimetres.

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
| `S2040_EXT_WIRE_SIDE` | `'+x'` | the `spooder-printed-parts` skill: the EXT terminal takes its wires from the board's right edge (the drawing does not show its opening) | not yet |
| `S2040_SENSOR_WIRE_SIDE` | `'+x'` | assumed: the sensor header's leads leave towards the board's right edge, the nearest edge clear of other headers | not yet |

Also measure, with no constant of their own yet: the LM2596's pad positions and the fan's wire exit.

## Reference data

Taken from the repository's reference files, each constant with its source in `measurements.py`:
- **Frame top** (`hardware/aecert-hexapod-v1/stl/Frame Top.stl`): the ring at z 25.75, the floor at z 20, the underside at z 16; the eight M3 insert holes (Ø4.4, the ring's blind to z 19.75, the floor's through); and the open rectangles with the battery bay below them (`FRAME_OPEN`).
- **Servo 2040** (`hardware/pimoroni-servo2040/servo-2040-drawing.png`): the servo header (its dimensions run to each group's centre; the dimensioned row is "+", with "−" above it and the signal row below), the EXT terminal, the sensor header, J6, the mounting holes and the USB-C connector, in the board's own frame. The pads' positions are checked against the drawing's pixels; which row or column carries which signal is read from its labels.
- **T-Display** (`hardware/lilygo-t-display/3d_file/T-Display.step`): the PCB, the screen and buttons on one face, the header rows and how far their pins stand out (8.5 mm from the PCB's back face), the tallest part on the other face (3.4 mm) and the bare spots on that face where a holder can bear (`TD_BARE`). The model has no USB-C connector.
