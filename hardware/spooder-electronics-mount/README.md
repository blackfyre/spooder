# Spooder electronics mount

Spooder's own printed mount for its electronics on the frame top: a deck screwed to the frame's inserts, a cover, corner bumpers and small holders. It is generated as code and checked before export (the `spooder-printed-parts` skill); the OpenSpec change `add-electronics-mount` describes it.

This folder is being built: the generator and its checks are in place, and the part set is still empty (the deck, the cover, the bumpers and the holders follow).

| File | What it is |
|---|---|
| `measurements.py` | every size, as a named constant with its source in a comment |
| `reference_check.py` | re-derives the reference data in `measurements.py` from the frame top STL, the T-Display STEP model and the Servo 2040 drawing, and fails on any mismatch |
| `build.py` | generates the parts, runs every check, and only when all pass writes the print and CAD files and re-reads them |
| `model.py` | the part set's model (printed parts, held parts, cable routes, deck openings, screws, the air channel) and geometry helpers such as `box()` and `teardrop()` |
| `checks.py` | the checks, each rule's number named with its source |
| `export.py` | the export, and the re-reading of what it wrote |
| `preview.py` | renders the assembly on the frame top to `images/` |
| `test_build.py` | shows each check failing on a bad shape and passing on a good one, and tests the export |
| `test_measurements.py` | checks that every constant has a source and that the table below lists each nominal value |

The scripts pin their dependencies inline (build123d, numpy, trimesh, matplotlib and, for the reference check, pillow), locked in `<script>.lock` by `uv lock --script`:

```sh
mise run cad          # reference check, unit tests, then build.py: check and write
mise run cad-preview  # images/assembly.png and images/top.png
uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --check  # check, write nothing
```

### What the build checks

Every check runs on what the part set declares; an empty set passes. When any check fails, the build names every failure and writes nothing.

- **One solid** per part, and distinct part names.
- **Fit:** nothing outside the legs' keep-clear region (r ≤ 78.8 mm, or the tabs |x| ≤ 24, |y| ≤ 79), enforced from the frame floor up; nothing below the frame floor (z 20); each part within the 256 mm bed in its print orientation, at some turn about the vertical.
- **Insert holes:** a clear M3 hole (Ø3.4) through the deck on each of the eight frame inserts, with material round it for the screw head.
- **Overlaps** between printed parts, their copies and inlays, and held parts.
- **Enclosure:** rays along ±x, ±y and +z from every held part's surface meet the cover, except from the regions declared exposed (the T-Display's screen and buttons, the XT60's mating face), which must meet nothing of it.
- **Routes:** no printed or held part enters a reserved cable route, except the Schottky diode, wholly inside the LM2596's output route and in no other.
- **Frame clearance:** each tie hole, slot and pass-through runs clear through the deck and has at least 1.5 mm of free space above the frame top's model under it.
- **Overhangs** in each part's print orientation: no face looks down more steeply than 45° unless it lies on the bed or is a planar bridge at most 3 mm wide; a curved downward face never passes, so a horizontal hole must be a teardrop.
- **Screwdriver paths:** a Ø4 cylinder from each screw head outwards along its axis meets no part fitted before the screw is driven.
- **Air channel:** connected boxes from the space behind the inlet's grille past the SZBK07 to the space behind the outlet's, entered by no part (the fans left out, as the passive case has none), whose cross-section along each box and across each junction is nowhere smaller than the inlet's net open area.

### Output

In the frame top's coordinates, each distinct part once (the four bumpers are one file):
- `print/<part>.stl`, or `print/cover.3mf` with the body and the inlays as separate objects for the AMS;
- `step/<part>.step`.

The build deletes files there that belong to no current part, and re-reads what it wrote: one print file and one STEP file per part, each with the built part's bounding box, and the deck's eight insert holes found again in its STEP. The files are byte-for-byte repeatable.

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
| `S2040_EXT_WIRE_SIDE` | `'+x'` | the `spooder-printed-parts` skill: the EXT terminal takes its wires from the board's right edge (the drawing does not show its opening) | not yet |
| `S2040_SENSOR_WIRE_SIDE` | `'+x'` | assumed: the sensor header's leads leave towards the board's right edge, the nearest edge clear of other headers | not yet |

Also measure, with no constant of their own yet: the SZBK07's terminal positions on its ends, the LM2596's pad positions, and the fan's wire exit.

## Reference data

Taken from the repository's reference files, each constant with its source in `measurements.py`:
- **Frame top** (`hardware/aecert-hexapod-v1/stl/Frame Top.stl`): the ring at z 25.75, the floor at z 20, the underside at z 16; the eight M3 insert holes (Ø4.4, the ring's blind to z 19.75, the floor's through); and the open rectangles with the battery bay below them (`FRAME_OPEN`).
- **Servo 2040** (`hardware/pimoroni-servo2040/servo-2040-drawing.png`): the servo header (its dimensions run to each group's centre; the dimensioned row is "+", with "−" above it and the signal row below), the EXT terminal, the sensor header, J6, the mounting holes and the USB-C connector, in the board's own frame. The pads' positions are checked against the drawing's pixels; which row or column carries which signal is read from its labels.
- **T-Display** (`hardware/lilygo-t-display/3d_file/T-Display.step`): the PCB, the screen and buttons on one face, the header rows, the tallest part on the other face (3.4 mm) and the bare spots on that face where a holder can bear (`TD_BARE`). The model has no USB-C connector.
