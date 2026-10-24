# Spooder electronics mount

Spooder's own printed mount for its electronics on the frame top: a deck screwed to the frame's inserts, a cover, corner bumpers and small holders. It is generated as code and checked before export (the `spooder-printed-parts` skill); the OpenSpec change `add-electronics-mount` describes it.

This folder is being built. So far it holds the measurements the parts will be built from:

| File | What it is |
|---|---|
| `measurements.py` | every size, as a named constant with its source in a comment |
| `reference_check.py` | re-derives the reference data in `measurements.py` from the frame top STL, the T-Display STEP model and the Servo 2040 drawing, and fails on any mismatch; its dependencies are locked in `reference_check.py.lock` |
| `test_measurements.py` | checks that every constant has a source and that the table below lists each nominal value |

```sh
uv run --locked --quiet hardware/spooder-electronics-mount/reference_check.py
cd hardware/spooder-electronics-mount && python3 -m unittest -v test_measurements
```

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
