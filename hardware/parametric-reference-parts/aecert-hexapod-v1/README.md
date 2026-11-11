# Aecert's Hexapod V1: parametric parts

Parametric solids of the printed parts of Ryan Mirch's Aecert's Hexapod V1, modelled from dimensions measured on its STLs (`hardware/aecert-hexapod-v1/stl/`), checked against them and exported as STEP. The build, the part contract and the gates are in the folder's README (`../README.md`).

## Source

- Repository: https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1
- Commit `ca5ed27` (2024-04-09), the one Spooder's movement code was ported from and whose STLs `hardware/aecert-hexapod-v1/` holds unchanged.
- Licence: upstream states no licence; the author knows about Spooder's derived models and has raised no concerns, which is informal and grants no licence. These parts reproduce Aecert's design; they are not Spooder's own work, and no licence is granted for them here.

Each part is built in its STL's own coordinates. Frame Top and Frame Bottom share one frame, the body frame (y forward, z up, the coxa pivots at z = 0), so the movement constants measured from them (`core/movement/body_geometry.h`) can be read off these solids too.

## Frame group

Every part passes the global gates (`../README.md`, "Gates and tolerances"); none declares an exception. The measured values are from `build.py --check` on 2026-11-11.

| Part | Part file | STL | STEP | Volume | Max deviation | 95th percentile | Exception |
|---|---|---|---|---|---|---|---|
| Frame Top | `parts/frame_top.py` | `hardware/aecert-hexapod-v1/stl/Frame Top.stl` | `step/Frame Top.step` | 0.109 % | 0.067 mm | 0.028 mm | none |
| Frame Bottom | `parts/frame_bottom.py` | `hardware/aecert-hexapod-v1/stl/Frame Bottom.stl` | `step/Frame Bottom.step` | 0.106 % | 0.122 mm | 0.029 mm | none |
| Servo Mount | `parts/servo_mount.py` | `hardware/aecert-hexapod-v1/stl/Servo Mount.stl` | `step/Servo Mount.step` | 0.001 % | 0.100 mm | 0.002 mm | none |
| Bearing Axle Mount | `parts/bearing_axle_mount.py` | `hardware/aecert-hexapod-v1/stl/Bearing Axle Mount.stl` | `step/Bearing Axle Mount.step` | 0.003 % | 0.006 mm | 0.002 mm | none |
| Hubcap | `parts/hubcap.py` | `hardware/aecert-hexapod-v1/stl/Hubcap.stl` | `step/Hubcap.step` | 0.019 % | 0.011 mm | 0.003 mm | none |

Limits: volume 0.5 %, maximum deviation 0.20 mm, 95th percentile 0.05 mm, bounding box 0.15 mm (every frame part within 0.025 mm).

- Frame Top and Frame Bottom share their plan in `parts/_frame.py`: the outline, the six arms (one modelled, placed at the gait's mount angles, 0° and ±56° from the sides, the corner arms 10.53 mm further out), the coxa servo slots, the Bearing Axle Mount hole pairs (28.5 mm apart, 87.60 mm from the centre on the middle arms and 98.13 mm on the corners), the open middle and the frame screws.
- Where an STL's arc is not exactly circular (Frame Top's and Frame Bottom's arc C inside the rim and walls, Frame Bottom's second rear arc), the constant is the least-squares circle through its vertices, and its comment says so.
- Frame Bottom's chamfers are cut as one wedge per convex edge (`_frame.chamfered`), where OpenCascade's chamfer fails on its thin walls; the STL's own unchamfered edges and its 0.75 mm rear floor round are modelled as it has them.
- Servo Mount: the 15 mm ring and the 5 mm ear pads that `hardware/aecert-hexapod-v1/README.md` uses for the coxa servo's height.
- Bearing Axle Mount: the 9.4 mm base and the axle 9.90 mm off the screw line, from which the coxa pivots are placed.

## Coxa group

Every part passes the global gates (`../README.md`, "Gates and tolerances"); none declares an exception. The measured values are from `build.py --check` on 2026-11-11.

| Part | Part file | STL | STEP | Volume | Max deviation | 95th percentile | Exception |
|---|---|---|---|---|---|---|---|
| Coxa Horn Connector | `parts/coxa_horn_connector.py` | `hardware/aecert-hexapod-v1/stl/Coxa Horn Connector.stl` | `step/Coxa Horn Connector.step` | 0.022 % | 0.013 mm | 0.002 mm | none |
| Coxa Horn Side | `parts/coxa_horn_side.py` | `hardware/aecert-hexapod-v1/stl/Coxa Horn Side.stl` | `step/Coxa Horn Side.step` | 0.001 % | 0.006 mm | 0.003 mm | none |
| Coxa Bearing Connector | `parts/coxa_bearing_connector.py` | `hardware/aecert-hexapod-v1/stl/Coxa Bearing Connector.stl` | `step/Coxa Bearing Connector.step` | 0.021 % | 0.013 mm | 0.003 mm | none |
| Coxa Bearing Side | `parts/coxa_bearing_side.py` | `hardware/aecert-hexapod-v1/stl/Coxa Bearing Side.stl` | `step/Coxa Bearing Side.step` | 0.003 % | 0.006 mm | 0.003 mm | none |

Limits: volume 0.5 %, maximum deviation 0.20 mm, 95th percentile 0.05 mm, bounding box 0.15 mm (every coxa part within 0.004 mm).

The STLs' own chord error (`build.py --chord-error`), far below the 0.15 mm above which a part would need a tessellation exception: Coxa Horn Connector 0.003 mm (r 10.10, n 123, the horn's recess), Coxa Horn Side 0.004 mm (r 7.00, n 92, the corners between the ears), Coxa Bearing Connector 0.003 mm (r 11.20, n 130, the bearing's seat), Coxa Bearing Side 0.004 mm (r 7.00, n 92).

- The sides share their plate in `parts/_coxa.py` (the outline, the ears, the 28.5 mm hole pair and the chamfers); they differ in their place, their window (57.3 × 20 mm on the Horn Side; 40 × 20 mm with an 8 mm slot on the Bearing Side) and which holes are countersunk. Their volumes differ by 21 %, so neither is a mirror of the other.
- The connectors share their ring and tongue in `parts/_coxa.py`, the Bearing Connector's turned 180° about its axis; they differ in the seat: the horn's r 10.1 recess with its five holes, or the bearing's r 11.2 bore over an r 9 hole.
- The Bearing Side's window chamfer is cut 0.001 mm short of 1 mm: at 1 mm it takes the slot's r 1 corners to nothing, which OpenCascade's chamfer cannot do.
- The connectors' seats, with their rounded rims and floors, are cut with a solid turned from their profile, not rounded with fillets: the fillets' tori come out of the STEP as open shells in solidcraft-cli 0.30.1, as the Bearing Axle Mount's boss foot did.

### Coxa link geometry against the record

The coxa link geometry that `hardware/aecert-hexapod-v1/README.md` records, measured on the built solids (`test_coxa.py` finds each value from the solid's faces and checks it against this table to 0.01 mm). A difference above 0.05 mm is marked `disagrees`; none is.

| Value | Part | Recorded (mm) | Model (mm) | Disagreement |
|---|---|---|---|---|
| Insert hole from the coxa axis | Coxa Horn Connector | 21.5 | 21.500 |  |
| Ring height | Coxa Horn Connector | 8.05 | 8.050 |  |
| Insert hole above the underside | Coxa Horn Connector | 4.025 | 4.025 |  |
| Insert hole from the coxa axis | Coxa Bearing Connector | 21.5 | 21.500 |  |
| Ring height | Coxa Bearing Connector | 8.05 | 8.050 |  |
| Insert hole above the underside | Coxa Bearing Connector | 4.025 | 4.025 |  |
| Each ear from the hole pair's middle | Coxa Horn Side | 30.10 | 30.100 |  |
| Hole pair spacing | Coxa Horn Side | 28.5 | 28.500 |  |
| Side plate length | Coxa Horn Side | 68.25 | 68.250 |  |
| Side plate width | Coxa Horn Side | 48 | 48.025 |  |
| Each ear from the hole pair's middle | Coxa Bearing Side | 30.10 | 30.100 |  |
| Hole pair spacing | Coxa Bearing Side | 28.5 | 28.500 |  |
| Side plate length | Coxa Bearing Side | 68.25 | 68.250 |  |
| Side plate width | Coxa Bearing Side | 48 | 48.025 |  |

The plate's width is 48.025 mm to the top of its ears (r 4.025 round holes 25.5 mm beyond the pair's middle, and 18.5 mm to the plate's other edge), within 0.05 mm of the recorded 48.

## Femur group

Every part passes the global gates; none declares an exception. The measured values are from `build.py --check` on 2026-11-11.

| Part | Part file | STL | STEP | Volume | Max deviation | 95th percentile | Exception |
|---|---|---|---|---|---|---|---|
| Femur Horn Side | `parts/femur_horn_side.py` | `hardware/aecert-hexapod-v1/stl/Femur Horn Side.stl` | `step/Femur Horn Side.step` | 0.000 % | 0.026 mm | 0.004 mm | none |
| Femur Bearing Side | `parts/femur_bearing_side.py` | `hardware/aecert-hexapod-v1/stl/Femur Bearing Side.stl` | `step/Femur Bearing Side.step` | 0.013 % | 0.032 mm | 0.004 mm | none |
| Femur Support | `parts/femur_support.py` | `hardware/aecert-hexapod-v1/stl/Femur Support.stl` | `step/Femur Support.step` | 0.022 % | 0.035 mm | 0.013 mm | none |

Limits: volume 0.5 %, maximum deviation 0.20 mm, 95th percentile 0.05 mm, bounding box 0.15 mm (every femur part within 0.007 mm). The STLs' chord error (`build.py --chord-error`) is 0.007 mm on both side plates and 0.003 mm on Femur Support, well under the 0.15 mm above which a tessellation exception would be declared.

- Femur Horn Side and Femur Bearing Side share their plan in `parts/_femur.py`: the 30 mm stadium outline round the two bore axes, 8.35 mm thick, the two lightening slots (their 45° sides running back on the horn side and forward on the bearing side), the countersunk middle hole and the recess over the web between the slots. Each part file holds its first bore's axis and its own bores: the horn side an r 10.098 horn recess with four screw holes 6.8 mm off each axis, the bearing side an r 11.2 bearing pocket over an r 9.0 hole through its floor.
- **Axis-to-axis length.** Both sides place their second bore `AXIS_TO_AXIS` (110.000 mm, `parts/_femur.py`) along x from their first. Circle fits to the bores give 110.000 mm on both STLs, on every section; `hardware/aecert-hexapod-v1/README.md` gives the femur as 108 mm ("Measured on Spooder"), 2 mm less. The constant takes the STLs' value, which the gates hold the solids to; correcting that README's 108 is a follow-up. It is the printed parts' bore spacing, not the movement's femur link length (`FEMUR_LENGTH`, 116 mm). `test_femur.py` finds both sides' bore axes from the built solids' cylindrical faces, checks they lie `AXIS_TO_AXIS` apart to 0.01 mm, and rebuilds both with the constant raised by 1 mm to check they follow it.
- Femur Support's sides and the two rounded ends of the plates' web recess are parabolas, not arcs (a parabola fits their vertices to 0.006 mm and 0.005 mm, a circle only to 0.081 mm and 0.07 mm), so each is modelled as a quadratic Bézier, which is exactly a parabola, and needs no exception.

## Feet group

The measured values are from `build.py --check` on 2026-11-11.

| Part | Part file | STL | STEP | Volume | Max deviation | 95th percentile | Exception |
|---|---|---|---|---|---|---|---|
| Foot Shaft | `parts/foot_shaft.py` | `hardware/aecert-hexapod-v1/stl/Foot Shaft.stl` | `step/Foot Shaft.step` | 0.022 % | 0.003 mm | 0.002 mm | none |
| Foot Tip | `parts/foot_tip.py` | `hardware/aecert-hexapod-v1/stl/Foot Tip.stl` | `step/Foot Tip.step` | 0.188 % | 0.012 mm | 0.005 mm | none |

Limits: volume 0.5 %, maximum deviation 0.20 mm, 95th percentile 0.05 mm, bounding box 0.15 mm (both parts within 0.003 mm).

- Foot Shaft: a hexagonal shaft (flats 4 mm apart, 45° sides meeting 4.5 mm from the axis, those points rounded 0.8 mm), a half-hexagon pin with a slanted end, an upright collar and a 5 mm tongue.
- Foot Tip: a 10.4 mm cylinder whose top edge is a 4.5 mm round, turned as one profile, with an octagonal socket 5.15 mm deep in its underside.

## Groups still to model

Each is a later change adding its part files, its STEP files, its table here and a `CHANGELOG.md` line:

- **Tibia:** Tibia Left and Tibia Right (a mirrored pair), Tibia Support.
- **Feet:** Foot Left and Foot Right (Foot Shaft and Foot Tip are in the feet group above).
- **Plates:** Leg Parts A, Leg Parts B and Foot Left Right, arrangements of the single parts.
