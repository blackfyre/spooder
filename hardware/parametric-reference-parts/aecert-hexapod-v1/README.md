# Aecert's Hexapod V1: parametric parts

Parametric solids of the printed parts of Ryan Mirch's Aecert's Hexapod V1, modelled from dimensions measured on its STLs (`hardware/aecert-hexapod-v1/stl/`), checked against them and exported as STEP. The build, the part contract and the gates are in the folder's README (`../README.md`).

## Source

- Repository: https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1
- Commit `ca5ed27` (2024-04-09), the one Spooder's movement code was ported from and whose STLs `hardware/aecert-hexapod-v1/` holds unchanged.
- Licence: upstream states no licence. These parts reproduce Aecert's design; they are not Spooder's own work, and no licence is granted for them here.

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

## Groups still to model

Each is a later change adding its part files, its STEP files, its table here and a `CHANGELOG.md` line:

- **Coxa:** Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector, Coxa Bearing Side.
- **Femur:** Femur Horn Side, Femur Bearing Side, Femur Support.
- **Tibia:** Tibia Left and Tibia Right (a mirrored pair), Tibia Support.
- **Feet:** Foot Left and Foot Right (a mirrored pair), Foot Shaft, Foot Tip.
- **Plates:** Leg Parts A, Leg Parts B and Foot Left Right, arrangements of the single parts.
