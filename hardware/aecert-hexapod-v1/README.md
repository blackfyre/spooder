# Aecert's Hexapod V1: printed parts

The mechanical design Spooder is built from: Ryan Mirch's [Aecert's Hexapod V1](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1). These files are the author's work, copied unchanged.

## Source

- Upstream commit `ca5ed27` (2024-04-09), the one Spooder's movement code was ported from.
- `stl/` is upstream's `CAD STLS V2/`; `Aecerts Hexapod V1.3mf` is the whole robot as one project. File names are upstream's.
- Upstream's later commits change only the feet: `3c7eade` (2024-11-13) adds a ball foot (`Foot Shaft (Ball).stl`, `Foot Tip (Ball).stl`), not included here. Every other part is identical in upstream's last commit.

## Parts

- **Frame:** `Frame Top`, `Frame Bottom`, `Servo Mount`, `Bearing Axle Mount`, `Hubcap`.
- **Legs:**
  - coxa: `Coxa Horn Connector`, `Coxa Horn Side`, `Coxa Bearing Connector`, `Coxa Bearing Side`;
  - femur: `Femur Horn Side`, `Femur Bearing Side`, `Femur Support`;
  - tibia: `Tibia Left`, `Tibia Right`, `Tibia Support`;
  - the plates `Leg Parts A` and `Leg Parts B`.
- **Feet:** `Foot Left`, `Foot Right` (`Foot Left Right` together); the sprung tip `Foot Shaft` and `Foot Tip`.
- **Extras:** armour (`Armor *`, `Attachment Peg`), `Roll Cage*`, `Spikes`, `Cup Holder*`, and the display stand (`Stand *`). `All Parts.3mf` holds the full plate set.

## Geometry against the firmware

The firmware's link lengths came from upstream's code at this commit (coxa 41, femur 116, tibia 183 mm). Measured on these parts, axis to axis, they differ:

| Link | Firmware | Upstream's correction (`3c7eade`) | Measured on Spooder |
|---|---|---|---|
| Coxa | 41 | 46 | not yet; about 46.9 from these parts (coxa axis to the femur servo's opening) |
| Femur | 116 | 108 | 108 |
| Tibia (to the sprung tip) | 183 | 200 (ball foot) | 195 at rest, 190 bottomed out |

The coxa pivots (`core/movement/body_geometry.h`) are measured from `Frame Top.stl`, `Frame Bottom.stl` and `Bearing Axle Mount.stl`:

- Each leg's Bearing Axle Mount screws to a pair of frame holes 28.5 mm apart, square to the leg, 87.60 mm (middle legs) or 98.13 mm (corner legs) from the frame's centre, at exactly the gait's mount angles (0°, ±56° from the sides).
- The mount holds the coxa axle centred between its screws, 9.90 mm off their line, on the side where the mount reaches 28.5 mm. Facing outward, that side ends flush with the frame's arm (116.10 and 126.63 mm out); the MG996R's off-centre shaft in its Servo Mount sits the same 9.9 mm off the line.
- So the pivots are 97.50 mm (middle) and 108.03 mm (corners) from the centre: (±97.50, 0) and (±60.41, ±89.56). Not yet checked on the built robot.

The underside (`body_geometry::UNDERSIDE_Z`, −24.2 mm), the lowest point of the body below the pivots and the first thing to touch the ground as the level body is lowered, is measured from the same STLs with `Frame Bottom.stl`, `Frame Top.stl`, `Servo Mount.stl`, the coxa link's parts (`Coxa Horn Connector`, `Coxa Bearing Connector`, `Coxa Horn Side`, `Coxa Bearing Side`, `Bearing Axle Mount`, `Hubcap`) and the MG996R model (`hardware/mg996r`), read with trimesh (sections and vertical probes); the assembly is `BOM.md`'s reading. Frame Top and Frame Bottom share one coordinate frame (their z below); the other parts are placed by their mating faces:

- At each arm, Frame Bottom spans z −15.75 to 1.0 and Frame Top 16.0 to 25.75: the coxa Servo Mount's 15 mm ring fills z 1 to 16, and its 5 mm ear pads (57 mm long, end to end with Frame Bottom's 57.15 mm slot) reach down to −4. The coxa servo hangs from them shaft down: its bottom at 24.4, its spline tip at −20.0, 4.25 mm below Frame Bottom.
- The coxa link is two vertical side plates (68.25 × 48 mm) clamping the femur servo's Servo Mount through their own 28.5 mm hole pair, joined by the two connectors' 15 mm tongues at ears 30.10 mm either side of that pair's middle: the Coxa Horn Connector on the coxa servo's horn below Frame Bottom, the Coxa Bearing Connector on the bearing above Frame Top. Each tongue's insert hole lies at its ring's mid-height (4.025 mm from either face of the 8.05 mm ring), 21.5 mm from the coxa axis; with the femur shaft 25.5 mm further out, on the pocket's centre line, the coxa link is 47.0 mm (the 46.9 above).
- The two connectors fix the link's height within about ±0.6 mm: the Horn Connector's top must clear Frame Bottom (−15.75) and the Bearing Connector's ring must clear the Bearing Axle Mount's 9.4 mm base on Frame Top (35.15). The femur shaft sits 9.9 mm off the plates' hole-pair middle, as in every Servo Mount. Taken below it, the femur axis lies at z −0.2 ± 0.6, on the frame STLs' own origin, so the pivots' z = 0 is that origin. Taken above it, it would lie at 19.6 with no such match. The plates are symmetric, so this is a reading, not a measurement: the femur servo's cable would leave upwards, towards the electronics on Frame Top.
- The plates reach 34.125 mm either side of the hole-pair middle, so their lower ends lie 34.125 − 9.9 = 24.2 mm below the pivots, flush with the Horn Connector's underside (its ring 4.025 mm below its tongue hole, which is 30.10 below the middle). That ring sits under each pivot and the plates turn with the coxa, so the underside is the plane z −24.2 under every coxa angle, and the six Coxa Horn Connectors (with the plates' lower ends) touch the ground first, in a hexagon through the pivots. Frame Bottom's underside lies 8.45 mm higher, at −15.75; nothing else of the body reaches below −18.8 (the femur's Bearing Axle Mount).
- Not yet checked on the built robot. Taking the femur shaft above the hole-pair middle would put the underside at −44.0: a measured value replaces the constant.

`BOM.md` lists the bought hardware (bearings, inserts, screws), derived from these parts, since upstream never published a list.

The sprung tip travels 5 mm: 40 g preload, about 330 g at full travel, so it bottoms out under the robot's standing load.
