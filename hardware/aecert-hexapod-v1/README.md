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

`BOM.md` lists the bought hardware (bearings, inserts, screws), derived from these parts, since upstream never published a list.

The sprung tip travels 5 mm: 40 g preload, about 330 g at full travel, so it bottoms out under the robot's standing load.
