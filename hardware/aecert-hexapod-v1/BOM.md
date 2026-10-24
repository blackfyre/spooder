# Hardware bill of materials (derived)

Upstream never published a hardware list. This one is derived from the STLs in `stl/`: every hole, insert pocket and bearing seat measured, and the screw lengths taken from the parts each screw passes through. It covers the walking robot only (not the armour, roll cage, spikes, cup holder or stand). The electronics mount's own hardware is listed with the mount.

## How the parts go together

The STLs do not show their assembly; this is the reading that every hole spacing and wall height agrees with.

- Every servo sits in a **Servo Mount**: a ring whose 15 mm walls carry a heat-set insert at each end of two through holes 28.5 mm apart. It is clamped between two plates with the same 28.5 mm hole pair: Coxa Horn Side and Coxa Bearing Side, Tibia Right and Tibia Left, or Frame Top and Frame Bottom.
- A **Bearing Axle Mount** sits against one of those plates, behind the servo. It is a cup with a printed Ø7.9 mm axle, opposite the servo's output shaft. The output link carries the horn on one side and a 608 bearing on the axle on the other:
  - coxa: Coxa Horn Connector (horn) and Coxa Bearing Connector (bearing);
  - femur: Femur Horn Side (horn) and Femur Bearing Side (bearings).
- **Coxa joints:** the servo's horn faces down, below Frame Bottom, and the Bearing Axle Mount sits on Frame Top. Each coxa axis is 9.9 mm outboard of its 28.5 mm hole pair: at (±97.5, 0) for the middle legs and (±60.4, ±89.6) for the corners.
- The spacers between side plates are all 15 mm: the connector tongues, the Tibia Support and the two foot halves stacked.
- **The sprung foot is printed:** Foot Left and Foot Right each contain a serpentine flexure. No spring is bought.

## Bill of materials

Confidence: **high** when the geometry fixes the item; **medium** when it depends on the assembly reading above; **low** when the meshes leave a real choice.

| Item | Size | Qty | Where | Confidence |
|---|---|---|---|---|
| Ball bearing | 608 (8 × 22 × 7), ZZ or 2RS | 18 | 1 in each Coxa Bearing Connector, 2 in each Femur Bearing Side | high |
| Heat-set insert | M3, for a Ø4.4 hole, ≤ 4 mm long (fits every hole) | 166, + 8 for the electronics mount | 4 per Servo Mount (72); 4 per pair of coxa connector tongues (24); 4 per Tibia Support (24); 2 per Femur Support (12); 1 at the foot end of each Tibia Right (6); 1 in each axle tip (18); 10 in Frame Bottom | high for the positions, medium for two per through hole |
| Screw M3 × 6 | | 18 | Hubcap onto the axle-tip insert | medium |
| Screw M3 × 8 | | 6 | Frame Top onto Frame Bottom, at the 4 mm thick points | medium (8–10 fits) |
| Screw M3 × 10 | | 12 | Femur plates into the Femur Support | medium |
| Screw M3 × 12 | | 72 | Coxa Horn Side to the Servo Mount (12); Tibia Right to the Servo Mount (12); coxa side plates to the connector tongues (24); tibia plates to the Tibia Support (24) | medium (12–14 fits) |
| Screw M3 × 14 | | 4 | Frame Top onto Frame Bottom, at the 9.8 mm thick points | medium (14–16 fits) |
| Screw M3 × 22 | | 36 | Bearing Axle Mount and Coxa Bearing Side, or Tibia Left, into the Servo Mount (9.4 + 8.3 mm), 24; coxa Servo Mounts through Frame Bottom (16.8 mm), 12 | medium (22–25 fits) |
| Screw M3 × 22–25 | | 12 | Coxa Bearing Axle Mounts and Frame Top into the coxa Servo Mounts (9.4 + 9.8 mm) | medium |
| Screw M3 × 25 | | 6 | Foot: Tibia Left and both foot halves into the insert in Tibia Right | medium (25–28 fits) |
| Servo mounting screws | self-tapping, about ST2.2–2.6 × 8–10 | 72 | Servo ears into the Servo Mount's Ø2.1 pilot holes | medium; usually supplied with the MG996R |
| Horn screws | see open question 1 | 72 | 4 per horn: Coxa Horn Connector, both ends of Femur Horn Side | low |
| Horn centre screw | M3 × 10–12 | 18 | Through the 2.1 mm floor of each horn recess into the servo spline | low; the stock screw may be too short |
| Self-tapping M2 × 4–6 | | 12 | 2 per Tibia Support, Ø1.6 pilot holes; use unknown | low |

In total, about 178 M3 screws, plus the servo-ear, horn and horn-centre screws.

**Head type.** Every M3 clearance hole has a 45° chamfer 1.1 mm deep on the head side, which suggests countersunk heads (ISO 10642). Socket heads also work. The lengths above are measured from the head face, so they hold for either type.

## Open questions

1. **Horn screws.** The Ø20.2 horn recesses have four Ø3.7–3.9 holes on a 6.8 mm radius under Ø6.0 pockets, which suits a round disc horn. With the stock plastic MG996R horn they take M2 self-tapping screws; with a tapped aluminium 25T disc, M3 × 5–8.
2. **Inserts per through hole.** Two per 15 mm hole, one from each end, is assumed. Single long inserts would cut the count by 120.
3. **The Ø1.6 holes in the Tibia Support** and the **Ø17 hole at Frame Top's centre** have no known use.
4. **Insert fit.** Every insert hole is Ø4.4: check that the chosen insert suits it. The axle-tip insert's wall is only 1.75 mm thick, so set it carefully.
