# Design

## Context

The parametric reference parts were founded by `parametric-reference-parts` (archived 2026-11-11), which set the part contract, the gates, the STEP export and the import checks, and modelled the frame group. Its README (`hardware/parametric-reference-parts/README.md`) is the contract this change follows: "Part contract", "Adding a part", "Gates and tolerances" and "Per-part exceptions".

Measured on the feet STLs (trimesh 4.5.3), all watertight, one body each:

| STL | Triangles | Volume (mm³) | Bounding box (mm) |
|---|---|---|---|
| Foot Left | 14 038 | 3 313.33 | x 104.765 to 121.479, y 55.323 to 103.811, z 0 to 12.45 |
| Foot Right | 14 038 | 3 313.33 | x 127.411 to 144.126, y 54.253 to 102.742, z 0 to 12.45 |
| Foot Shaft | 1 544 | 2 055.77 | x 943.066 to 1018.402, y 322.978 to 334.419, z 9.788 to 19.576 |
| Foot Tip | 4 054 | 548.48 | x 558.866 to 569.261, y 122.80 to 133.20, z −52.187 to −42.685 |

First sections of Foot Left: from z 0 to 5 a plate whose sections show 22 loops (the outline and the flexure's cuts); from z 5 to about 6.5 a closed layer with rounded edges (many vertex levels between z 5.5 and 5.76); above it a small post, about 4.4 mm square at z 8, narrowing to about 3 mm at z 12.3. Foot Shaft lies along x, with a collar near x 965. Foot Tip narrows towards its top, a round or domed end, and holds a second loop (a bore) in its lower half.

## Goals / Non-Goals

**Goals:**
- Foot Left, Foot Right, Foot Shaft and Foot Tip as parametric parts that pass every global gate and import as one closed solid in FreeCAD and solidcraft.
- The flexure's path and beam widths as named dimensions, shared by both feet, so changing one and running `--check` rebuilds both feet with it.

**Non-Goals:**
- Foot Left Right (a later arrangement change), the ball foot, the other groups.
- Any edit to the shared tooling, its tests, the folder README or another group's files.
- Changing the spring: upstream's flexure is reproduced as it is.

## Decisions

### Files

```text
aecert-hexapod-v1/parts/
  _feet.py        the foot half: its named dimensions (outline, flexure, cap, post) and foot_half()
  foot_left.py    Foot Left: build() returns _feet.foot_half()
  foot_right.py   Foot Right: Foot Left's solid, mirrored and moved (below)
  foot_shaft.py   Foot Shaft
  foot_tip.py     Foot Tip
```

The foot half's dimensions live in `_feet.py`, not in `foot_left.py`, because both feet are built from them, as Frame Top and Frame Bottom share `_frame.py`. Each constant carries its source comment (the trimesh section, probe or circle fit, and where), measured on Foot Left's STL.

### The flexure

The serpentine is modelled as a path, not as a set of separate slots: named constants give its start, the number of turns, the pitch between runs, the run length, the turn radius and the beam width (and, where the STL's beams differ, the width of each kind of beam, such as the end beams joining the frame and the tip). The cuts between the beams follow from the path and the widths, so retuning the spring is a change of constants. Where the STL's turns are not exact arcs, the constant is the least-squares circle through their vertices and the comment says so, as for the frame.

### Foot Right

The two foot STLs are mirror images: Foot Right reflected across x and moved onto Foot Left lies within 0.0012 mm of it (20 000 area samples). Their bounding boxes give a mirror plane x = 124.445, but Foot Right's y range sits 1.069 mm lower than Foot Left's. A mirrored-pair declaration takes only a `Plane`, and a reflection across any plane cannot add that offset; a tilted plane would turn the part instead. The offset is far above the 0.15 mm bounding-box gate, so the mirrored-pair mechanism does not meet the gates.

Foot Right is therefore its own part file whose `build` imports `foot_left` (a sibling, allowed by the contract), mirrors its solid across `Plane.YZ.offset(MIRROR_X)` and moves it by `RIGHT_OFFSET_Y`, both measured from the two STLs' bounding boxes and comments saying so. Foot Right has no model of its own, so a retuned flexure changes both feet, which is what the mirrored pair would have given.

### Foot Shaft and Foot Tip

Each is modelled from its own sections in its STL's coordinates. Both look like turned parts (a shaft with a collar; a tip with a bore and a domed end), so the likely features are revolved profiles and extrusions; a feature that turns out to be freeform is handled as below.

### Gates and exceptions

Every part is held to the global gates. `build.py --chord-error` measures each STL's chord error once the parts are declared. A part whose chord error exceeds 0.15 mm gets the documented per-part exception for the tessellation gate, at its chord error plus 0.05 mm. A feature that feature-based modelling cannot match (a freeform surface) is recorded as a per-part exception that names the feature and its size, and the amount by which it exceeds the global limit; a global limit is never loosened. Both are listed in the feet group's table.

## Risks / Trade-offs

- **The flexure's thin beams and rounds** may defeat OpenCascade's fillets, as Frame Bottom's chamfers did → cut them with explicit profiles in the flexure sketch, as `_frame.chamfered` cuts wedges.
- **Run time:** the surface deviation gate on Foot Left's 14 038 facets adds to the build's ten minutes; acceptable for a gate run locally.
- **Merge conflicts:** the coxa, femur, tibia and LilyGO changes edit `aecert-hexapod-v1/README.md` and `CHANGELOG.md` in parallel → this change touches only its own section, its own line in "Groups still to model" and one changelog line.

## Assumptions

- The group is delivered as one change, all four parts, although the parts are independent: the programme adds one group per change (the archived change's Follow-ups, the source README's "Groups still to model"), and splitting it would leave a group half modelled with a second change for two small parts.
- Foot Right is a separate part file built from Foot Left's solid (Foot Right, above), not a mirrored-pair declaration, since the pair mechanism cannot carry the 1.069 mm offset.
- The foot half's dimensions are measured on Foot Left's STL; Foot Right is checked through the mirror against its own STL.
- No unit test is added: the brief limits this change to its part files, STEP files, README section and changelog line, and `test_build.py` is shared tooling. The gates check each part, and the retuning is checked by changing a flexure constant and running `--check` (tasks 2.3).
- The folder README's sentence "No part declares an exception yet" and its chord-error table belong to the shared README, which this change does not edit; if a foot needs an exception, the feet group's table records it and the folder README is left to a change that may edit it (Follow-ups).
- The changelog line goes under Unreleased, Added.

## Follow-ups

- Foot Left Right, the combined plate, as an arrangement of Foot Left and Foot Right (the plates change).
- If any foot declares an exception: update the folder README's "No part declares an exception yet" sentence and, if wanted, add the feet to its chord-error table.
