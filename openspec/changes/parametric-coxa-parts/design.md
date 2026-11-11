# Design

## Context

The foundation and the frame group are in `hardware/parametric-reference-parts/` (change `parametric-reference-parts`, archived 2026-11-11). Its README fixes the part contract (one file per part, `PARTS`, the import allowlist), how a part is added (measure with trimesh, model from features, `--check`, `mise run cad-reference`, the source README's table, a changelog line) and the gates (volume 0.5 %, deviation maximum 0.20 mm and 95th percentile 0.05 mm, bounding box 0.15 mm; a per-part exception only for an STL chord error above 0.15 mm or a named feature that feature-based modelling cannot match).

Measured on the coxa STLs (trimesh 4.5.3), all watertight, one body each:

| STL | Triangles | Volume (mm³) | Bounding box (mm) |
|---|---|---|---|
| Coxa Horn Connector | 12 602 | 7 606.75 | x 81.50 to 138.22, y −15.80 to 15.80, z −43.35 to −30.47 |
| Coxa Horn Side | 3 032 | 11 494.58 | x 476.56 to 544.81, y 112.77 to 160.79, z 0 to 8.30 |
| Coxa Bearing Connector | 13 744 | 6 712.26 | x 267.75 to 324.47, y 78.21 to 109.81, z 36.91 to 49.79 |
| Coxa Bearing Side | 3 148 | 13 940.38 | x 475.82 to 544.07, y 162.13 to 210.15, z 0 to 8.30 |

The sides are 68.25 × 48.02 × 8.3 mm, as `hardware/aecert-hexapod-v1/README.md` records (68.25 × 48 mm); the connectors are 56.72 × 31.6 × 12.88 mm.

`hardware/aecert-hexapod-v1/README.md` records the coxa link from these meshes: the tongues' insert holes 21.5 mm from the coxa axis, at the mid-height of the 8.05 mm ring; ears 30.10 mm either side of the side plates' 28.5 mm hole pair's middle; side plates 68.25 × 48 mm reaching 34.125 mm either side of that middle.

## Goals / Non-Goals

**Goals:**
- The four coxa parts as feature-built solids passing the global gates, exported as STEP that passes both import checks.
- The recorded coxa link geometry checked against the model, each disagreement stated.
- The author's informal awareness stated beside the licence sentence.

**Non-Goals:**
- The other groups (parallel changes) and any shared tooling, test or folder README change.
- Correcting `hardware/aecert-hexapod-v1/README.md` or the movement constants where the model disagrees: the disagreement is reported, and a correction is a follow-up.

## Decisions

### Each part modelled on its own

The brief allows the mirrored-pair mechanism only for true mirrors. Neither pair is one: the sides' volumes differ by 2 446 mm³ (21 %) and the connectors' by 894 mm³ (13 %), far beyond the 0.5 % volume gate, so a mirrored side would fail its own STL. Each of the four files declares one part with its own `build`. Shapes the parts share (the side plates' outline and 28.5 mm hole pair, the connectors' tongue, ring and insert hole, should the measurements show them equal to 0.01 mm) go in `parts/_coxa.py`, as the frame group's shared plan went in `_frame.py`; a value that differs between two parts stays in each part's own file.

### Where the group's records go

The group's parts table, its chord errors, any exception and the coxa link comparison go in the coxa section of `aecert-hexapod-v1/README.md`, in the frame section's form. The folder README (`../README.md`) is shared tooling documentation and is not edited: its chord-error table stays the frame group's, and its sentence "No part declares an exception yet" is left to a follow-up if this group needs one (see Assumptions).

### Checking the recorded coxa link geometry

Each recorded value is measured on the built solids, not on the meshes, with build123d in the build's pinned environment: the distance from the tongue's insert hole axis to the coxa axis (the ring's axis), the ring's height, the distance of each ear's hole from the middle of the side plate's 28.5 mm hole pair (and that pair's spacing), and the side plates' outline. Each is compared with the recorded value; a difference above 0.05 mm (the 95th percentile limit, and more than the 0.01 mm to which dimensions are measured) is a disagreement and is stated with both values. The measurement runs as a script outside the repository, since the part tests live in the shared `test_build.py`; its output is recorded in the source README.

## Risks / Trade-offs

- **The connectors are fine-meshed** (12.6 k and 13.7 k triangles in a 57 mm part), which suggests a horn spline or many small rounds. → A spline or round that feature-based modelling cannot meet within the global limits gets a per-part exception naming the feature and its size; the global limits stay.
- **Parallel groups edit the same README and CHANGELOG.** → This change touches only its own section, the licence sentence, the coxa line of "Groups still to model" and one changelog line, so merges conflict at most on adjacent lines.

## Assumptions

- The whole coxa group is the smallest coherent unit: the recorded coxa link geometry spans all four parts (the tongues and rings on the connectors, the ears and plates on the sides), and the foundation planned one change per group. The licence sentence is one line and rides with it.
- The two sides and the two connectors are modelled separately, not as mirrored pairs, from the volumes above.
- `test_build.py` is not edited: the brief forbids the shared tooling, and parallel changes would conflict in it. The frame group's tests filter on the frame files, so new part files do not break them. The coxa link check is therefore a recorded measurement, not a unit test.
- The chord error of each coxa STL is measured with `build.py --chord-error` and recorded in the source README's coxa section, not in the folder README's table.
- If a coxa part needs an exception, the folder README's "No part declares an exception yet" becomes stale; this change does not edit that file, and records the follow-up below.
- Where the model disagrees with `hardware/aecert-hexapod-v1/README.md`, the reference README is left unchanged (the reference folders stay unchanged) and the disagreement is stated in the source README and the completion report.
- The author's awareness is stated as given in the brief: the author knows about Spooder's derived models and has raised no concerns; it is informal and grants no licence.

## Follow-ups

- The femur, tibia, feet, plates and LilyGO groups (parallel changes).
- If any coxa part declares an exception: update the folder README's "No part declares an exception yet" and its chord-error table once the parallel groups have merged.
- If the coxa link comparison disagrees: correct `hardware/aecert-hexapod-v1/README.md` and, if a movement constant depends on the value (`body_geometry::UNDERSIDE_Z`), the constant, in a change of its own.
