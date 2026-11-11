# Design

## Context

The foundation (`openspec/changes/archive/2026-11-11-parametric-reference-parts/`, main spec `parametric-reference-parts`) discovers every part file in `aecert-hexapod-v1/parts/`, gates it against its STL, exports STEP and runs the FreeCAD and solidcraft import checks. A group adds its part files, its STEP files, its README section and a `CHANGELOG.md` line, and nothing else (folder README, "Adding a part"). The coxa, tibia, feet and LilyGO groups are being added in parallel by other changes, so this change keeps to files they do not touch.

The femur group is three STLs in `hardware/aecert-hexapod-v1/stl/` (probed with trimesh 4.5.3 for this proposal; all three are watertight):

| Part | Bounding box (mm) | Volume (mm³) | Features seen in horizontal sections |
|---|---|---|---|
| Femur Horn Side | x 329.588–469.574, y 156.827–186.827, z 0–8.35 | 18 622.7 | two r 10.10 horn bores at x 344.581 and 454.581 (y 171.827), each with four r 3.02 screw holes 6.8 mm off it; an r 1.60 hole at x 408.081; two lightening slots; chamfered outline |
| Femur Bearing Side | x 328.233–468.219, y 190.315–220.315, z 0–8.35 | 16 904.2 | two r 11.20 bearing pockets at x 343.226 and 453.226 (y 205.315) narrowing to r 9.0 at the bottom; an r 1.60 hole at x 406.726; two lightening slots; chamfered outline |
| Femur Support | x 261.847–295.045, y 51.765–108.465, z −0.675–6.096 | 7 266.9 | two 20 mm openings; an outline that widens with height (a curved or sloped profile) |

## Goals / Non-Goals

**Goals:** the three femur parts as feature-built solids passing the global gates and both import checks; the femur's axis-to-axis length as a named constant checked against the built solids.

**Non-Goals:** the other groups; any edit to the shared tooling, `test_build.py`, another group's files or `hardware/aecert-hexapod-v1/`; the firmware's link lengths.

## Decisions

### One change for the whole group

The three parts are one group in the source README and in the foundation's follow-ups, share one BOM line (the femur plates screw into the Femur Support) and one README section. Splitting them would give each part change the same section, CHANGELOG line and follow-up bookkeeping for no independence gained, so the group is the smallest coherent unit.

### Shared plan in `_femur.py`

Femur Horn Side and Femur Bearing Side are the same plate outline (139.986 mm by 30 mm, 8.35 mm thick) with the same bore spacing, the same lightening slots and the same middle hole, differing in their bores (horn recess with four screw holes against stepped bearing pocket) and their placement in the build plate. `_femur.py` holds the shared constants and builders (the outline, the slots, the bore positions relative to the first bore) and the axis-to-axis length; each part file holds its own origin (first bore centre, from circle fits) and its own bore features. `_femur.py` declares no part and imports only the allowlist.

### The axis-to-axis length

`_femur.py` defines `AXIS_TO_AXIS`, the distance between the two bore axes of either side plate, and both sides place their second bore `AXIS_TO_AXIS` along x from their first. It names the printed parts' geometry, not the movement's link length: `leg-actuation` keeps the femur link at 116 mm (`FEMUR_LENGTH`), and nothing in `core/` reads `AXIS_TO_AXIS`. Its value is the measured one: least-squares circle fits to the bores in horizontal sections every 0.5 mm give 454.581 − 344.581 = 110.000 mm on Femur Horn Side and 453.226 − 343.226 = 110.000 mm on Femur Bearing Side, on every section.

`test_femur.py`, a new file beside `test_build.py` that the existing `build.py --test` discovers (`test_*.py`), builds both sides, finds their bore axes from the built solids' cylindrical faces (not from the constants), and checks each side's two axes lie `AXIS_TO_AXIS` apart to 0.01 mm. It then rebuilds both sides with `AXIS_TO_AXIS` raised by 1 mm (loading `_femur.py` and the two part files afresh with the changed value, never writing a file) and checks both spacings rose by 1 mm, so a part that hard-codes its own spacing fails. It also checks the three femur part files declare exactly the three femur parts, one assertion per part whose message names it, as the frame group's test does. With the gates holding each bore to its STL within 0.20 mm, the constant is therefore tied to the STLs.

### Exceptions

Each part's STL chord error is measured with `build.py --chord-error` once its part file exists, and recorded in the source README's femur section. A part over 0.15 mm declares a tessellation exception at its chord error plus 0.05 mm; a feature that feature-based modelling cannot match (Femur Support's widening outline is the candidate) is modelled as closely as features allow and declares an exception for that gate only, naming the feature and the amount over the global limit. The global limits are not changed. The tessellation rule is the foundation's own (`hardware/parametric-reference-parts/README.md`, "Per-part exceptions"), which this change follows rather than revises.

## Risks / Trade-offs

- Femur Support's profile may be a freeform or not exactly circular curve. Mitigation: model it as the least-squares arc or loft through its sections, as the frame group did for its non-circular arcs, and fall back to a documented exception.
- The group's `--check` adds three parts to a run that already takes about ten minutes; the femur parts are smaller than the frame plates, so the added time is a few minutes.
- `test_femur.py` is a second test file; it is the group's own, so no parallel change touches it.

## Assumptions

- **The axis-to-axis length is 110.000 mm, not 108.** The brief asks for the 108 mm in `hardware/aecert-hexapod-v1/README.md` ("Measured on Spooder"). The STLs place both sides' bore axes 110.000 mm apart, and the gates hold the model to the STLs, so a model checked against 108 could not pass both. The constant takes the STL's value, and its comment cites the README's 108 and the difference. The 108 is most likely upstream's correction (`3c7eade`) carried into the "measured" column; the old robot is dismantled, so no physical measurement contradicts the STL. Correcting that README (and any downstream use of the length) is outside this change's files and is a follow-up.
- The femur check lives in a new `test_femur.py` rather than in `_femur.py` or `test_build.py`: a part file may not import the tooling and an assertion inside `build()` would only compare a constant with itself, while `test_build.py` is shared by every group running in parallel.
- The femur line is removed from "Groups still to model" in the source README, as part of the group's own section.
- Named, documented dimensions are held to the frame group's convention (module-level constants, each with its measurement in a comment) and checked in review, as the foundation does; a source-level check would be shared tooling, which this change does not touch.
- The dimensions in the context table are from this proposal's probe; the part files re-measure and record each constant with its source.
- **Parabolic rounds are features, not exceptions** (found while applying task group 1). Femur Support's widening sides and the two rounded ends of the plates' web recess are not circular: a circle fits their vertices only to 0.081 mm (Support) and 0.07 mm (recess), while a parabola fits them to 0.006 mm and 0.005 mm. A quadratic Bézier is exactly a parabola, so each is modelled as one (control point at the tangents' intersection, which for the recess is the corner of a square round the middle hole), and every femur part passes the global gates without an exception.
- **The femur table goes in with the parts** (task group 1, ahead of task 2.1). The existing `Documentation` unit test fails `build.py --test` for any declared part without a row in its source README, so task 1.3's Verify (`build.py --test` passes) needs the femur group's table rows. Group 1 adds the section heading and the table with the measured values; task 2.1 completes the section's prose, removes the femur line from "Groups still to model" and adds the `CHANGELOG.md` line.

## Follow-ups

- Correct the femur's measured length in `hardware/aecert-hexapod-v1/README.md` (108 against the parts' 110.000 mm), and decide with it whether `core/movement/kinematics.h`'s `FEMUR_LENGTH` (116, upstream's original) should follow; that is a movement change with its own checks.
- The other groups (coxa, tibia, feet, plates, LilyGO case shell) are their own changes.
