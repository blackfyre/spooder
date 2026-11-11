# Proposal

## Why

`parametric-reference-parts` laid the foundation for parametric solids of Aecert's printed parts and converted the frame group. The femur is the next group: its two side plates carry the femur servo's horn and the tibia servo's horn (and the bearings opposite them), so their axis-to-axis length is the femur's bore spacing in the printed parts. As build123d solids, checked against the STLs and exported as STEP, they can be measured and mated by dimension instead of by triangle, and that spacing becomes a named, checked constant. It describes the reference parts only: the movement code's femur link length (`FEMUR_LENGTH`, 116 mm in `leg-actuation`) is unchanged and not derived from it.

## What Changes

- **The femur group** under `hardware/parametric-reference-parts/aecert-hexapod-v1/parts/`: Femur Horn Side, Femur Bearing Side and Femur Support, one file each, each built from features with named dimensions measured from its STL with trimesh, in the STL's own coordinates; the two sides' shared plan (the bore spacing, the outline, the screw pattern) in `parts/_femur.py`.
- **The femur's axis-to-axis length** (the plates' bore spacing) as a named constant in `_femur.py`, from which both sides place their bores, and a group test (`test_femur.py`, run by the existing `build.py --test`) that measures the built solids' bore axes, checks them against it, and checks that changing it moves both sides' second bores.
- **STEP files** for the three parts in `aecert-hexapod-v1/step/`, passing the global gates and the FreeCAD and solidcraft import checks; any part whose STL chord error exceeds 0.15 mm, or whose feature cannot be matched, declares the documented per-part exception for that gate only.
- **Documentation:** the femur group's section and table in `aecert-hexapod-v1/README.md`, and one `CHANGELOG.md` line.

**Non-goals:**
- the coxa, tibia, feet, plate and LilyGO groups, which other changes add in parallel;
- any change to the shared tooling (`build.py`, `discovery.py`, `gates.py`, `export.py`, `import_check.py`, `freecad_import.py`), to `test_build.py` or to another group's files;
- changing `hardware/aecert-hexapod-v1/` (its README's femur figure included) or the firmware's `FEMUR_LENGTH`: the measured difference is recorded as a follow-up (design.md).

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `parametric-reference-parts`: adds the femur group requirement, with its named axis-to-axis length checked against the built solids.

## Impact

- **New:** `aecert-hexapod-v1/parts/_femur.py`, `femur_horn_side.py`, `femur_bearing_side.py`, `femur_support.py`; their three STEP files; `hardware/parametric-reference-parts/test_femur.py`.
- **Changed:** `hardware/parametric-reference-parts/aecert-hexapod-v1/README.md` (the femur section, and the femur line removed from "Groups still to model"), `CHANGELOG.md`.
- **Unchanged:** the shared tooling, the frame group, `hardware/aecert-hexapod-v1/`, firmware, desktop and CI.
- **Risk:** low. Nothing runs on the robot; the gates tie each solid to its STL.
