# Proposal

## Why

The parametric reference parts (`hardware/parametric-reference-parts/`) hold the frame group only. The feet carry Aecert's printed spring: Foot Left and Foot Right each contain a serpentine flexure that gives the sprung tip its 5 mm of travel. As a mesh the spring cannot be retuned; as a parametric solid its path and beam widths are named dimensions that can be changed and rebuilt. Foot Shaft and Foot Tip complete the sprung tip.

## What Changes

- **The feet group** in `hardware/parametric-reference-parts/aecert-hexapod-v1/`: Foot Left, Foot Right, Foot Shaft and Foot Tip, each as build123d code with named, sourced dimensions measured from its STL, built from features in the STL's own coordinates, passing the gates (the global limits, or a documented per-part exception), and exported as STEP that imports as one closed solid in FreeCAD and solidcraft.
- **The flexure as named dimensions:** the serpentine's path (its turns, pitch and run lengths) and its beam widths are constants in `parts/_feet.py`, shared by both feet.
- **Foot Right from Foot Left:** the two STLs are mirror images, but the right one also sits 1.07 mm further along y, which a mirror plane cannot express. Foot Right is therefore its own part file whose build mirrors Foot Left's solid and moves it, instead of a mirrored-pair declaration (design.md, "Foot Right").
- **Tests of the feet group** in a new `test_feet.py`, which `build.py --test` discovers by its name: the four parts declared, the flexure retuned through the shared model, and each foot's exception (or none) matching its STL's chord error.
- **Documentation:** the feet group's table in `aecert-hexapod-v1/README.md`, one `CHANGELOG.md` line and, only if a foot declares an exception, the folder README's sentence "No part declares an exception yet."

**Non-goals:**
- Foot Left Right, the combined plate, which a later change builds as an arrangement;
- the ball foot of upstream's later commit (`Foot Shaft (Ball)`, `Foot Tip (Ball)`), which `hardware/aecert-hexapod-v1/` does not hold;
- the coxa, femur, tibia and LilyGO groups, which other changes add in parallel;
- any change to the shared tooling (`build.py`, `discovery.py`, `gates.py`, `export.py`, `import_check.py`, `freecad_import.py`, `chord_error.py`) or its tests (`test_build.py`), and any edit to the folder README beyond that one sentence;
- retuning the spring: this change reproduces upstream's flexure.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `parametric-reference-parts`: adds the feet group, with the flexure's path and widths as named dimensions.

## Impact

- **New:** `aecert-hexapod-v1/parts/_feet.py`, `foot_left.py`, `foot_right.py`, `foot_shaft.py`, `foot_tip.py`; `test_feet.py`; `aecert-hexapod-v1/step/Foot Left.step`, `Foot Right.step`, `Foot Shaft.step`, `Foot Tip.step`.
- **Changed:** `aecert-hexapod-v1/README.md` (the feet group's section, and its line in "Groups still to model"), `CHANGELOG.md`, and the folder README's exception sentence only if a foot declares an exception.
- **Unchanged:** the shared tooling, the other groups' files, `hardware/aecert-hexapod-v1/`, firmware, desktop and CI.
- **Risk:** low. Nothing runs on the robot; the gates tie each solid to its STL.
