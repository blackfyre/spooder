# Proposal

## Why

The parametric reference parts (`hardware/parametric-reference-parts/`, change `parametric-reference-parts`) hold only the frame group. The coxa link, the four parts between the frame and the femur servo, is the next group: Spooder's underside (`body_geometry::UNDERSIDE_Z`) and the measured coxa length (about 46.9 mm against the firmware's 41) come from these parts, and `hardware/aecert-hexapod-v1/README.md` records their geometry from the meshes only. Exact solids let that geometry be read from named dimensions and let Spooder's own parts mate against it.

## What Changes

- **The coxa group** in `hardware/parametric-reference-parts/aecert-hexapod-v1/parts/`: Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector and Coxa Bearing Side, each built from features with named, sourced dimensions measured from its STL with trimesh, in the STL's own coordinates, and each passing the global gates. The two sides and the two connectors are not mirrors of each other (their STL volumes differ: 11 494.6 against 13 940.4 mm³ for the sides, 7 606.8 against 6 712.3 mm³ for the connectors), so each part is modelled on its own; shared shapes go in `parts/_coxa.py`.
- **STEP files** for the four parts in `aecert-hexapod-v1/step/`, each passing the FreeCAD and solidcraft import checks.
- **The source README** (`aecert-hexapod-v1/README.md`) gains a coxa group table, the comparison of the built parts with the coxa link geometry that `hardware/aecert-hexapod-v1/README.md` records, and, beside the sentence that upstream states no licence, that the author knows about Spooder's derived models and has raised no concerns (informal, not a licence). The coxa line leaves "Groups still to model".
- **A `CHANGELOG.md` line.**

**Non-goals:**
- the femur, tibia, feet, plate and LilyGO groups, added by parallel changes;
- any change to the shared tooling (`build.py`, `discovery.py`, `gates.py`, `export.py`, `import_check.py`, `freecad_import.py`), its tests, the folder README or the global gate limits;
- any change to `hardware/aecert-hexapod-v1/` (its STLs and README stay unchanged, including where the model disagrees with them) or to the movement constants.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `parametric-reference-parts`: adds the coxa group, and the Aecert source README's licence statement gains the author's informal awareness.

## Impact

- **New:** four part files and `_coxa.py` in `hardware/parametric-reference-parts/aecert-hexapod-v1/parts/`, four STEP files in its `step/`.
- **Changed:** `hardware/parametric-reference-parts/aecert-hexapod-v1/README.md` (licence sentence, coxa section, groups still to model), `CHANGELOG.md`.
- **Unchanged:** the shared tooling, the frame group, firmware, desktop and CI.
- **Risk:** low. Nothing runs on the robot; the gates tie each solid to its STL.
