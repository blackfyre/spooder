# Tasks

All commands run in `hardware/parametric-reference-parts/` unless they start with `mise`.

## 1. Foot Shaft and Foot Tip

- [x] 1.1 Model Foot Shaft in `aecert-hexapod-v1/parts/foot_shaft.py` from named dimensions measured with trimesh (sections, probes, circle fits), each with its source comment, built from features in the STL's coordinates. Verify `uv run --locked --quiet --script build.py --check` passes Foot Shaft against `hardware/aecert-hexapod-v1/stl/Foot Shaft.stl` with the global limits, or with an exception declared as in design.md ("Gates and exceptions").
- [x] 1.2 Model Foot Tip in `aecert-hexapod-v1/parts/foot_tip.py` the same way. Verify `build.py --check` passes Foot Tip against its STL as in 1.1.

## 2. Foot Left and Foot Right

- [x] 2.1 Model the foot half in `aecert-hexapod-v1/parts/_feet.py`, the flexure as a path with named start, turns, pitch, run length, turn radius and beam widths (design.md, "The flexure"), and declare Foot Left in `foot_left.py`. Verify `build.py --check` passes Foot Left against its STL as in 1.1, and `grep -cE '^FLEXURE_[A-Z_]+ *=' aecert-hexapod-v1/parts/_feet.py` counts at least the six named flexure dimensions.
- [x] 2.2 Declare Foot Right in `foot_right.py` as Foot Left's solid mirrored across `Plane.YZ.offset(MIRROR_X)` and moved by `RIGHT_OFFSET_Y`, both measured from the two STLs' bounding boxes (design.md, "Foot Right"). Verify `build.py --check` passes Foot Right against its own STL as in 1.1.

## 3. Exceptions, documentation and tests

- [x] 3.1 Measure the four feet STLs' chord error with `uv run --locked --quiet --script build.py --chord-error` and declare any exception it requires (design.md, "Gates and exceptions"): above 0.15 mm, the `deviation_max` exception at the chord error plus 0.05 mm rounded up to 0.01 mm. A freeform feature that cannot be matched gets an exception for the failing gate only, its reason starting with "freeform". Verify `build.py --check` passes all four foot parts.
- [x] 3.2 Add the feet group's section to `aecert-hexapod-v1/README.md` (a table of the four parts with part file, STL, STEP, the measured gate values and any exception with its feature and amount; the flexure's named dimensions; how Foot Right is made), remove the Feet line from "Groups still to model", add one line under Unreleased, Added in `CHANGELOG.md` and, if any foot declares an exception, rewrite the folder README's sentence "No part declares an exception yet." to name the foot exceptions and point to the feet group's table. Verify `grep -c 'parts/foot_' aecert-hexapod-v1/README.md` counts four table rows, `grep -c '^- \*\*Feet:\*\*' aecert-hexapod-v1/README.md` prints 0, `grep -c 'No part declares an exception yet' README.md` prints 1 if no foot declares an exception and 0 otherwise, and `git diff --stat main -- CHANGELOG.md` shows one line added.
- [x] 3.3 Add `test_feet.py` with the feet group's tests (design.md, "Tests"): the four foot parts declared, each in its file and against its STL, the failure naming a missing part; the retuning, one flexure beam width in `_feet` raised by 0.3 mm and restored in a `finally`, both feet rebuilt differing in volume from their unchanged builds and each failing its volume or deviation gate; each foot's exceptions matching `chord_error.measure` on its STL as in 3.1 (a tessellation exception at the derived limit; any other only with a reason starting with "freeform", a limit above the global one, and its gate failing when the part is checked without it; none otherwise); and each foot exception's README row giving its amount over the global limit in the gate's unit (% for `volume`, mm otherwise). Verify `uv run --locked --quiet --script build.py --test` passes (its README test also checks each foot's row, feature and limit); that with `foot_tip.py` moved out of `parts/` the declaration test fails naming Foot Tip; that with one foot's exception limit changed by 0.01 mm (or, for a foot with none, a `deviation_max` exception added without a freeform reason) the exception test fails naming that part; and that with the amount removed from a foot exception's README row (if any foot declares one) the README amount test fails naming that part; then restore each.

## 4. Export

- [x] 4.1 Export the four parts and commit them with their STEP files. Verify `mise run cad-reference` passes with every FreeCAD and solidcraft import check reporting `PASS` for Foot Left, Foot Right, Foot Shaft and Foot Tip, a second run leaves `git status --porcelain` empty, and `git diff --name-only main` lists only this change's files.
