# Tasks

All commands run in `hardware/parametric-reference-parts/` unless they start with `mise`.

## 1. Foot Shaft and Foot Tip

- [ ] 1.1 Model Foot Shaft in `aecert-hexapod-v1/parts/foot_shaft.py` from named dimensions measured with trimesh (sections, probes, circle fits), each with its source comment, built from features in the STL's coordinates. Verify `uv run --locked --quiet --script build.py --check` passes Foot Shaft against `hardware/aecert-hexapod-v1/stl/Foot Shaft.stl` with the global limits, or with an exception declared as in design.md ("Gates and exceptions").
- [ ] 1.2 Model Foot Tip in `aecert-hexapod-v1/parts/foot_tip.py` the same way. Verify `build.py --check` passes Foot Tip against its STL as in 1.1.

## 2. Foot Left and Foot Right

- [ ] 2.1 Model the foot half in `aecert-hexapod-v1/parts/_feet.py`, the flexure as a path with named start, turns, pitch, run length, turn radius and beam widths (design.md, "The flexure"), and declare Foot Left in `foot_left.py`. Verify `build.py --check` passes Foot Left against its STL as in 1.1, and `grep -cE '^FLEXURE_[A-Z_]+ *=' aecert-hexapod-v1/parts/_feet.py` counts at least the six named flexure dimensions.
- [ ] 2.2 Declare Foot Right in `foot_right.py` as Foot Left's solid mirrored across `Plane.YZ.offset(MIRROR_X)` and moved by `RIGHT_OFFSET_Y`, both measured from the two STLs' bounding boxes (design.md, "Foot Right"). Verify `build.py --check` passes Foot Right against its own STL as in 1.1.
- [ ] 2.3 Check that the spring can be retuned. Verify that with one flexure beam width raised by 0.3 mm in `_feet.py`, `build.py --check` fails the volume or surface deviation gate for both Foot Left and Foot Right, and that after `git checkout -- aecert-hexapod-v1/parts/_feet.py` it passes again.

## 3. Export and documentation

- [ ] 3.1 Measure the four feet STLs' chord error and declare any exception it requires (design.md, "Gates and exceptions"). Verify `uv run --locked --quiet --script build.py --chord-error` prints a chord error for each foot part, and every foot part above 0.15 mm declares the tessellation exception at its chord error plus 0.05 mm.
- [ ] 3.2 Export the four parts and commit them with their STEP files. Verify `mise run cad-reference` passes with every FreeCAD and solidcraft import check reporting `PASS` for Foot Left, Foot Right, Foot Shaft and Foot Tip, and a second run leaves `git status --porcelain` empty.
- [ ] 3.3 Add the feet group's section to `aecert-hexapod-v1/README.md` (a table of the four parts with part file, STL, STEP, the measured gate values and any exception with its feature and amount; the flexure's named dimensions; how Foot Right is made), remove the Feet line from "Groups still to model", and add one line under Unreleased, Added in `CHANGELOG.md`. Verify `grep -c 'parts/foot_' aecert-hexapod-v1/README.md` counts four table rows, `grep -c '^- \*\*Feet:\*\*' aecert-hexapod-v1/README.md` prints 0, `git diff --stat main -- CHANGELOG.md` shows one line added, and `git diff --name-only main` lists only this change's files.
