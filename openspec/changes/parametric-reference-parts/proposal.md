# Proposal

## Why

Spooder's mechanical reference is upstream's STL meshes (`hardware/aecert-hexapod-v1/stl/`, and LilyGO's case shell). A mesh cannot be edited by dimension: changing a hole, a wall or an arm length means remodelling the part by hand. Parametric solids, built from named dimensions and opened in FreeCAD and in solidcraft as STEP, let a part be changed and regenerated, and let Spooder's own parts be designed against exact geometry instead of triangles.

The programme converts every unique Aecert part and the LilyGO case shell. This change lays the foundation and converts the first group, the frame, so that later changes, run in parallel, can each add one further group without touching shared files beyond their own README section and CHANGELOG line.

## What Changes

- **A new parametric part set,** `hardware/parametric-reference-parts/`, following the electronics mount's tooling: uv inline-script dependencies locked with `.lock` files, build123d 0.13.0 and trimesh, check before writing, write into a staging folder and move only after re-reading.
- **One file per part, discovered by the build:** each source (for now `aecert-hexapod-v1/`) has a `parts/` folder; the build finds every part file there. There is no central registry. A mirrored pair is one file that models one side and mirrors it; both sides are checked.
- **Parts built from features** (sketches, extrusions, holes, chamfers, fillets) with named, documented dimensions measured from the STL with trimesh, in each STL's own coordinate frame. Not a mesh-to-solid conversion.
- **Gates per part, enforced by the build's check mode:** one valid closed solid; volume, symmetric surface deviation (maximum and 95th percentile) and bounding box within stated tolerances of the STL. A part over a gate writes nothing.
- **STEP export, committed,** each re-imported as one solid with matching volume in FreeCAD (headless) and in solidcraft-cli, each import check reporting `SKIPPED` clearly when its tool is absent.
- **The frame group:** Frame Top, Frame Bottom, Servo Mount, Bearing Axle Mount and Hubcap.
- **A mise task,** `mise run cad-reference`, that checks and exports; named in the root `CLAUDE.md`'s verification gates for when the new folder changes; and a `CHANGELOG.md` entry.

**Non-goals:**
- the coxa, femur, tibia and feet groups, the combined plates (Leg Parts A, Leg Parts B, Foot Left Right) and the LilyGO case shell: later changes, one group each (design.md, "Follow-ups");
- Aecert's extras (armour, attachment peg, roll cage, spikes, cup holder, stand) and `All Parts.3mf`;
- the electronics mount, which is already build123d with STEP;
- changes to the reference files, or print files (the reference STLs stay the print source);
- printability rules (overhangs, teardrops): the parts reproduce upstream's shapes as they are.

## Capabilities

### New Capabilities

- `parametric-reference-parts`: parametric solids of the reference printed parts: how they are built and discovered, the gates they must pass against their STL, the STEP export and its import checks in FreeCAD and solidcraft, and the documentation each source carries.

### Modified Capabilities

None.

## Impact

- **New:** `hardware/parametric-reference-parts/` (build, gates, export, import checks, tests, README) with `aecert-hexapod-v1/` (README, `parts/`, `step/`).
- **Changed:** `mise.toml` (`cad-reference`), `CLAUDE.md` (commands, verification gates, layout), `CHANGELOG.md`.
- **Unchanged:** `hardware/aecert-hexapod-v1/`, `hardware/lilygo-t-display/`, firmware, desktop and CI.
- **Dependencies:** build123d, numpy and trimesh (and whatever trimesh's surface queries need), pinned inline and locked. FreeCAD 1.1.4 (flatpak `org.freecad.FreeCAD`) and solidcraft-cli 0.30.1 are local, unpinned tools, used only by the import checks.
- **Risk:** low. Nothing runs on the robot; the gates tie each solid to its STL.
