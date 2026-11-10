# Design

## Context

The reference parts are meshes: `hardware/aecert-hexapod-v1/stl/` (upstream `ca5ed27`, no licence stated) and LilyGO's case shell in `hardware/lilygo-t-display/3d_file/` (MIT). Spooder's movement constants (`core/movement/body_geometry.h`, the pivots, `UNDERSIDE_Z`, the battery's position) were measured from those STLs in their own coordinates, as `hardware/aecert-hexapod-v1/README.md` records.

The electronics mount (`hardware/spooder-electronics-mount/`) already sets the tooling pattern: build123d 0.13.0 scripts with uv inline metadata and `.lock` files, a `--check` / `--test` / write mode, a staging folder re-read before it replaces `step/`, and a mise task (`cad`).

Measured on the frame group's STLs (trimesh 4.5.3), all watertight, one body each:

| STL | Triangles | Volume (mm³) | Bounding box (mm) |
|---|---|---|---|
| Frame Top | 8232 | 156 715.3 | x ±116.10, y ±112.58, z 16.00 to 25.75 |
| Frame Bottom | 9486 | 287 487.1 | x ±116.10, y ±112.58, z −15.75 to 16.00 |
| Servo Mount | 5024 | 12 182.1 | x 191.11 to 248.11, y 120.90 to 157.90, z 6.00 to 26.00 |
| Bearing Axle Mount | 5918 | 10 013.2 | x 345.97 to 380.97, y 116.39 to 153.39, z 0 to 19.10 |
| Hubcap | 1558 | 1 848.8 | x 146.04 to 172.04, y 181.73 to 207.73, z 45.22 to 48.97 |

## Goals / Non-Goals

**Goals:**
- A foundation into which later changes add one group each, in parallel, editing only their own part files, STEP outputs, their README section and a CHANGELOG line.
- The frame group, proving the pattern end to end: gates, STEP, FreeCAD and solidcraft import.

**Non-Goals:**
- The other groups and the LilyGO case (Follow-ups).
- Print files: the reference STLs stay the print source.
- Printability rules from the `spooder-printed-parts` skill (overhangs, teardrops, bridges): the parts reproduce upstream's shapes. Its toolchain, staging and pitfall rules apply.

## Decisions

### Folder layout

```text
hardware/parametric-reference-parts/
  README.md            how to add a part, the part contract, gates and tolerances, import checks, tool versions
  build.py, build.py.lock
  gates.py             the STL gates
  export.py            STEP export, staging, re-read, publish
  discovery.py         finding the sources and their parts, the allowlist scan, the part contract
  import_check.py      FreeCAD and solidcraft import checks
  freecad_import.py    run inside FreeCADCmd
  test_build.py        unit tests (build.py --test)
  aecert-hexapod-v1/
    README.md          source, commit, licence, one parts table per group
    parts/frame_top.py, frame_bottom.py, servo_mount.py, bearing_axle_mount.py, hubcap.py
    step/Frame Top.step, ...
```

One folder with a subfolder per source, rather than `hardware/aecert-hexapod-v1-parametric/` and a sibling for LilyGO, because both sources share the build and its gates; a sibling folder would either duplicate the tooling or depend on the other folder's. The LilyGO change adds `lilygo-t-display/` (its README, `parts/`, `step/`) with no shared edit. The reference folders stay unchanged.

STEP files keep upstream's names (`Frame Top.step` for `Frame Top.stl`), so each pairs with its STL by name.

### Part contract and discovery

The build imports every `*.py` in `<source>/parts/` whose name does not start with `_`, each under a unique module name, in sorted order. Each module defines `PARTS`, a list of part declarations, each with:
- `name`: upstream's file name without `.stl`;
- `stl`: the reference STL's path relative to the repository root;
- `build`: a function returning the solid, in the STL's coordinates; or, for the second side of a mirrored pair only, `mirror_of` (the first side's name) and `plane` instead, from which the build makes that side itself;
- `exceptions`: optional per-gate limits, each with the feature responsible and the reason (see Tolerances).

Named dimensions are module-level constants in the part file, each with a comment saying how it was measured (section or probe, and where), as `hardware/aecert-hexapod-v1/README.md` does. Helpers that one group needs go in that group's own `parts/_<group>.py` (not discovered); the shared tooling is not edited by group changes. The build fails on two declarations with the same name or the same STL.

`build` reads its module's constants when it is called, not at import, so a test can change one constant and rebuild. Before importing any file in `parts/` (the `_` helpers included), the build parses it with `ast` and fails, naming the file, unless every import is on an allowlist: `math`, `dataclasses`, `typing`, `build123d` (except `import_stl`, `import_3mf`, `import_step`, `import_brep` and `Mesher`), and modules in the same `parts/` folder; and unless it calls no `open`, `exec`, `eval`, `compile`, `__import__` or `getattr`, and none of the excluded build123d names appears anywhere in it, as a name, an imported name or an attribute (so `import build123d as b; b.import_stl(...)` fails too). The scan stops ordinary conversion, by mistake or by habit; deliberately obfuscated code is left to review, as every commit is reviewed against `REVIEW.md`. Reading a mesh (or any file) is how a part would be converted rather than modelled; with no file access and no I/O module, part code cannot read the STL. The declared `stl` path is a plain string the gates read.

Each discovered part file (not a `_` helper) declares one part, except a mirrored pair (two). The build fails on a discovered file declaring more, or none. In a pair the second declaration has `mirror_of` and `plane` and no `build`, so the second side is always the build's mirror of the first and cannot be modelled separately; a declaration with both `build` and `mirror_of`, or a `mirror_of` naming a part in another file, fails.

A mirrored pair is one file declaring two parts: the left built from features, the right declared with `mirror_of` the left and the plane that maps one STL onto the other (found from the two STLs' bounding boxes and checked by the gates), each with its own STL; the build makes the right as `mirror()` of the left's solid. An arrangement plate (Leg Parts A, B, Foot Left Right) is one file declaring one part whose `build` imports the single parts' modules and places their solids; its gates run against the plate's STL like any part. Neither needs foundation code beyond `mirror` and placement, which build123d provides.

### Gates

`gates.py` runs, per part, in order, and collects every failure (the build reports all, then exits non-zero before writing):

1. **Solid:** the build result is exactly one `Solid`, `is_valid()`, and its shell closed.
2. **Volume:** |V − V_stl| ≤ 0.5 % of V_stl.
3. **Surface deviation:** the solid is tessellated at 0.005 mm chordal tolerance (so its own chord error is negligible). On each side, the points are every vertex of that side's mesh after subdividing it (longest-edge bisection, `gates.bisected_vertices`; see Assumptions) until no edge exceeds 0.5 mm, plus 100 000 points sampled by area with a fixed seed; each point's distance to the other mesh's closest point is taken. The subdivided vertices bound the maximum regardless of where the random samples fall: every point of a facet whose edges are at most e lies within e/√3 of one of its vertices, and the distance to the other surface changes by at most the distance moved, so the true maximum deviation is at most the reported maximum + 0.29 mm. A feature present on one side only that stands more than 0.49 mm off the other surface is therefore always caught; one standing between 0.20 and 0.49 mm off is caught when a vertex or sample lands where it stands more than 0.20 mm off (an extra 1 × 1 × 0.3 mm boss has its own vertices 0.3 mm off the STL's face; a missing bore through a 3 mm wall leaves subdivided STL wall vertices up to 1.5 mm off the part), and otherwise only by the volume gate. This residual blind spot (a feature under 0.49 mm deep and smaller than the vertex spacing) is a stated limit, not a guarantee. The area samples weight the 95th percentile by area, which the subdivided vertices (uniform by edge, not area) would not. The union of both directions must have a maximum ≤ 0.20 mm (over vertices and samples) and a 95th percentile ≤ 0.05 mm (over the area samples). The report gives both directions separately, so a missing feature (STL → part) is told apart from an extra one (part → STL).
4. **Bounding box:** each of the six faces within 0.15 mm of the STL's.

### Tolerances, against the STL's chord error

The STL's vertices lie on the true surface; its facets cut inside convex arcs and outside concave ones by the chord error (sagitta) r(1 − cos(θ/2)) for a facet spanning θ on radius r. A correct parametric solid therefore deviates from the STL by up to that sagitta on curved faces and by nothing on flat ones.

A first estimate from the frame STLs (each pair of adjacent facets meeting at 0.5–30°, facet width taken as the shorter triangle's height, r = w / (2 sin(θ/2))): the largest sagitta is 0.05 mm on Frame Top and Frame Bottom, 0.13 mm on Servo Mount, 0.003 mm on Bearing Axle Mount and 0.002 mm on Hubcap; most of each part's area is flat. This estimate is not reliable where a flat face meets a shallow draft or chamfer (a wide facet at a small angle reads as a large radius), so task 2.1 measures it properly: circles fitted to the vertex loops of each curved feature in trimesh sections give r and the segment count n, and the chord error r(1 − cos(π/n)).

Measured (task 2.1, `build.py --chord-error`, `chord_error.py`): sections every 0.5 mm along each axis through each STL; in each section loop, runs of at least four segments whose vertices lie on one circle to 0.002 mm and turn at each vertex by the angle that circle gives them (to 10 %) are a curved feature, and its chord error is r(1 − cos(θ/2)) for its longest segment's angle θ, n = 2π/θ. Horizontal sections see the vertical cylinders and cones; vertical sections see rounds whose axes lie level (such as Frame Bottom's rear floor step), and count a run only where the facets it crosses stand square to the section, since a slantwise cut through a vertical cylinder or cone shows its facets stretched. No section sees the one round revolved about a vertical axis in its meridian, the 1 mm round at the foot of the Bearing Axle Mount's boss: a circle fitted directly to its vertices gives r 1.003 mm and a chord error of 0.003 mm, no more than that part's maximum below (found in review; the script does not measure it). Arcs that are not exactly circular in the STL (Frame Top's and Frame Bottom's arc C, fitted to 0.02 mm) are counted only where a run of them fits a circle to 0.002 mm; their facets' sagitta is under 0.03 mm. A unit test measures the five STLs and compares them with this table.

| Part | Chord error (mm) | Feature (r, n) |
|---|---|---|
| Frame Top | 0.050 | r 25.00, n 49 (the outline's side arcs) |
| Frame Bottom | 0.053 | r 38.82, n 60 |
| Servo Mount | 0.003 | r 2.20, n 56 |
| Bearing Axle Mount | 0.003 | r 1.60, n 54 |
| Hubcap | 0.002 | r 13.00, n 187 |

No part exceeds 0.15 mm, so no part declares a tessellation exception. The first estimate's 0.13 mm on Servo Mount came from a flat face meeting a chamfer, as cautioned above.

- **Maximum 0.20 mm:** above the worst estimated chord error (0.13 mm) with 0.07 mm left for measurement rounding (dimensions are measured to 0.01 mm).
- **95th percentile 0.05 mm:** flat faces dominate every frame part's area, and a flat face measured correctly deviates by nothing; a face measured 0.1 mm off over more than 5 % of the area fails it.
- **Volume 0.5 %:** a 34-segment bore loses about θ²/6 ≈ 0.6 % of its own cross-section to its facets, a small fraction of a part's volume; a wrong wall thickness or a missing hole of any size that matters exceeds 0.5 %.
- **Bounding box 0.15 mm:** a true arc at the part's extreme can stand past the STL's vertices by its chord error (≤ 0.13 mm estimated).
- **STEP import volume 0.1 %** of the built solid: the STEP holds the same exact geometry; the tolerance covers the kernels' volume integration.

**Per-part exceptions.** If task 2.1 finds an STL whose own chord error exceeds 0.15 mm, that part's maximum becomes its chord error + 0.05 mm, declared as an exception naming the feature (tessellation, not modelling). If a part cannot meet a gate because a feature is organic or freeform, its exception names the feature and the amount by which it exceeds the global limit, here and in the source README. The global limits are not loosened. The build prints every exception it applies.

### STEP export and re-read

As the mount's `export.py`: each part's solid is written with `export_step` and a fixed header timestamp (so an unchanged part writes an unchanged file) into a staging folder; each file is re-read with `import_step` and must be one solid, volume within 0.1 % and bounding box within 0.05 mm of the built part's. Only then is each source's `step/` replaced, removing files of parts that no longer exist. The removal is per source, so one change's build never deletes another source's files.

Unlike the mount's file-by-file replacement, the folder is replaced as a whole: the staging folder is a sibling of `step/` (same filesystem), `step/` is renamed to `step.old/`, the staging folder renamed to `step/`, then `step.old/` removed. If the second rename fails, `step.old/` is renamed back. A build that finds a leftover `step.old/` (an interrupted run) restores it when `step/` is missing and otherwise removes it, before writing. `step/` therefore never holds a mixture of old and new files. A run killed between the two renames leaves `step/` absent and the old files complete in `step.old/`; the next build (any mode) restores them before anything else, and `git status` shows the absence until then.

### Import checks

`import_check.py` runs after the export, on the committed `step/` files:
- **FreeCAD:** `flatpak run --command=FreeCADCmd org.freecad.FreeCAD freecad_import.py <files>`; the script imports each STEP with `Part.read`, and reports the number of solids, `isClosed()`, `isValid()` and the volume as JSON, with `FreeCAD.Version()`. Found when `flatpak info org.freecad.FreeCAD` succeeds, or `FreeCADCmd` is on `PATH`.
- **solidcraft:** `solidcraft-cli eval <file>` (its JSON gives `bodies[]` with `closed` and `volume_mm3`); one body, closed, volume within 0.1 %. Found when `solidcraft-cli` is on `PATH`; its version from `solidcraft-cli --version`.

Each prints one line per part and tool: `PASS` with the version, `FAIL` with the measured value, or `SKIPPED` with the reason (tool not found). The summary counts passes, failures and skips per tool; a skip never counts as a pass and never fails the build; a failure fails it. Checked locally on 2026-11-10: FreeCAD 1.1.4 (flathub) and solidcraft-cli 0.30.1 are installed; solidcraft-cli `eval` on an electronics mount STEP returns one closed body with its volume.

The flatpak's sandbox must read the STEP files: the installed FreeCAD flatpak has `filesystems=host`, so the files are passed by absolute path. A sandbox without that access fails to open them, which the check reports as a failure naming the file, not as a skip.

### Tooling and tests

`build.py` modes, as the mount: `--test` (unit tests), `--check` (gates only, no files written), and default (gates, export, re-read, publish, import checks). The mise task:

```toml
[tasks.cad-reference]
description = "Check the parametric reference parts against their STLs and write their STEP files"
dir = "hardware/parametric-reference-parts"
run = [
  "uv run --locked --quiet --script build.py --test",
  "uv run --locked --quiet --script build.py",
]
```

Dependencies pinned inline as the mount's (`requires-python = "==3.12.*"`, build123d 0.13.0, numpy 2.1.3, trimesh 4.5.3, `exclude-newer`), plus the exact version of whatever trimesh's closest-point query needs (rtree), locked with `uv lock --script`.

The unit tests prove each gate fails on a bad shape and passes on a good one, with synthetic fixtures (a box with a bore against its own STL; the same with the bore missing, with a 0.3 mm offset face, with an extra boss, with two solids), that a mirrored pair is checked side by side, that discovery finds a part file dropped into a temporary source and ignores `_` files, that a duplicate name, a file with too many declarations and a part or helper file reading a mesh each fail, that an extra small boss no area sample lands on fails through the subdivided vertices, that a `_` helper with no `PARTS` is accepted while a discovered file with none fails, that a mirrored pair's second side with its own `build` fails, that a failed re-read and a failed folder swap each leave `step/` complete, that each import mismatch (two solids, open shell, volume) fails and an absent tool reports `SKIPPED` rather than a pass, that the Aecert source declares exactly the five frame parts, and that raising a named Hubcap dimension makes its gates fail.

Root `CLAUDE.md`: the command list, the verification gates ("when `hardware/parametric-reference-parts/` changes, `mise run cad-reference`") and the layout. CI stays as it is, like `mise run cad`: the build would pass there with both import checks skipped.

## Risks / Trade-offs

- **Fillets and blends that are not circular** (a variable fillet, a spline profile) may not meet 0.05 mm at the 95th percentile. → Per-part exception, recorded with the feature and the amount.
- **Measurements fitted to the gates, not to design intent** (a 3.02 mm wall that was meant as 3.0): → dimensions are recorded as measured; where a round value meets the gates it is used, and the comment says so.
- **Frame Top and Frame Bottom are large and detailed** (8–9 k triangles, six arms, insert holes, slots). → They use the frame's six-fold and mirror symmetry: one arm modelled and patterned, which also keeps the movement constants (pivots at 0°, ±56°) visible as named angles.
- **Sampling noise** in the percentile gate. → Fixed seeds and 100 000 points each way make it repeatable; a part close to the limit shows the same value every run.
- **FreeCAD and solidcraft are unpinned**: a newer version may read STEP differently. → The versions checked are recorded in the README; the check prints the version it ran.

## Assumptions

- One folder, `hardware/parametric-reference-parts/`, with a subfolder per source, instead of the brief's example `hardware/aecert-hexapod-v1-parametric/`: the brief gives that name as an example, and the shared tooling needs one home (Folder layout).
- Only STEP is exported; no STL or 3MF, since the reference STLs remain the print files.
- The import checks run in the default build mode (after export), not in `--check`, because they read the committed STEP files.
- The STL gates compare in the STL's coordinates with no registration: a misplaced part is a failure, which protects the measurements that depend on those coordinates.
- A part file may declare more than one part only for a mirrored pair; every other part has its own file.
- The import checks are not added to CI: neither tool is installed there, and CI excludes `mise run cad` likewise.
- The commit that adds this change's code is reviewed against `REVIEW.md` as any other; the parts carry Aecert's design, and the README states that upstream gives no licence.

- A part declaration is a plain dict (`name`, `stl`, `build` or `mirror_of` and `plane`, `exceptions`), since part files may not import the tooling. An exception is `{gate: {"limit", "feature", "reason"}}` with the gate one of `volume` (limit in %), `deviation_max`, `deviation_p95` or `bbox` (mm); the solid gate has none.
- The allowlist also admits `from __future__ import annotations` (a compiler directive), and refuses a relative import, a submodule of an allowed module (`build123d.importers`) and any reference to `__builtins__`, besides the names listed above. A sibling in `parts/` is imported by its bare name; such modules are dropped from `sys.modules` after their source loads, so two sources may each have a helper of the same name.
- Part names and STLs must be unique across all sources, not only within one.
- The subdivision is longest-edge bisection of each facet on its own (`gates.bisected_vertices`), not `trimesh.remesh.subdivide_to_size`: both leave no edge over 0.5 mm, so the e/√3 bound holds, but splitting every edge turns long thin facets into slivers. Measured on Frame Bottom's STL, `subdivide_to_size` gives 7.8 million vertices and bisection 1.7 million; trimesh's closest-point query runs at about 25 000 points/s, so the former would take minutes per direction. The sub-triangles need not form a conforming mesh, because only their corners are used. The closest-point query needs `rtree`, pinned at 1.4.1.
- The area samples use seed 20261110 on the part side and 20261111 on the STL side.
- When the solid gate fails, the other gates are not run for that part (they need one closed solid), and the report says so.
- Staging and re-reading cover every source before any `step/` is replaced, so a failed re-read in one source writes nothing anywhere. Stale `.step-staging-*` folders from a killed run are removed with the leftover `step.old/`. A source with no parts keeps no empty `step/`.
- FreeCADCmd opens any further argument as a document, so `freecad_import.py` gets its files in `SPOODER_STEP_FILES` (passed into the flatpak with `--env=`), and prints its JSON after a marker line. The flatpak has its own `/tmp`, so the real-tool test stages its STEP under the tooling folder, not in `/tmp`. FreeCADCmd on `PATH` is preferred to the flatpak.

- The frame group's dimensions are measured as `hardware/aecert-hexapod-v1/README.md` describes (trimesh sections, probes and circle fits), recorded in each part file beside its constant. Where an STL's curve is not exactly circular (Frame Top's and Frame Bottom's arc C inside the rim and walls, Frame Bottom's second rear arc), the constant is the least-squares circle through its vertices, and the comment says so; every other arc fits its vertices to 0.002 mm.
- Frame Top and Frame Bottom share their plan in `parts/_frame.py` (the outline, the arms placed at 0° and ±56° with the corner arms 10.53 mm further out, the slots, the mount holes, the open middle, the frame screws), since both STLs agree on every shared value to 0.001 mm.
- Frame Bottom's chamfers are cut as one wedge per convex horizontal edge (`_frame.chamfered`), not with OpenCascade's chamfer, which fails on its walls (thin walls, tips of r 0.7 under a 1 mm chamfer, tangent joins). Where a chamfered edge meets a re-entrant corner the two wedges' meet is cut too; where it runs tangent into an edge that is not chamfered its wedge runs on 6 mm; a chamfer on a face stops at the walls standing on it. The STL's own exceptions are modelled as it has them: no chamfer on the walls' top edges facing the open middle and the open rear, on the floor hole's top edges, on the corner arms' slot ends or on the rear floor's outer edge, and a 0.75 mm round (not a chamfer) on the rear floor's step.
- The test that the Aecert source declares "exactly the five frame parts" checks the five frame part files declare exactly those five parts, and fails naming any missing one, so that a later group's part files do not need the test changed.
- `chord_error.py` chains each section's segments itself, since trimesh's section loops need networkx, which the build does not pin.

## Follow-ups

Each is a later change adding one group under this foundation: its part files, STEP outputs, its section in the source README and a CHANGELOG line.
- **Coxa:** Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector, Coxa Bearing Side.
- **Femur:** Femur Horn Side, Femur Bearing Side, Femur Support.
- **Tibia:** Tibia Left, Tibia Right (a mirrored pair), Tibia Support.
- **Feet:** Foot Left, Foot Right (a mirrored pair), Foot Shaft, Foot Tip.
- **Plates:** Leg Parts A, Leg Parts B, Foot Left Right, as arrangements of the single parts (after the groups they place).
- **LilyGO case shell:** `prt0003_23.stl` and `prt0005_14(2).stl`, under `lilygo-t-display/` with its own README (MIT, commit `6f89cdd`).
