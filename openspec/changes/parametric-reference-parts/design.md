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
- `build`: a function returning the solid, in the STL's coordinates;
- `exceptions`: optional per-gate limits, each with the feature responsible and the reason (see Tolerances).

Named dimensions are module-level constants in the part file, each with a comment saying how it was measured (section or probe, and where), as `hardware/aecert-hexapod-v1/README.md` does. Helpers that one group needs go in that group's own `parts/_<group>.py` (not discovered); the shared tooling is not edited by group changes. The build fails on two declarations with the same name or the same STL.

A mirrored pair is one file declaring two parts: the left built from features, the right as `mirror()` of the left about the plane that maps one STL onto the other (found from the two STLs' bounding boxes and checked by the gates), each with its own STL. An arrangement plate (Leg Parts A, B, Foot Left Right) is one file declaring one part whose `build` imports the single parts' modules and places their solids; its gates run against the plate's STL like any part. Neither needs foundation code beyond `mirror` and placement, which build123d provides.

### Gates

`gates.py` runs, per part, in order, and collects every failure (the build reports all, then exits non-zero before writing):

1. **Solid:** the build result is exactly one `Solid`, `is_valid()`, and its shell closed.
2. **Volume:** |V − V_stl| ≤ 0.5 % of V_stl.
3. **Surface deviation:** the solid is tessellated at 0.005 mm chordal tolerance (so its own chord error is negligible). 100 000 points are sampled by area on each of the solid's mesh and the STL, with fixed seeds; each point's distance to the other mesh's closest point is taken. The union of both directions must have a maximum ≤ 0.20 mm and a 95th percentile ≤ 0.05 mm. The report gives both directions separately, so a missing feature (STL → part) is told apart from an extra one (part → STL).
4. **Bounding box:** each of the six faces within 0.15 mm of the STL's.

### Tolerances, against the STL's chord error

The STL's vertices lie on the true surface; its facets cut inside convex arcs and outside concave ones by the chord error (sagitta) r(1 − cos(θ/2)) for a facet spanning θ on radius r. A correct parametric solid therefore deviates from the STL by up to that sagitta on curved faces and by nothing on flat ones.

A first estimate from the frame STLs (each pair of adjacent facets meeting at 0.5–30°, facet width taken as the shorter triangle's height, r = w / (2 sin(θ/2))): the largest sagitta is 0.05 mm on Frame Top and Frame Bottom, 0.13 mm on Servo Mount, 0.003 mm on Bearing Axle Mount and 0.002 mm on Hubcap; most of each part's area is flat. This estimate is not reliable where a flat face meets a shallow draft or chamfer (a wide facet at a small angle reads as a large radius), so task 2.1 measures it properly: circles fitted to the vertex loops of each curved feature in trimesh sections give r and the segment count n, and the chord error r(1 − cos(π/n)).

- **Maximum 0.20 mm:** above the worst estimated chord error (0.13 mm) with 0.07 mm left for measurement rounding (dimensions are measured to 0.01 mm).
- **95th percentile 0.05 mm:** flat faces dominate every frame part's area, and a flat face measured correctly deviates by nothing; a face measured 0.1 mm off over more than 5 % of the area fails it.
- **Volume 0.5 %:** a 34-segment bore loses about θ²/6 ≈ 0.6 % of its own cross-section to its facets, a small fraction of a part's volume; a wrong wall thickness or a missing hole of any size that matters exceeds 0.5 %.
- **Bounding box 0.15 mm:** a true arc at the part's extreme can stand past the STL's vertices by its chord error (≤ 0.13 mm estimated).
- **STEP import volume 0.1 %** of the built solid: the STEP holds the same exact geometry; the tolerance covers the kernels' volume integration.

**Per-part exceptions.** If task 2.1 finds an STL whose own chord error exceeds 0.15 mm, that part's maximum becomes its chord error + 0.05 mm, declared as an exception naming the feature (tessellation, not modelling). If a part cannot meet a gate because a feature is organic or freeform, its exception names the feature and the amount by which it exceeds the global limit, here and in the source README. The global limits are not loosened. The build prints every exception it applies.

### STEP export and re-read

As the mount's `export.py`: each part's solid is written with `export_step` and a fixed header timestamp (so an unchanged part writes an unchanged file) into a staging folder; each file is re-read with `import_step` and must be one solid, volume within 0.1 % and bounding box within 0.05 mm of the built part's. Only then is each source's `step/` replaced, removing files of parts that no longer exist. The removal is per source, so one change's build never deletes another source's files.

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

The unit tests prove each gate fails on a bad shape and passes on a good one, with synthetic fixtures (a box with a bore against its own STL; the same with the bore missing, with a 0.3 mm offset face, with an extra boss, with two solids), that a mirrored pair is checked side by side, that discovery finds a part file dropped into a temporary source and ignores `_` files, that a duplicate name fails, that a failed re-read leaves `step/` unchanged, and that an absent tool reports `SKIPPED` rather than a pass.

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

## Follow-ups

Each is a later change adding one group under this foundation: its part files, STEP outputs, its section in the source README and a CHANGELOG line.
- **Coxa:** Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector, Coxa Bearing Side.
- **Femur:** Femur Horn Side, Femur Bearing Side, Femur Support.
- **Tibia:** Tibia Left, Tibia Right (a mirrored pair), Tibia Support.
- **Feet:** Foot Left, Foot Right (a mirrored pair), Foot Shaft, Foot Tip.
- **Plates:** Leg Parts A, Leg Parts B, Foot Left Right, as arrangements of the single parts (after the groups they place).
- **LilyGO case shell:** `prt0003_23.stl` and `prt0005_14(2).stl`, under `lilygo-t-display/` with its own README (MIT, commit `6f89cdd`).
