# Parametric reference parts

Parametric solids of the reference printed parts that Spooder is built from, modelled in build123d from named, measured dimensions, checked against the reference STL meshes, and exported as STEP files that open as one closed solid in FreeCAD and solidcraft.

The reference folders (`hardware/aecert-hexapod-v1/`, `hardware/lilygo-t-display/`) stay unchanged, and their STLs remain the print files. Nothing here is printed: the STEP files are for CAD, measuring and mating Spooder's own parts.

## Layout

```text
hardware/parametric-reference-parts/
  README.md            this file
  build.py, build.py.lock   the build (modes below), its pinned dependencies
  discovery.py         finding the sources and their parts, the allowlist scan, the part contract
  gates.py             the gates against the STL
  export.py            STEP export, staging, re-read and publish
  import_check.py      the FreeCAD and solidcraft import checks
  freecad_import.py    run inside FreeCADCmd
  chord_error.py       the STLs' own chord error (build.py --chord-error)
  test_build.py        the unit tests (build.py --test)
  aecert-hexapod-v1/   a source: Aecert's Hexapod V1
    README.md          the source, its commit and licence, a parts table per group
    parts/             one file per part; _*.py are helpers
    step/              the committed STEP files, named as the STLs
```

A source is any folder here that holds a `parts/` folder. Each source has its own `step/`, and a build never touches another source's.

## Part contract

The build imports every `<source>/parts/*.py` whose name does not start with `_`, in sorted order. Each such file defines `PARTS`, a list of plain dicts:

| Key | Value |
|---|---|
| `name` | Upstream's file name without `.stl`, such as `"Frame Top"`. The STEP file is `<name>.step`. |
| `stl` | The reference STL, relative to the repository root, such as `"hardware/aecert-hexapod-v1/stl/Frame Top.stl"`. |
| `build` | A function that returns the solid, in the STL's own coordinates. |
| `mirror_of`, `plane` | Instead of `build`, for the second side of a mirrored pair only: the first side's name and the build123d `Plane` that maps one side onto the other. |
| `exceptions` | Optional per-part gate limits (see "Per-part exceptions"). |

The build enforces these rules, and fails naming the file:

- Each discovered file declares exactly one part, or two for a mirrored pair. A file with `PARTS = []`, with no `PARTS`, or with more declarations fails.
- In a pair, the second declaration has `mirror_of` and `plane` and no `build`. A `mirror_of` that names a part in another file fails.
- Names and STLs are unique across all sources.
- Helpers in `parts/_<group>.py` are not discovered, and need no `PARTS`.
- Before any file in `parts/` is imported (the helpers included), it is parsed, and it may import only `math`, `dataclasses`, `typing`, `build123d`, `from __future__ import annotations` and its siblings in `parts/` (by bare name). It may not call or name `open`, `exec`, `eval`, `compile`, `__import__`, `getattr` or `__builtins__`, nor name build123d's importers (`import_stl`, `import_3mf`, `import_step`, `import_brep`, `Mesher`) in any form, including `b.import_stl` after `import build123d as b`. A relative import and a submodule of an allowed module (`build123d.importers`) fail too.

So part code cannot read the STL it is checked against: a part is modelled from features (sketches, extrusions, holes, chamfers, fillets), not converted from the mesh. The scan stops conversion by mistake or habit; deliberately obfuscated code is left to review.

Each dimension is a module-level constant with a comment saying how it was measured (a trimesh section or probe, and where), as `hardware/aecert-hexapod-v1/README.md` records its measurements. `build` reads the constants when it is called, so changing one and running `--check` rebuilds the part with it.

## Adding a part

1. Measure the STL with trimesh (sections, probes, circle fits), in its own coordinates.
2. Add `<source>/parts/<part_name>.py`, with the measured constants, `build()` and its declaration. A shortened `aecert-hexapod-v1/parts/hubcap.py` (the real one adds the chamfer, the pocket and the countersunk hole):

   ```python
   from build123d import Align, Cylinder, Pos

   # The disc's axis and its underside. Source: circle fits, z 45.3 to 48.95;
   # the downward face at z 45.225.
   CENTRE_X = 159.037
   CENTRE_Y = 194.727
   BASE_Z = 45.225
   # Source: the upward face at z 48.975, less the base.
   THICKNESS = 3.75
   # Source: the outer circle fit below the chamfer, z 45.3 to 47.6.
   RADIUS = 13.0


   def build():
       return Pos(CENTRE_X, CENTRE_Y, BASE_Z) * Cylinder(RADIUS, THICKNESS,
                                                         align=(Align.CENTER, Align.CENTER, Align.MIN))


   PARTS = [
       {
           "name": "Hubcap",
           "stl": "hardware/aecert-hexapod-v1/stl/Hubcap.stl",
           "build": build,
       },
   ]
   ```

3. Run `uv run --locked --quiet --script build.py --check` until every gate passes.
4. Run `mise run cad-reference`, which writes `<source>/step/<name>.step`, and commit the part file with its STEP file.
5. Add the part to its group's table in the source README, and a line to `CHANGELOG.md`.

No other file changes: there is no registry of parts. Helpers that only one group needs go in that group's `parts/_<group>.py`; the shared tooling is not edited for a group.

A **mirrored pair** (such as Tibia Left and Tibia Right) is one file declaring both sides, each with its own STL. The first is modelled; the second is the build's mirror of it:

```python
from build123d import Plane

PARTS = [
    {"name": "Tibia Left", "stl": "hardware/aecert-hexapod-v1/stl/Tibia Left.stl", "build": build},
    {"name": "Tibia Right", "stl": "hardware/aecert-hexapod-v1/stl/Tibia Right.stl",
     "mirror_of": "Tibia Left", "plane": Plane.YZ.offset(MIRROR_X)},
]
```

The plane is found from the two STLs' bounding boxes, and the gates check it: each side is checked against its own STL.

An **arrangement plate** (Leg Parts A, Leg Parts B, Foot Left Right) is one file declaring one part whose `build` imports the single parts' modules from `parts/` and places their solids; its gates run against the plate's STL like any part.

## Gates and tolerances

For each part, in order, the build checks, and collects every failure before it exits non-zero with nothing written:

1. **Solid:** the result is exactly one valid `Solid` with a closed shell. When this fails, the other gates are not run for that part.
2. **Volume:** within **0.5 %** of the STL's.
3. **Surface deviation**, both ways: the solid is tessellated at 0.005 mm chordal tolerance. On each side, the points are every vertex of that side's mesh after longest-edge bisection until no edge exceeds 0.5 mm, plus 100 000 points sampled by area (seed 20261110 on the part, 20261111 on the STL), each measured to the other mesh's closest point. Over both directions, the maximum (vertices and samples) must be at most **0.20 mm** and the 95th percentile (area samples) at most **0.05 mm**. The report gives each direction: STL to part shows a missing feature, part to STL an extra one.
4. **Bounding box:** each of its six faces within **0.15 mm** of the STL's.

Then, in the default mode, each staged STEP file must re-read as one solid with volume within 0.1 % and each bounding-box face within 0.05 mm of the built part's.

**Justification.** The STL's vertices lie on the true surface, and its facets cut inside convex arcs and outside concave ones by their chord error (sagitta) r(1 − cos(π/n)), for radius r and n segments to a full turn. A correct parametric solid therefore deviates from the STL by up to that chord error on curved faces, and by nothing on flat ones. `build.py --chord-error` measures each STL's chord error from circles fitted to the vertex runs of its curved features, in sections every 0.5 mm along each axis:

| Part | Chord error (mm) | Feature (r, n) |
|---|---|---|
| Frame Top | 0.050 | r 25.00, n 49 (the outline's side arcs) |
| Frame Bottom | 0.053 | r 38.82, n 60 |
| Servo Mount | 0.003 | r 2.20, n 56 |
| Bearing Axle Mount | 0.003 | r 1.60, n 54 (the boss foot's 1 mm round, which no section sees, fits to 0.003 mm too) |
| Hubcap | 0.002 | r 13.00, n 187 |

- **Maximum 0.20 mm:** above the worst chord error (0.053 mm measured; 0.13 mm in the first, cruder estimate), with room for measurement rounding, since dimensions are measured to 0.01 mm.
- **95th percentile 0.05 mm:** flat faces cover most of every frame part's area, and a correctly measured flat face deviates by nothing; a face 0.1 mm off over more than 5 % of the area fails.
- **Volume 0.5 %:** facets cost a bore only a fraction of its own cross-section, a small part of a part's volume; a wrong wall or a missing hole that matters exceeds it.
- **Bounding box 0.15 mm:** a true arc at the part's extreme can stand past the STL's vertices by its chord error.
- **STEP re-read and import volume 0.1 %:** the STEP holds the same exact geometry; the tolerance covers the kernels' volume integration.

**Sampling bound.** Every point of a facet whose edges are at most 0.5 mm lies within 0.5/√3 ≈ 0.29 mm of one of its vertices, so the true maximum deviation is at most the reported maximum plus 0.29 mm. A feature on one side only that stands more than 0.49 mm off the other surface is always caught, however small its area. One standing 0.20 to 0.49 mm off is caught when a vertex or sample lands where it stands more than 0.20 mm off, and otherwise only by the volume gate: a feature under 0.49 mm deep and smaller than the vertex spacing is a stated blind spot, not a guarantee.

The parts are compared in the STL's coordinates with no alignment, so a misplaced part fails. That protects the movement constants measured in those coordinates (`core/movement/body_geometry.h`).

## Per-part exceptions

A part whose STL's own chord error exceeds 0.15 mm, or that has an organic or freeform feature that feature-based modelling cannot meet, declares an exception for that gate only:

```python
"exceptions": {
    "deviation_max": {"limit": 0.25, "feature": "the toe's spline profile",
                      "reason": "freeform; 0.05 mm over the global limit"},
},
```

The gate is one of `volume` (limit in %), `deviation_max`, `deviation_p95` or `bbox` (mm); the solid gate has none. For a tessellation exception the limit is the STL's chord error plus 0.05 mm. The build prints every exception it applies, every other part keeps the global limits, and the global limits are never loosened. The source README names each exception's feature and the amount by which it exceeds the global limit.

No part declares an exception yet.

## Import checks

The default mode then imports each committed STEP file in two tools:

- **FreeCAD**, headless: `freecad_import.py` under `FreeCADCmd`, from `PATH` or the flatpak (`flatpak run --command=FreeCADCmd org.freecad.FreeCAD`), reads each file with `Part.read` and reports its solids, `isClosed()`, `isValid()`, the volume and `FreeCAD.Version()`. The files go in `SPOODER_STEP_FILES` by absolute path, which the flatpak reads through its host file-system access.
- **solidcraft:** `solidcraft-cli eval <file>`, whose JSON gives the bodies with `closed` and `volume_mm3`, and `solidcraft-cli --version`.

Each part must import as one closed solid (in FreeCAD, valid too), with volume within 0.1 % of the built part's. Each check prints one line per part and tool: `PASS` with the tool's version, `FAIL` with the measured value, or `SKIPPED` with the reason when the tool is not installed. The summary counts passes, failures and skips per tool. A skip is never a pass and never fails the build; a failure fails it.

Versions checked: **FreeCAD 1.1.4** (flathub) and **solidcraft-cli 0.30.1**, on 2026-11-10, every frame part passing in both. Neither tool is pinned, so a newer version may read STEP differently; the check prints the version it ran.

## Running the build

From this folder:

```sh
uv run --locked --quiet --script build.py --test         # the unit tests
uv run --locked --quiet --script build.py --check        # build every part and run the gates; nothing written
uv run --locked --quiet --script build.py                # the gates, then STEP export, re-read, publish and the import checks
uv run --locked --quiet --script build.py --chord-error  # each STL's chord error; nothing written
```

From the repository root, `mise run cad-reference` runs the tests and then the default build. It is a verification gate when this folder changes. The frame group's `--check` takes about ten minutes, most of it in the surface deviation's closest-point queries. A run that leaves `git status` clean has reproduced the committed STEP files byte for byte (their header time is fixed).

Dependencies are pinned inline in `build.py` (Python 3.12, build123d 0.13.0, numpy 2.1.3, trimesh 4.5.3, rtree 1.4.1 for trimesh's closest-point query, `exclude-newer`) and locked in `build.py.lock` (`uv lock --script build.py` after a reviewed change of pins).

`step/` is replaced as a whole: the files are staged beside it, re-read, then `step/` is renamed to `step.old/`, the staging folder to `step/`, and `step.old/` removed. A run killed between the renames leaves the old files complete in `step.old/`, and the next build, in any mode, restores them before anything else.

CI does not run `mise run cad-reference`, as it does not run `mise run cad`: neither import tool is installed there.
