# Design

## Context

The parametric reference parts were founded by `parametric-reference-parts` (archived 2026-11-11), which set the part contract, the gates, the STEP export and the import checks, and modelled the frame group. Its README (`hardware/parametric-reference-parts/README.md`) is the contract this change follows: "Part contract", "Adding a part", "Gates and tolerances" and "Per-part exceptions".

Measured on the feet STLs (trimesh 4.5.3), all watertight, one body each:

| STL | Triangles | Volume (mm³) | Bounding box (mm) |
|---|---|---|---|
| Foot Left | 14 038 | 3 313.33 | x 104.765 to 121.479, y 55.323 to 103.811, z 0 to 12.45 |
| Foot Right | 14 038 | 3 313.33 | x 127.411 to 144.126, y 54.253 to 102.742, z 0 to 12.45 |
| Foot Shaft | 1 544 | 2 055.77 | x 943.066 to 1018.402, y 322.978 to 334.419, z 9.788 to 19.576 |
| Foot Tip | 4 054 | 548.48 | x 558.866 to 569.261, y 122.80 to 133.20, z −52.187 to −42.685 |

First sections of Foot Left: from z 0 to 5 a plate whose sections show 22 loops (the outline and the flexure's cuts); from z 5 to about 6.5 a closed layer with rounded edges (many vertex levels between z 5.5 and 5.76); above it a small post, about 4.4 mm square at z 8, narrowing to about 3 mm at z 12.3. Foot Shaft lies along x, with a collar near x 965. Foot Tip narrows towards its top, a round or domed end, and holds a second loop (a bore) in its lower half.

## Goals / Non-Goals

**Goals:**
- Foot Left, Foot Right, Foot Shaft and Foot Tip as parametric parts that pass the gates (the global limits, or a per-part exception documented as below) and import as one closed solid in FreeCAD and solidcraft.
- The flexure's path and beam widths as named dimensions, shared by both feet, so changing one and running `--check` rebuilds both feet with it.

**Non-Goals:**
- Foot Left Right (a later arrangement change), the ball foot, the other groups.
- Any edit to the shared tooling, its tests (`test_build.py`) or another group's files, and any edit to the folder README beyond its exception sentence (Gates and exceptions).
- Changing the spring: upstream's flexure is reproduced as it is.

## Decisions

### Files

```text
aecert-hexapod-v1/parts/
  _feet.py        the foot half: its named dimensions (outline, flexure, cap, post) and foot_half()
  foot_left.py    Foot Left: build() returns _feet.foot_half()
  foot_right.py   Foot Right: Foot Left's solid, mirrored and moved (below)
  foot_shaft.py   Foot Shaft
  foot_tip.py     Foot Tip
test_feet.py      the feet group's tests (Tests, below)
```

The foot half's dimensions live in `_feet.py`, not in `foot_left.py`, because both feet are built from them, as Frame Top and Frame Bottom share `_frame.py`. Each constant carries its source comment (the trimesh section, probe or circle fit, and where), measured on Foot Left's STL.

### The flexure

The serpentine is modelled as a path, not as a set of separate slots: named constants give its start, the number of turns, the pitch between runs, the run length, the turn radius and the beam width (and, where the STL's beams differ, the width of each kind of beam, such as the end beams joining the frame and the tip). The cuts between the beams follow from the path and the widths, so retuning the spring is a change of constants. Where the STL's turns are not exact arcs, the constant is the least-squares circle through their vertices and the comment says so, as for the frame.

### Foot Right

The two foot STLs are mirror images: Foot Right reflected across x and moved onto Foot Left lies within 0.0012 mm of it (20 000 area samples). Their bounding boxes give a mirror plane x = 124.445, but Foot Right's y range sits 1.069 mm lower than Foot Left's. A mirrored-pair declaration takes only a `Plane`, and a reflection across any plane cannot add that offset; a tilted plane would turn the part instead. The offset is far above the 0.15 mm bounding-box gate, so the mirrored-pair mechanism does not meet the gates.

Foot Right is therefore its own part file whose `build` imports `foot_left` (a sibling, allowed by the contract, as an arrangement plate imports the single parts), mirrors its solid across `Plane.YZ.offset(MIRROR_X)` and moves it by `RIGHT_OFFSET_Y`, both measured from the two STLs' bounding boxes and comments saying so. Foot Right has no model of its own, so a retuned flexure changes both feet, which is what the mirrored pair would have given. The main spec's mirrored-pair rule covers a part file declaring two sides; Foot Right declares one part, so discovery accepts it, and the feet requirement states this exception.

### Foot Shaft and Foot Tip

Each is modelled from its own sections in its STL's coordinates. Both look like turned parts (a shaft with a collar; a tip with a bore and a domed end), so the likely features are revolved profiles and extrusions; a feature that turns out to be freeform is handled as below.

### Gates and exceptions

Every part is held to the global gates. `build.py --chord-error` measures each STL's chord error once the parts are declared. A part whose chord error exceeds 0.15 mm gets the documented per-part exception for the tessellation gate, at its chord error plus 0.05 mm. A feature that feature-based modelling cannot match (a freeform surface) is recorded as a per-part exception that names the feature and its size, and the amount by which it exceeds the global limit; a global limit is never loosened. Both are listed in the feet group's table. The tessellation limit is rounded up to 0.01 mm. A freeform exception is marked by a reason starting with "freeform" (as the folder README's example), so the tests can tell the two kinds apart; it applies to the failing gate only, at the measured amount, and is not derived from the chord error. If any foot declares an exception, the folder README's sentence "No part declares an exception yet." would become false, so this change rewrites that one sentence to name the foot exceptions and point to the feet group's table; the rest of the folder README, including its chord-error table, is left alone.

### Tests

The feet group's tests live in `test_feet.py`, a new file beside `test_build.py`, which `build.py --test` already discovers (`test_*.py`); `test_build.py` is not edited, so the parallel group changes do not conflict on it. They discover the real Aecert source, as `FrameGroup` does, and check:

- the four foot parts are declared, each in its file and against its STL, failing with the missing part's name;
- the retuning: one flexure beam width in `_feet` (the module object that Foot Left's build reads) raised by 0.3 mm and restored in a `finally`; both feet rebuilt differ in volume from their unchanged builds and each fails its volume or deviation gate, which also shows Foot Right is built from the shared model;
- for each foot part, `chord_error.measure` on its STL: above 0.15 mm the part declares the `deviation_max` exception with the limit at the chord error plus 0.05 mm, rounded up to 0.01 mm; at or below it every exception it declares has a reason starting with "freeform", a feature and a limit above the global limit, and `gates.check` on the part without that exception fails that exception's gate (so a freeform exception is never declared for a gate that passes); a foot with neither declares none;
- for each foot exception, the README row gives the amount over the global limit (the limit minus the global limit, in the gate's unit: % for `volume`, mm for the others, to two decimals, followed by "over"), which `test_source_readmes_list_their_parts` does not check.

That the README rows name each exception's feature and absolute limit is already checked by `test_source_readmes_list_their_parts` for every part.

## Risks / Trade-offs

- **The flexure's thin beams and rounds** may defeat OpenCascade's fillets, as Frame Bottom's chamfers did → cut them with explicit profiles in the flexure sketch, as `_frame.chamfered` cuts wedges.
- **Run time:** the surface deviation gate on Foot Left's 14 038 facets adds to the build's ten minutes; acceptable for a gate run locally.
- **Merge conflicts:** the coxa, femur, tibia and LilyGO changes edit `aecert-hexapod-v1/README.md` and `CHANGELOG.md` in parallel → this change touches only its own section, its own line in "Groups still to model", one changelog line, its own `test_feet.py` and, if a foot declares an exception, the folder README's exception sentence.

## Assumptions

- The group is delivered as one change, all four parts, although the parts are independent: the programme adds one group per change (the archived change's Follow-ups, the source README's "Groups still to model"), and splitting it would leave a group half modelled with a second change for two small parts.
- Foot Right is a separate part file built from Foot Left's solid (Foot Right, above), not a mirrored-pair declaration, since the pair mechanism cannot carry the 1.069 mm offset.
- The foot half's dimensions are measured on Foot Left's STL; Foot Right is checked through the mirror against its own STL.
- The feet group's tests go in a new `test_feet.py`, not in `test_build.py`: the brief keeps this change off the shared tooling and its tests, and the parallel group changes would otherwise conflict there. The retuning is checked in a unit test that restores the constant in-process, not by editing `_feet.py` and checking it out again.
- The folder README's chord-error table belongs to the shared README and is not edited. Its sentence "No part declares an exception yet." is the one exception: leaving it false would make the documentation lie, so it is rewritten if a foot declares an exception.
- The changelog line goes under Unreleased, Added.
- The feet group's section in `aecert-hexapod-v1/README.md` starts with task group 1, holding the rows of Foot Shaft and Foot Tip: `test_source_readmes_list_their_parts` fails for any declared part without a row, so each group's commit adds its parts' rows to keep `build.py --test` passing. Task 3.2 completes the section (the feet's rows, the flexure, Foot Right) and its other edits.
- Foot Shaft and Foot Tip need no exception: their sections are flats, 45° faces and circular rounds and arcs (Foot Shaft's 0.8 mm rounds, Foot Tip's 5.2 mm side and 4.5 mm top round), which feature-based modelling meets within the global limits.
- The flexure's named dimensions (task group 2) are its start (`FLEXURE_START_X`, `FLEXURE_START_Y`, the first beam's lower side), `FLEXURE_TURNS` (17, so 18 beams), `FLEXURE_PITCH`, `FLEXURE_RUN_LENGTH`, `FLEXURE_TURN_RADIUS` (the turns' outer corners) and two beam widths, `FLEXURE_BEAM_WIDTH` for the runs and `FLEXURE_TURN_WIDTH` for the turns, with `FLEXURE_BLOCK_GAP` for the first cut, which is wider than the others. The cuts' width (pitch less beam width) and their end rounds (half that width) are derived, not named, so a retuned beam width still builds a valid solid.
- Foot Left needs no exception: every feature in its sections is a flat, a 45° or 35° face, or a circular round or arc. Its boss's two sides, which look straight, are arcs of r 194.42 and r 296.47 (their vertices fit circles to 0.0002 mm), and are modelled as such. The STL's chamfers stop at rounds by carrying the chamfered sides' cut on past them (the rounds keep their unchamfered shape where the cut leaves them), so they are modelled as cuts of the sides' tapered outline inside a bounded box, not as OpenCascade chamfers.
- Foot Left and Foot Right get their rows in the feet group's table in task group 2, as Foot Shaft and Foot Tip did in task group 1, so that `build.py --test` keeps passing; task 3.2 completes the section.
- `MIRROR_X` and `RIGHT_OFFSET_Y` are given to 0.0001 mm, from the STLs' bounding boxes (both pairs of faces agree to that).

## Follow-ups

- Foot Left Right, the combined plate, as an arrangement of Foot Left and Foot Right (the plates change).
- If wanted, add the feet to the folder README's chord-error table.
