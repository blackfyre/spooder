"""Writing the print and CAD files, and checking what was written.

Each distinct part is written once, in the frame top's coordinates:
- print/<name>.stl, or print/<name>.3mf for a part with inlays (the cover),
  holding the body and the inlays as two meshes, the parts of one object
  (one build item), which the slicer assigns to different filaments;
- step/<name>.step, the body and any inlays together.

Files in print/ and step/ that the build did not write (a renamed or removed
part) are deleted, so the folders hold exactly the current part set.
`verify_export()` re-reads what was written: one print file and one STEP
file per distinct part and nothing else, each STEP's and print file's
bounding box equal to the built part's, each 3MF one object of two parts,
and the deck's insert holes found again in its STEP.
"""

from __future__ import annotations

import copy
import re
import uuid
import zipfile
from pathlib import Path

from build123d import Compound, Mesher, export_step, export_stl, import_step

from checks import insert_hole_failures
from model import Part

PRINT_DIR = "print"
STEP_DIR = "step"
PRINT_SUFFIXES = (".stl", ".3mf")
STEP_SUFFIX = ".step"
# STL and 3MF tessellation. Source: chosen, below a 0.4 mm nozzle's resolution.
MESH_TOLERANCE = 0.01  # mm, chordal
MESH_ANGULAR = 0.1  # rad
# A fixed STEP header time, so an unchanged part writes an unchanged file.
# Source: chosen.
STEP_TIMESTAMP = "2026-01-01T00:00:00"
# Bounding boxes of re-read files agree with the built part within this.
# Source: chosen; a mesh's vertices lie on the surface, so its box can only
# shrink by the chordal tolerance.
BBOX_TOL = 0.05


def print_name(part: Part) -> str:
    return part.name + (".3mf" if part.inlays is not None else ".stl")


def expected_files(parts: list[Part]) -> set[str]:
    return {f"{PRINT_DIR}/{print_name(p)}" for p in parts} | {f"{STEP_DIR}/{p.name}{STEP_SUFFIX}" for p in parts}


def existing_files(outdir: Path) -> set[str]:
    found = set()
    for sub, suffixes in ((PRINT_DIR, PRINT_SUFFIXES), (STEP_DIR, (STEP_SUFFIX,))):
        d = outdir / sub
        if d.is_dir():
            found |= {f"{sub}/{f.name}" for f in d.iterdir() if f.suffix.lower() in suffixes}
    return found


def cad_shape(part: Part):
    return part.shape if part.inlays is None else Compound([part.shape, part.inlays])


def export_parts(parts: list[Part], outdir: Path) -> list[Path]:
    """Write every distinct part's files and delete stale ones."""
    keep = expected_files(parts)
    for stale in sorted(existing_files(outdir) - keep):
        (outdir / stale).unlink()
    for sub in (PRINT_DIR, STEP_DIR):
        d = outdir / sub
        if d.is_dir() and not any(d.iterdir()):
            d.rmdir()
    written = []
    for part in parts:
        (outdir / PRINT_DIR).mkdir(parents=True, exist_ok=True)
        (outdir / STEP_DIR).mkdir(parents=True, exist_ok=True)
        target = outdir / PRINT_DIR / print_name(part)
        if part.inlays is None:
            export_stl(part.shape, target, tolerance=MESH_TOLERANCE, angular_tolerance=MESH_ANGULAR)
        else:
            write_3mf(part, target)
            normalise_3mf(target, part.name)
        written.append(target)
        step = outdir / STEP_DIR / f"{part.name}{STEP_SUFFIX}"
        export_step(cad_shape(part), step, timestamp=STEP_TIMESTAMP)
        written.append(step)
    return written


def write_3mf(part: Part, target: Path) -> None:
    """The body and the inlays, each one mesh (the inlays' pieces together),
    as the two parts of one object, the file's one build item."""
    mesher = Mesher()
    model, identity = mesher.model, mesher.wrapper.GetIdentityTransform()
    obj = model.AddComponentsObject()
    obj.SetName(part.name)
    for label, shape in (("body", part.shape), ("inlays", part.inlays)):
        verts, tris = Mesher._mesh_shape(copy.deepcopy(shape), MESH_TOLERANCE, MESH_ANGULAR)
        mesh = model.AddMeshObject()
        mesh.SetGeometry(*Mesher._create_3mf_mesh(verts, tris))
        mesh.SetName(f"{part.name} {label}")
        if not mesh.IsValid():
            raise RuntimeError(f"{part.name} {label}: invalid 3MF mesh")
        obj.AddComponent(mesh, identity)
    model.AddBuildItem(obj, identity)
    mesher.write(str(target))


def model_xml(path: Path) -> str:
    with zipfile.ZipFile(path) as z:
        name = next(n for n in z.namelist() if n.endswith(".model"))
        return z.read(name).decode()


def normalise_3mf(path: Path, name: str) -> None:
    """Make a 3MF byte-for-byte repeatable: lib3mf gives its components and
    build items random UUIDs and the archive the time of writing. Each UUID
    becomes one derived from the part's name and its place in the file, and
    every entry gets a fixed date."""
    with zipfile.ZipFile(path) as z:
        entries = [(i.filename, z.read(i.filename)) for i in z.infolist()]
    count = iter(range(1_000_000))

    def fixed(_: re.Match) -> bytes:
        return f'p:UUID="{uuid.uuid5(uuid.NAMESPACE_URL, f"spooder/{name}/{next(count)}")}"'.encode()

    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for filename, data in entries:
            if filename.endswith(".model"):
                data = re.sub(rb'p:UUID="[0-9a-fA-F-]{36}"', fixed, data)
            info = zipfile.ZipInfo(filename, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)


def bbox_tuple(bb) -> tuple[float, ...]:
    return (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)


def bbox_differs(a, b, tol: float) -> bool:
    return any(abs(x - y) > tol for x, y in zip(bbox_tuple(a), bbox_tuple(b)))


def verify_export(parts: list[Part], outdir: Path) -> list[str]:
    """Re-read the written files and compare them with the built parts."""
    import trimesh

    failures = []
    expected, found = expected_files(parts), existing_files(outdir)
    for missing in sorted(expected - found):
        failures.append(f"export: {missing} was not written")
    for extra in sorted(found - expected):
        failures.append(f"export: {extra} is not a current part")
    for part in parts:
        built = cad_shape(part).bounding_box()
        step = outdir / STEP_DIR / f"{part.name}{STEP_SUFFIX}"
        if step.exists():
            shape = import_step(step)
            if bbox_differs(shape.bounding_box(), built, 1e-3):
                failures.append(f"export: {step.name}'s bounding box {bbox_tuple(shape.bounding_box())} "
                                f"is not the built part's {bbox_tuple(built)}")
            if part.role == "deck":
                failures += [f"export: {step.name}: {f}" for f in insert_hole_failures(part.name, shape)]
        target = outdir / PRINT_DIR / print_name(part)
        if not target.exists():
            continue
        if part.inlays is None:
            mesh = trimesh.load(target)
            lo, hi = mesh.bounds
            pb = bbox_tuple(built)
            if any(abs(a - b) > BBOX_TOL for a, b in zip((*lo, *hi), pb)):
                failures.append(f"export: {target.name}'s bounding box is not the built part's")
        else:
            meshes = Mesher().read(str(target))
            if len(meshes) != 2:
                failures.append(f"export: {target.name} holds {len(meshes)} meshes, not the body and the inlays")
            elif bbox_differs(Compound(meshes).bounding_box(), built, BBOX_TOL):
                failures.append(f"export: {target.name}'s bounding box is not the built part's")
            xml = model_xml(target)
            items, components = len(re.findall(r"<item\b", xml)), len(re.findall(r"<component\b", xml))
            if (items, components) != (1, 2):
                failures.append(f"export: {target.name} has {items} build item(s) and {components} component(s), "
                                "not one object of two parts")
    return failures
