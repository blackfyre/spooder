"""The checks every generated part set must pass before anything is written.

Each check takes an `Assembly` (model.py) and returns its failures as
sentences; an empty list is a pass. `run_checks()` runs them all. A check
reads only what the assembly declares: an assembly with no cover has nothing
to enclose against, one with no air channel nothing to keep clear, and an
empty assembly passes every check (task 2.1). Each check is shown to fail on
a deliberately bad shape in test_build.py.

The rules' numbers are named below, each with its source.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np
from build123d import Compound, Face, GeomType, Location, Plane, Shape, Solid, Vector, Vertex
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from build123d.topology.kernel import list_shapes
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS
from OCP.collections import IndexedDataMap_TopoDS_Shape_List_TopoDS_Shape_TopTools_ShapeMapHasher as EdgeFaceMap

import measurements as M
from model import (AXES, FANS, IMU, J6, LM2596_OUTPUT_ROUTE, SCHOTTKY, SENSOR_HEADER, SERVO_2040, SZBK07, Assembly,
                   Box6, Terminal, box, box6, turned)

# Leg keep-clear radius about the body's centre. Source: design.md "Context":
# nothing on the legs comes nearer than r 81.8 above the frame top, less a
# 3 mm margin; the spec's "Fit to the frame and the legs".
KEEP_CLEAR_R = 78.8
# The front and rear tabs, also clear of the legs: |x| <= 24, |y| <= 79.
# Source: the spec's "Fit to the frame and the legs".
KEEP_CLEAR_TAB = (24.0, 79.0)
# The keep-clear region is enforced from the frame's floor up, not only above
# the ring's top as the spec states: below the ring a part outside it would
# meet the frame or the coxa servos. Source: design.md "Assumptions".
KEEP_CLEAR_FROM_Z = M.FRAME_FLOOR_Z
# The printer's bed: a Bambu P1P, 256 x 256 x 256 mm. Source: the spec's
# "Printability".
BED = 256.0
# Steepest downward face allowed without support, from the horizontal.
# Source: the spec's "Printability".
OVERHANG_DEG = 45.0
# A downward planar face at most this wide bridges. Source: the spec's
# "Printability"; the spooder-printed-parts skill's `BRIDGE`.
BRIDGE = 3.0
# A downward face hangs from material at a point on its edge when a
# neighbouring face passes within ANCHOR_REACH of the point ANCHOR_PROBE
# below it: a wall or a 45-degree face going down does (0 and 0.21), one
# rising from the edge does not (0.3). Source: chosen.
ANCHOR_PROBE = 0.3
ANCHOR_REACH = 0.25
# Free space the frame must leave under a tie hole or slot. Source: the spec's
# "Cable routes"; the skill's `TIE_GAP`.
TIE_GAP = 1.5
# M3 clearance hole. Source: the spooder-printed-parts skill, "Fits and
# fasteners".
M3_CLEARANCE_D = 3.4
# Material round an insert's hole for the screw head to bear on, out to this
# radius. Source: an ISO 7380 M3 button head, Ø5.7 (radius 2.85), rounded up.
SCREW_SEAT_R = 3.0
# Screwdriver path: a cylinder of this diameter along the screw's axis.
# Source: the spec's "Access and service".
DRIVER_D = 4.0
# How far a screwdriver path is followed out of the mount. Source: chosen,
# beyond the bed size, so it reaches past every part.
DRIVER_REACH = 300.0
# The air channel passes the SZBK07 when it comes this close to its outline.
# Source: chosen, a gap the air sweeps across.
AIR_PAST = 1.0
# Boolean noise: overlaps up to this volume (mm^3) are coplanar contact, not
# collision. Source: chosen, well below any real collision.
VOLUME_TOL = 0.01
# Grid step of the frame top's height map. Source: reference_check.py's GRID.
FRAME_GRID = 0.25
# A route reaches a terminal when it holds the space next to the terminal's
# held part on the terminal's side: from this far off the part's outline
# (clear of the fit gap round it), this deep. Source: chosen, beyond the
# spooder-printed-parts skill's FIT of 0.3.
APPROACH_OFFSET = 0.4
APPROACH = 1.0
# The IMU breakout lies within this of the body's centre, and this far from
# the SZBK07's outline. Source: the spec's "Sensor provisions".
IMU_MAX_R = 30.0
IMU_SZBK07_GAP = 10.0
# Clearance round each wire counted through an entry or a pass-through.
# Source: chosen.
WIRE_CLEARANCE = 0.2
# Surface samples are spaced about this far apart. Source: chosen, finer than
# the narrowest feature the checks look for (a 3 mm bridge).
SAMPLE_STEP = 1.5
# The lid must have no opening above these held parts. Source: the spec's
# "Cooling" (the SZBK07's heatsinks are live).
CLOSED_OVER = (SZBK07,)
# Rays up from the closed-over part's top are this far apart. Source: chosen,
# finer than a 2 mm slot.
CLOSED_STEP = 1.0
# The cover is lifted by each of these (mm) and must meet nothing on the way:
# it lifts straight off. Source: the spec's "Access and service"; the steps
# are chosen to reach past the cover's height.
LIFT_STEPS = (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0)
# A fan is supported at each end when a part (not the cover) lies within this
# below its underside, under the last FAN_END of its width; it is located at
# each end when a part lies within FAN_LOCATE beyond its end face. Source:
# chosen, the skill's FIT with margin.
FAN_SEAT_GAP = 0.4
FAN_END = 3.0
FAN_LOCATE = 0.8
# Moved this far across its width or inwards, a fan meets a part other than
# the cover (its holder); moved outwards, it meets a part or the cover's wall.
# Source: chosen, beyond the skill's FIT.
FAN_PLAY = 1.0
# The cover may come no nearer a fan than this: it carries no fan. Source:
# chosen, well below the skill's FIT, which the layout keeps (a box grown by
# it reaches FAN_COVER_GAP x sqrt 2 into a chamfer at its edges).
FAN_COVER_GAP = 0.1
# At least this share of rays from a fan's outer face, out along its axis,
# must pass the cover through its vent. Source: chosen, a grille of 2 mm
# slots at a 4 mm pitch over most of the fan's face passes about 0.4.
FAN_VENT_OPEN = 0.25

# Slices through a junction between boxes in differently turned frames are
# this thick. Source: chosen.
SLICE_T = 0.2
# A vent's open area, measured through the cover, may differ from its
# declared one by this share (boolean noise). Source: chosen.
VENT_AREA_TOL = 0.01

# The bumpers stand at least this far proud of the cover, the held parts and
# the screw heads, in every horizontal direction (every BUMPER_STEP degrees)
# and upwards: a flat obstacle or floor meets them first. Source: the spec's
# "Protection"; chosen.
BUMPER_PROUD = 1.0
BUMPER_STEP = 5.0
# Lowered this far, a bumper meets the cover (its stop on the rails); moved
# BUMPER_PLAY sideways, it meets nothing (it slides on them). Source: chosen,
# below the layout's sliding fit (0.2).
BUMPER_STOP = 0.5
BUMPER_PLAY = 0.1
# A bumper pulled off the cover (along its `outward`) meets the cover before
# it has gone this far: its rails keep it on. Source: chosen, beyond the
# layout's rails (3 mm deep); the pull is sampled every BUMPER_PULL_STEP.
BUMPER_PULL = 5.0
BUMPER_PULL_STEP = 0.25
# A screw's head lies on a part when its centre is this near it, or (for a
# screw with a shank, whose head centre lies over its hole) within its
# shank's diameter. Source: chosen.
SCREW_ON = 0.1
# A supporting part bears on a held part's underside where it comes within
# CONTACT_GAP of it, over at least CONTACT_AREA in each bearing region; the
# held part moved LOCATE sideways meets it. Source: chosen.
CONTACT_GAP = 0.1
CONTACT_AREA = 2.0
LOCATE = 1.0

COS_OVERHANG = math.cos(math.radians(OVERHANG_DEG))
RAY_DIRECTIONS = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1))


# --------------------------------------------------------------------------- helpers


def volume(shape: Shape | None) -> float:
    if shape is None:
        return 0.0
    return sum(s.volume for s in shape.solids())


def common(a: Shape, b: Shape) -> float:
    """The volume a and b share, after a bounding-box test."""
    ba, bb = a.bounding_box(), b.bounding_box()
    if (ba.max.X <= bb.min.X or bb.max.X <= ba.min.X or ba.max.Y <= bb.min.Y or bb.max.Y <= ba.min.Y
            or ba.max.Z <= bb.min.Z or bb.max.Z <= ba.min.Z):
        return 0.0
    return volume(a & b)


def outside(a: Shape, b: Shape) -> float:
    """The volume of a that lies outside b."""
    return volume(a - b)


def cylinder(r: float, origin, axis, length: float) -> Solid:
    return Solid.make_cylinder(r, length, Plane(origin=origin, z_dir=axis))


def vertices(shape: Shape, tolerance: float = 0.05) -> np.ndarray:
    verts, _ = shape.tessellate(tolerance)
    return np.array([(v.X, v.Y, v.Z) for v in verts])


def triangles(shapes: Iterable[Shape], tolerance: float = 0.05) -> np.ndarray:
    tris = []
    for shape in shapes:
        verts, faces = shape.tessellate(tolerance)
        pts = np.array([(v.X, v.Y, v.Z) for v in verts])
        if len(faces):
            tris.append(pts[np.array(faces)])
    return np.concatenate(tris) if tris else np.zeros((0, 3, 3))


def face_samples(face, step: float = SAMPLE_STEP, minimum: int = 9):
    """Points on the face about `step` apart (at least minimum x minimum over
    its parameter range) and the face's outward normal at each. Sampled by
    position_at() and kept where is_inside(), because tessellating a single
    face can return only part of it (the skill's build123d pitfalls). The
    points are cell centres, an odd number across so the middle is one, and
    never on the face's edges, where a ray would graze its neighbours."""
    bb = face.bounding_box()
    size = max(bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z)
    n = max(minimum, min(41, int(size / step) + 1))
    if n % 2 == 0:
        n += 1
    cells = (np.arange(n) + 0.5) / n
    found = []
    for u in cells:
        for v in cells:
            p = face.position_at(u, v)
            if face.is_inside(p, 1e-4):
                found.append((p, face.normal_at(p)))
    return found


def ray_hits(origins: np.ndarray, direction, tris: np.ndarray) -> np.ndarray:
    """For each origin, whether a ray along `direction` meets any triangle
    (Möller–Trumbore, ahead of the origin)."""
    hit = np.zeros(len(origins), dtype=bool)
    if len(tris) == 0 or len(origins) == 0:
        return hit
    d = np.asarray(direction, dtype=float)
    v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    p = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, p)
    ok = np.abs(det) > 1e-12
    v0, e1, e2, p, det = v0[ok], e1[ok], e2[ok], p[ok], det[ok]
    inv = 1.0 / det
    for i in range(0, len(origins), 64):
        o = origins[i:i + 64, None, :]
        s = o - v0[None]
        u = np.einsum("nij,ij->ni", s, p) * inv
        q = np.cross(s, e1[None])
        v = np.einsum("j,nij->ni", d, q) * inv
        t = np.einsum("ij,nij->ni", e2, q) * inv
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6)
        hit[i:i + 64] = inside.any(axis=1)
    return hit


def ray_distances(origins: np.ndarray, direction, tris: np.ndarray) -> np.ndarray:
    """For each origin, how far along `direction` a ray goes before it meets
    a triangle (inf if it meets none; 0 if it starts on one, a contact)."""
    out = np.full(len(origins), np.inf)
    if len(tris) == 0 or len(origins) == 0:
        return out
    d = np.asarray(direction, dtype=float)
    v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    p = np.cross(d, e2)
    det = np.einsum("ij,ij->i", e1, p)
    ok = np.abs(det) > 1e-12
    v0, e1, e2, p, det = v0[ok], e1[ok], e2[ok], p[ok], det[ok]
    inv = 1.0 / det
    for i in range(0, len(origins), 64):
        o = origins[i:i + 64, None, :]
        s = o - v0[None]
        u = np.einsum("nij,ij->ni", s, p) * inv
        q = np.cross(s, e1[None])
        v = np.einsum("j,nij->ni", d, q) * inv
        t = np.einsum("ij,nij->ni", e2, q) * inv
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > -1e-6)
        out[i:i + 64] = np.where(inside, np.maximum(t, 0.0), np.inf).min(axis=1)
    return out


def labelled_solids(asm: Assembly):
    """(label, shape) for every printed copy, its inlays and every held part."""
    for part in asm.parts:
        for i, shape in enumerate(part.instances()):
            yield (part.name if i == 0 else f"{part.name} copy {i + 1}"), shape
        if part.inlays is not None:
            yield f"{part.name} inlays", part.inlays
    for held in asm.held:
        yield held.name, held.shape


def base_name(label: str) -> str:
    """The part's name in a label from labelled_solids()."""
    return label.split(" copy ")[0].removesuffix(" inlays")


def fmt(p) -> str:
    return "(" + ", ".join(f"{c:.1f}" for c in p) + ")"


# --------------------------------------------------------------------------- checks


def check_single_solid(asm: Assembly) -> list[str]:
    """Each part is one solid, with volume."""
    failures = []
    for part in asm.parts:
        n = len(part.shape.solids())
        if n != 1:
            failures.append(f"{part.name}: {n} solids, not one")
        elif volume(part.shape) <= VOLUME_TOL:
            failures.append(f"{part.name}: no volume")
    names = [p.name for p in asm.parts]
    for name in sorted({n for n in names if names.count(n) > 1}):
        failures.append(f"{name}: two parts share the name")
    return failures


def keep_clear_violation() -> Solid:
    """Everything from the frame floor up that lies outside the keep-clear
    region (the radius, or the front and rear tabs)."""
    tx, ty = KEEP_CLEAR_TAB
    allowed = cylinder(KEEP_CLEAR_R, (0, 0, KEEP_CLEAR_FROM_Z - 1), (0, 0, 1), 1000) + box(
        -tx, tx, -ty, ty, KEEP_CLEAR_FROM_Z - 1, 1000)
    return box(-600, 600, -600, 600, KEEP_CLEAR_FROM_Z, 900) - allowed


def hull_2d(points: np.ndarray) -> np.ndarray:
    """The convex hull of 2D points, counter-clockwise (monotone chain)."""
    pts = np.unique(np.round(points, 6), axis=0)
    if len(pts) < 3:
        return pts

    def half(seq):
        out: list = []
        for p in seq:
            while len(out) >= 2 and np.cross(out[-1] - out[-2], p - out[-2]) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def bed_fit(points: np.ndarray, up) -> tuple[float, float, float] | None:
    """The part's size as printed, (width, depth, height), turned about the
    vertical to a footprint that fits the bed, or None if no turn fits. The
    turns tried are those that lay each edge of the footprint's convex hull
    along the bed's x axis (where each width is least), and every 0.05°
    between, so a narrow range of fitting turns is not stepped over."""
    u = np.asarray(up, dtype=float) / np.linalg.norm(up)
    e1 = np.cross(u, (1, 0, 0) if abs(u[0]) < 0.9 else (0, 1, 0))
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    height = np.ptp(points @ u)
    if height > BED + 1e-6:
        return None
    hull = hull_2d(np.stack([points @ e1, points @ e2], axis=1))
    edges = np.diff(np.vstack([hull, hull[:1]]), axis=0)
    angles = np.concatenate([np.arctan2(edges[:, 1], edges[:, 0]) % (math.pi / 2),
                             np.radians(np.arange(0, 90, 0.05))])
    for a in angles:
        rot = hull @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        w, d = np.ptp(rot[:, 0]), np.ptp(rot[:, 1])
        if w <= BED + 1e-6 and d <= BED + 1e-6:
            return w, d, height
    return None


def check_fit(asm: Assembly) -> list[str]:
    """The keep-clear region, nothing below the frame floor, the bed."""
    failures = []
    if not asm.parts and not asm.held:
        return failures
    forbidden = keep_clear_violation()
    for label, shape in labelled_solids(asm):
        out = common(shape, forbidden)
        if out > VOLUME_TOL:
            failures.append(f"{label}: {out:.2f} mm³ outside the legs' keep-clear region")
        low = shape.bounding_box().min.Z
        if low < M.FRAME_FLOOR_Z - 1e-6:
            failures.append(f"{label}: reaches z {low:.2f}, below the frame floor at z {M.FRAME_FLOOR_Z}")
    for part in asm.parts:
        pts = vertices(part.shape)
        if part.inlays is not None:
            pts = np.concatenate([pts, vertices(part.inlays)])
        if bed_fit(pts, part.print_up) is None:
            failures.append(f"{part.name}: does not fit the {BED:.0f} mm bed as printed")
    return failures


def check_insert_holes(asm: Assembly) -> list[str]:
    """A clear M3 hole through each deck on each of the eight insert
    positions, with material round it for the screw head."""
    failures = []
    decks = [p for p in asm.parts if p.role == "deck"]
    if asm.parts and not decks:
        failures.append("no deck: nothing fastens to the frame's inserts")
    for deck in decks:
        failures += insert_hole_failures(deck.name, deck.shape)
    return failures


def insert_hole_failures(name: str, shape: Shape) -> list[str]:
    failures = []
    bb = shape.bounding_box()
    z0, h = bb.min.Z - 1, bb.max.Z - bb.min.Z + 2
    r = M3_CLEARANCE_D / 2
    for x, y in M.FRAME_INSERTS:
        clear = common(shape, cylinder(r - 0.05, (x, y, z0), (0, 0, 1), h))
        if clear > VOLUME_TOL:
            failures.append(f"{name}: no clear M3 hole at insert ({x}, {y}) ({clear:.2f} mm³ in the way)")
            continue
        seat = cylinder(SCREW_SEAT_R, (x, y, z0), (0, 0, 1), h) - cylinder(r + 0.05, (x, y, z0 - 1), (0, 0, 1), h + 2)
        if common(shape, seat) <= 1.0:
            failures.append(f"{name}: no material round the hole at insert ({x}, {y})")
    return failures


def check_overlaps(asm: Assembly) -> list[str]:
    """No two printed copies, inlays or held parts share volume."""
    items = list(labelled_solids(asm))
    failures = []
    for i, (la, a) in enumerate(items):
        for lb, b in items[i + 1:]:
            v = common(a, b)
            if v > VOLUME_TOL:
                failures.append(f"{la} overlaps {lb} by {v:.2f} mm³")
    return failures


def check_enclosure(asm: Assembly) -> list[str]:
    """Every held part is enclosed by the mount: rays from its surface along
    ±x, ±y and +z all meet the cover, a vent's grille, another printed part or
    another held part (not a fan, which is optional), except from its exposed
    regions in their direction, which (when they must show) meet nothing of
    the cover."""
    covers = [p for p in asm.parts if p.role == "cover"]
    if not covers or not asm.held:
        return []
    tris = triangles(s for c in covers for s in c.solids())
    fixed = [tris, triangles(s for p in asm.parts if p.role != "cover" for s in p.solids()),
             triangles(v.shape for v in asm.vents)]
    others = {h.name: triangles([h.shape]) for h in asm.held if h.name not in FANS}
    failures = []
    for held in asm.held:
        blockers = np.concatenate(fixed + [t for n, t in others.items() if n != held.name])
        open_dirs: dict[tuple, list] = {}
        shown: dict[int, int] = {}
        for face in held.shape.faces():
            for p, n in face_samples(face, step=3.0, minimum=3):
                for d in RAY_DIRECTIONS:
                    if n.dot(Vector(d)) <= 0.5:
                        continue
                    exp = next((k for k, e in enumerate(held.exposed)
                                if tuple(e.direction) == d and in_box(p, e.region)), None)
                    origin = np.array([[p.X, p.Y, p.Z]])
                    if exp is not None:
                        if held.exposed[exp].must_show:
                            shown[exp] = shown.get(exp, 0) + 1
                            if ray_hits(origin, d, tris)[0]:
                                open_dirs.setdefault(("covered", d), []).append(p)
                    elif not ray_hits(origin, d, blockers)[0]:
                        open_dirs.setdefault(("open", d), []).append(p)
        for (kind, d), pts in open_dirs.items():
            where = "".join(f"{'+-'[c < 0]}{'xyz'[i]}" for i, c in enumerate(d) if c)
            if kind == "open":
                failures.append(f"{held.name}: not enclosed towards {where} ({len(pts)} points, first at {fmt(pts[0])})")
            else:
                failures.append(f"{held.name}: covered towards {where} where it must show ({len(pts)} points, first at {fmt(pts[0])})")
        for k, e in enumerate(held.exposed):
            if e.must_show and not shown.get(k):
                failures.append(f"{held.name}: nothing of it faces {e.direction} in its exposed region {e.region}")
    return failures


def in_box(p, b: Box6, tol: float = 1e-3) -> bool:
    return (b[0] - tol <= p.X <= b[1] + tol and b[2] - tol <= p.Y <= b[3] + tol and b[4] - tol <= p.Z <= b[5] + tol)


def check_routes(asm: Assembly) -> list[str]:
    """No printed or held part enters a reserved route, except the Schottky
    diode in the LM2596's output route, which lies wholly inside it and
    overlaps no other route. No other held part may claim a route."""
    failures = []
    routes = {r.name: r for r in asm.routes}
    inline = {h.name: h for h in asm.held if h.route is not None}
    for name, held in inline.items():
        if (name, held.route) != (SCHOTTKY, LM2596_OUTPUT_ROUTE):
            failures.append(f"{name}: only the {SCHOTTKY} may lie in a route, and only '{LM2596_OUTPUT_ROUTE}' "
                            f"(not '{held.route}')")
    covers = {p.name for p in asm.parts if p.role == "cover"}
    for route in asm.routes:
        for label, shape in labelled_solids(asm):
            if route.cover_off and base_name(label) in covers:
                continue
            held = inline.get(label)
            if held is not None and label == SCHOTTKY and held.route == route.name == LM2596_OUTPUT_ROUTE:
                continue
            v = common(shape, route.shape)
            if v > VOLUME_TOL:
                failures.append(f"{label} enters the route '{route.name}' ({v:.2f} mm³)")
    diode = inline.get(SCHOTTKY)
    if diode is not None and diode.route == LM2596_OUTPUT_ROUTE:
        if LM2596_OUTPUT_ROUTE not in routes:
            failures.append(f"{SCHOTTKY}: its route '{LM2596_OUTPUT_ROUTE}' is not reserved")
        else:
            out = outside(diode.shape, routes[LM2596_OUTPUT_ROUTE].shape)
            if out > VOLUME_TOL:
                failures.append(f"{SCHOTTKY}: {out:.2f} mm³ outside its route '{LM2596_OUTPUT_ROUTE}'")
    route = routes.get(LM2596_OUTPUT_ROUTE)
    if route is not None:
        failures += sleeve_failures(route)
    return failures


def sleeve_failures(route) -> list[str]:
    """The diode's route carries its sleeve along its whole length: the route
    is made of boxes, each at least the sleeve across in every direction, and
    the boxes join, one to the next, through openings at least the sleeve
    across in two directions, into one run."""
    d = M.SCHOTTKY_SLEEVE_D - 1e-6
    if not route.boxes:
        return [f"route '{route.name}': not made of boxes, so its width cannot be checked"]
    failures = []
    for b in route.boxes:
        narrow = min(b[1] - b[0], b[3] - b[2], b[5] - b[4])
        if narrow < d:
            failures.append(f"route '{route.name}': box {fmt(b)} is {narrow:.1f} mm across, "
                            f"too narrow for the diode's {M.SCHOTTKY_SLEEVE_D} mm sleeve")
    n = len(route.boxes)
    joined = {0}
    frontier = [0]
    while frontier:
        i = frontier.pop()
        for j in range(n):
            if j in joined:
                continue
            a, b = route.boxes[i], route.boxes[j]
            sizes = [min(a[2 * k + 1], b[2 * k + 1]) - max(a[2 * k], b[2 * k]) for k in range(3)]
            if min(sizes) >= -1e-9 and sorted(sizes)[1] >= d:
                joined.add(j)
                frontier.append(j)
    if len(joined) < n:
        failures.append(f"route '{route.name}': its boxes do not join into one run at least the diode's "
                        f"{M.SCHOTTKY_SLEEVE_D} mm sleeve across")
    return failures


def approach(end: Terminal, held_box: Box6 | None) -> Box6:
    """The space a route must hold to reach `end`: next to the held part's
    outline on the terminal's side, over the terminal's cross-section; or,
    for a free-space terminal, its region."""
    if held_box is None:
        return end.region
    axis = next(i for i, c in enumerate(end.side) if c)
    sign = 1 if end.side[axis] > 0 else -1
    face = held_box[2 * axis + 1] if sign > 0 else held_box[2 * axis]
    lo, hi = sorted((face + sign * APPROACH_OFFSET, face + sign * (APPROACH_OFFSET + APPROACH)))
    probe = list(end.region)
    probe[2 * axis], probe[2 * axis + 1] = lo, hi
    return tuple(probe)


def bbox6(shape: Shape) -> Box6:
    bb = shape.bounding_box()
    return (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)


def check_route_ends(asm: Assembly) -> list[str]:
    """Each route is one connected run, and reaches each of its terminals
    from the side the terminal takes its wires from: it holds the space next
    to the terminal's part on that side (or, for a wire entry, the space
    above it)."""
    held = {h.name: h for h in asm.held}
    failures = []
    for route in asm.routes:
        if len(route.shape.solids()) != 1:
            failures.append(f"route '{route.name}': {len(route.shape.solids())} separate pieces, not one run")
        for end in route.ends:
            if sum(1 for c in end.side if c) != 1 or sum(abs(c) for c in end.side) != 1:
                failures.append(f"route '{route.name}': {end.name}'s side {end.side} is not an axis direction")
                continue
            box_ = None
            if end.held is not None:
                part = held.get(end.held)
                if part is None:
                    failures.append(f"route '{route.name}': no held part '{end.held}' for {end.name}")
                    continue
                box_ = bbox6(part.shape)
                inside = all(box_[2 * i] - 1e-3 <= end.region[2 * i] <= end.region[2 * i + 1] <= box_[2 * i + 1] + 1e-3
                             for i in range(3))
                if not inside:
                    failures.append(f"route '{route.name}': {end.name} {fmt(end.region)} is not on {end.held}")
                    continue
            probe = approach(end, box_)
            out = outside(box6(probe), route.shape)
            if out > VOLUME_TOL:
                where = "".join(f"{'+-'[c < 0]}{'xyz'[i]}" for i, c in enumerate(end.side) if c)
                failures.append(f"route '{route.name}' does not reach {end.name} from {where} "
                                f"({out:.2f} mm³ of {fmt(probe)} outside it)")
    return failures


class FrameMap:
    """The frame top's height map: the highest upward surface at each grid
    point, NaN where the frame is open (the battery bay below)."""

    def __init__(self, step: float = FRAME_GRID):
        import trimesh

        from reference_check import FRAME_STL, height_map

        self.step = step
        self.gx, self.gy, self.h = height_map(trimesh.load(FRAME_STL), step)

    def highest(self, x0: float, x1: float, y0: float, y1: float) -> float | None:
        """The frame's highest point under the rectangle, None if all open."""
        s = self.step / 2
        mask = (self.gx >= x0 - s) & (self.gx <= x1 + s) & (self.gy >= y0 - s) & (self.gy <= y1 + s)
        vals = self.h[mask]
        vals = vals[~np.isnan(vals)]
        return float(vals.max()) if len(vals) else None


def check_frame_clearance(asm: Assembly, frame: FrameMap | None = None) -> list[str]:
    """Each tie hole, slot and pass-through runs clear through its part and
    opens onto at least TIE_GAP of free space above the frame top."""
    if not asm.openings:
        return []
    frame = frame or FrameMap()
    parts = {p.name: p for p in asm.parts}
    failures = []
    for o in asm.openings:
        x0, x1, y0, y1 = o.footprint
        part = parts.get(o.part)
        if part is None:
            failures.append(f"{o.name}: no part '{o.part}' to pass through")
        else:
            v = common(part.shape, box(x0 + 0.05, x1 - 0.05, y0 + 0.05, y1 - 0.05, o.z0 - 0.5, o.z1 + 0.5))
            if v > VOLUME_TOL:
                failures.append(f"{o.name}: does not run clear through {o.part} ({v:.2f} mm³ in the way)")
        top = frame.highest(x0, x1, y0, y1)
        if top is not None and top > o.z0 - TIE_GAP + 1e-6:
            failures.append(f"{o.name}: the frame rises to z {top:.2f} under it, "
                            f"{o.z0 - top:.2f} below the underside at z {o.z0} (needs {TIE_GAP})")
    return failures


def edge_faces(shape: Shape) -> EdgeFaceMap:
    """Each edge of the shape mapped to the faces that share it."""
    found = EdgeFaceMap()
    TopExp.MapShapesAndAncestors_s(shape.wrapped, TopAbs_EDGE, TopAbs_FACE, found)
    return found


def anchor_points(face, up: Vector, ancestors, step: float = 0.25) -> np.ndarray:
    """Points along a downward face's edges where it hangs from material: a
    neighbouring face carries on below the point (a wall, a column, a 45
    degree face under it). Where the neighbour rises from the edge instead,
    the edge is free, so a face held only at one end is a cantilever."""
    u = np.array([up.X, up.Y, up.Z])
    found = []
    for edge in face.edges():
        if not ancestors.Contains(edge.wrapped):
            continue
        others = [Face(TopoDS.Face(w)) for w in list_shapes(ancestors.FindFromKey(edge.wrapped))]
        others = [f for f in others if not f.is_same(face)]
        for t in np.linspace(0, 1, max(3, int(edge.length / step) + 1)):
            p = np.array(tuple(edge.position_at(t)))
            below = Vertex(*(p - u * ANCHOR_PROBE))
            if any(below.distance_to(f) < ANCHOR_REACH for f in others):
                found.append(p)
    return np.array(found).reshape(-1, 3)


def overhang_failures(name: str, shapes: list[Shape], up) -> list[str]:
    """Downward faces steeper than OVERHANG_DEG in the print orientation
    `up`. A planar one passes on the bed, or if it is at most BRIDGE wide
    and no point of it lies more than BRIDGE from where it hangs from
    material (anchor_points(): a bridge between supports, or a short ledge;
    not an arm held at one end); a curved one never does (a round horizontal
    hole needs a teardrop)."""
    u = Vector(up).normalized()
    pts = np.concatenate([vertices(s) for s in shapes])
    bed = float((pts @ np.array([u.X, u.Y, u.Z])).min())
    failures = []
    for shape in shapes:
        ancestors = edge_faces(shape)
        for face in shape.faces():
            samples = face_samples(face)
            steep = [(p, n) for p, n in samples if n.dot(u) < -COS_OVERHANG - 1e-3]
            if not steep:
                continue
            p0 = steep[0][0]
            if face.geom_type == GeomType.PLANE:
                if all(abs(p.dot(u) - bed) < 0.01 for p, _ in samples):
                    continue  # on the bed
                edges = Compound(face.edges())
                width = 2 * max(Vertex(p.X, p.Y, p.Z).distance_to(edges) for p, _ in samples)
                if width > BRIDGE + 1e-3:
                    failures.append(f"{name}: a downward face {width:.1f} mm wide near {fmt(p0)} needs support")
                    continue
                anchors = anchor_points(face, u, ancestors)
                pts = np.array([tuple(p) for p, _ in samples] + [tuple(v) for v in face.vertices()])
                reach = (float(np.sqrt(((pts[:, None, :] - anchors[None]) ** 2).sum(-1)).min(axis=1).max())
                         if len(anchors) else math.inf)
                if reach > BRIDGE + 1e-3:
                    failures.append(f"{name}: a downward face near {fmt(p0)} reaches {reach:.1f} mm from what "
                                    f"it hangs from (at most {BRIDGE}): a cantilever, not a bridge")
            else:
                failures.append(f"{name}: a curved {face.geom_type.name.lower()} face looks down "
                                f"more steeply than {OVERHANG_DEG:.0f}° near {fmt(p0)} (a horizontal hole needs a teardrop)")
    return failures


def check_overhangs(asm: Assembly) -> list[str]:
    """No face needs support in its part's print orientation."""
    failures = []
    for part in asm.parts:
        shapes = [part.shape] + ([part.inlays] if part.inlays is not None else [])
        failures += overhang_failures(part.name, shapes, part.print_up)
    return failures


def check_screwdriver_paths(asm: Assembly) -> list[str]:
    """A DRIVER_D cylinder from each screw's head outwards along its axis
    meets no part fitted before it is driven."""
    failures = []
    for screw in asm.screws:
        path = cylinder(DRIVER_D / 2, screw.head, screw.axis, DRIVER_REACH)
        for label, shape in labelled_solids(asm):
            if base_name(label) in screw.after:
                continue
            v = common(shape, path)
            if v > VOLUME_TOL:
                failures.append(f"{screw.name}: {label} blocks the screwdriver path ({v:.2f} mm³)")
    return failures


def cross_section(b: Box6, axis: str) -> float:
    dx, dy, dz = b[1] - b[0], b[3] - b[2], b[5] - b[4]
    return {"x": dy * dz, "y": dx * dz, "z": dx * dy}[axis]


def overlap_box(a: Box6, b: Box6) -> Box6 | None:
    o = (max(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), min(a[3], b[3]), max(a[4], b[4]), min(a[5], b[5]))
    sizes = (o[1] - o[0], o[3] - o[2], o[5] - o[4])
    if min(sizes) < -1e-9 or sorted(sizes)[1] <= 1e-9:
        return None  # apart, or meeting along an edge
    return o


def flow(axis: str, turn: float) -> np.ndarray:
    """The unit direction of a flow along `axis` in a frame turned `turn`
    degrees about the vertical."""
    v = np.array(AXES[axis])
    c, s = math.cos(math.radians(turn)), math.sin(math.radians(turn))
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def junction(a: Box6, b: Box6, axis_a: str, axis_b: str, turn_a: float = 0.0, turn_b: float = 0.0) -> float:
    """The area air passes through from box a into box b: the smaller of
    their shared region's cross-sections across each one's flow. Boxes in
    the same frame may meet face to face; boxes in frames turned differently
    must overlap, and the shared region's cross-section across a flow is its
    widest section across that flow, as it is for a box."""
    if abs(turn_a - turn_b) < 1e-9:
        o = overlap_box(a, b)
        if o is None:
            return 0.0
        return min(cross_section(o, axis_a), cross_section(o, axis_b))
    shared = turned(a, turn_a) & turned(b, turn_b)
    if volume(shared) <= VOLUME_TOL:
        return 0.0
    return min(widest_slice(shared, flow(axis, turn)) for axis, turn in ((axis_a, turn_a), (axis_b, turn_b)))


def widest_slice(shape: Shape, direction: np.ndarray, n: int = 11) -> float:
    """The largest area of the shape's sections across `direction`, from n
    slices SLICE_T thick spread along it (for a box, every section's area)."""
    pts = vertices(shape)
    d = np.asarray(direction, dtype=float)
    along = pts @ d
    centre = pts.mean(axis=0)
    centre = centre - d * (centre @ d)
    size = 2 * float(np.ptp(pts, axis=0).max()) + 10
    best = 0.0
    for t in np.linspace(along.min() + SLICE_T, along.max() - SLICE_T, n):
        plane = Plane(origin=tuple(centre + d * t), z_dir=tuple(d))
        slab = Solid.make_box(size, size, SLICE_T).moved(Location((-size / 2, -size / 2, -SLICE_T / 2)))
        best = max(best, volume(shape & slab.moved(plane.location)) / SLICE_T)
    return best


def open_area(region: Shape, solids: list[Shape], across) -> float:
    """The smallest area left open through `region` by the solids, across
    the direction `across` (a grille's open area through its wall), from
    exact planar sections. Between two consecutive depths at which the
    region or the solids in it have a vertex, a section's area through
    planar faces is a quadratic in the depth: three sections fix it, and its
    minimum lies at an end or at the quadratic's turning point, where one
    more section is taken. So no depth where the opening narrows is missed,
    and a slot shut at any depth, however thin the skin, counts as shut."""
    d = np.asarray(across, dtype=float)
    d = d / np.linalg.norm(d)
    inside = [s & region for s in solids if common(s, region) > 0]
    pts = vertices(region)
    along = pts @ d
    lo, hi = float(along.min()), float(along.max())
    marks = np.concatenate([along] + [vertices(s) @ d for s in inside])
    marks = np.unique(np.round(marks[(marks >= lo) & (marks <= hi)], 6))
    centre = pts.mean(axis=0)
    centre = centre - d * (centre @ d)
    size = 2 * float(np.ptp(pts, axis=0).max()) + 10

    def open_at(t: float) -> float:
        plane = Face.make_rect(size, size, Plane(origin=tuple(centre + d * t), z_dir=tuple(d)))
        return (sum(f.area for f in (region & plane).faces())
                - sum(f.area for s in inside for f in (s & plane).faces()))

    eps = 1e-4
    least = math.inf
    for a, b in zip(marks[:-1], marks[1:]):
        if b - a <= 4 * eps:
            continue
        ts = np.array([a + eps, (a + b) / 2, b - eps])
        vals = np.array([open_at(t) for t in ts])
        least = min(least, float(vals.min()))
        k2, k1, _ = np.polyfit(ts, vals, 2)
        if k2 > 1e-9 and a + eps < -k1 / (2 * k2) < b - eps:
            least = min(least, open_at(-k1 / (2 * k2)))
    return least


def segments(ch) -> list[tuple[Box6, str, float]]:
    """The channel's segments as (box, axis, turn)."""
    return [(s[0], s[1], s[2] if len(s) > 2 else 0.0) for s in ch.segments]


def channel_solids(ch) -> list[Solid]:
    """The channel's inlet, segments and outlet as solids (for the preview)."""
    return ([turned(ch.inlet, ch.inlet_turn)] + [turned(b, t) for b, _, t in segments(ch)]
            + [turned(ch.outlet, ch.outlet_turn)])


def check_air_channel(asm: Assembly) -> list[str]:
    """The air channel runs connected from the inlet past the SZBK07 to the
    outlet, no part enters it, and its cross-section, along each segment
    and at each junction, is nowhere smaller than the inlet's net open area.
    Each vent's open area, measured through the cover, is the one it declares
    and no smaller than the inlet's."""
    ch = asm.channel
    if ch is None:
        return []
    failures = []
    area = ch.inlet_open_area
    segs = segments(ch)
    if area <= 0 or not segs:
        return ["air channel: no inlet area or no segments"]
    for i, (b, axis, _) in enumerate(segs):
        if cross_section(b, axis) < area - 1e-6:
            failures.append(f"air channel segment {i + 1}: cross-section {cross_section(b, axis):.0f} mm², "
                            f"smaller than the inlet's {area:.0f} mm²")
    regions = [("inlet", turned(ch.inlet, ch.inlet_turn))]
    regions += [(f"segment {i + 1}", turned(b, t)) for i, (b, _, t) in enumerate(segs)]
    regions.append(("outlet", turned(ch.outlet, ch.outlet_turn)))
    solids = list(labelled_solids(Assembly(parts=asm.parts)))
    solids += [(h.name, h.shape) for h in asm.held if h.name not in FANS]
    for name, region in regions:
        for label, shape in solids:
            v = common(shape, region)
            if v > VOLUME_TOL:
                failures.append(f"air channel {name}: {label} enters it ({v:.2f} mm³)")
    first, last = segs[0], segs[-1]
    joins = [("inlet", (ch.inlet, first[1], ch.inlet_turn), first, "segment 1")]
    for i in range(len(segs) - 1):
        joins.append((f"segment {i + 1}", segs[i], segs[i + 1], f"segment {i + 2}"))
    joins.append((f"segment {len(segs)}", last, (ch.outlet, ch.outlet_axis or last[1], ch.outlet_turn), "outlet"))
    for na, (a, xa, ta), (b, xb, tb), nb in joins:
        j = junction(a, b, xa, xb, ta, tb)
        if j < area - 1e-6:
            failures.append(f"air channel: {na} to {nb} passes {j:.0f} mm², less than the inlet's {area:.0f} mm²")
    covers = [s for p in asm.parts if p.role == "cover" for s in p.solids()]
    for vent in asm.vents if covers else []:
        if vent.across is None:
            failures.append(f"vent {vent.name}: no direction through the wall to measure its open area across")
            continue
        measured = open_area(vent.shape, covers, vent.across)
        if abs(measured - vent.open_area) > VENT_AREA_TOL * vent.open_area:
            failures.append(f"vent {vent.name}: the cover leaves {measured:.0f} mm² open through it, "
                            f"not the {vent.open_area:.0f} mm² declared")
        if measured < area - 1e-6:
            failures.append(f"air channel: vent {vent.name} leaves {measured:.0f} mm² open, "
                            f"less than the inlet's {area:.0f} mm²")
    past = next((h for h in asm.held if h.name == ch.past), None)
    if past is None:
        failures.append(f"air channel: no held part '{ch.past}' to pass")
    else:
        gap = min(turned(b, t).distance_to(past.shape) for b, _, t in segs)
        if gap > AIR_PAST:
            failures.append(f"air channel: passes {ch.past} no nearer than {gap:.1f} mm (needs {AIR_PAST})")
    return failures


def check_fans(asm: Assembly) -> list[str]:
    """Each fan position seats a fan behind its vent: the fan is supported
    at both ends and located at both end faces by parts other than the cover,
    which comes no nearer than FAN_COVER_GAP (it carries no fan); moved
    FAN_PLAY across its width or inwards it meets its holder, and outwards
    its holder or the cover's wall; rays from
    its outer face out along its axis pass the cover through a vent; an
    exhaust faces out of the box and an intake into it. A part set with an
    air channel has an exhaust position. Collisions are the overlap check's."""
    failures = []
    if asm.channel is not None and not any(f.exhaust for f in asm.fans):
        failures.append("fans: no exhaust fan position at the outlet")
    if not asm.fans:
        return failures
    held = {h.name: h for h in asm.held}
    covers = [s for p in asm.parts if p.role == "cover" for s in p.solids()]
    supports = [s for p in asm.parts if p.role not in ("cover", "bumper") for s in p.instances()]
    # A vent's air must pass the cover and the bumpers on it.
    cover_tris = triangles(covers + [s for p in asm.parts if p.role == "bumper" for s in p.instances()])
    for fan in asm.fans:
        if fan.name not in FANS:
            failures.append(f"{fan.name}: not a fan position ({', '.join(FANS)})")
        h = held.get(fan.name)
        if h is None:
            failures.append(f"{fan.name}: no held fan in its position")
        elif outside(fan.shape(), h.shape) > VOLUME_TOL or outside(h.shape, fan.shape()) > VOLUME_TOL:
            failures.append(f"{fan.name}: the held fan is not in its position")
        w, ht, t = fan.size
        for end, sign in (("one end", -1), ("the other end", 1)):
            x0, x1 = sorted((sign * w / 2, sign * (w / 2 - FAN_END)))
            under = fan.local(x0, x1, -t, 0, -FAN_SEAT_GAP, -0.01)
            if not any(common(s, under) > VOLUME_TOL for s in supports):
                failures.append(f"{fan.name}: nothing supports {end} from below")
            x0, x1 = sorted((sign * (w / 2 + 0.01), sign * (w / 2 + FAN_LOCATE)))
            beyond = fan.local(x0, x1, -t, 0, 0, ht / 2)
            if not any(common(s, beyond) > VOLUME_TOL for s in supports):
                failures.append(f"{fan.name}: nothing locates {end}")
        pl = fan.plane()
        for what, (dx, dy), within in (("across its width", (1, 0), supports), ("across its width", (-1, 0), supports),
                                       ("inwards", (0, -1), supports), ("outwards", (0, 1), supports + covers)):
            step = pl.x_dir * (dx * FAN_PLAY) + pl.y_dir * (dy * FAN_PLAY)
            moved = fan.shape().moved(Location(tuple(step)))
            if not any(common(s, moved) > VOLUME_TOL for s in within):
                failures.append(f"{fan.name}: it slides {FAN_PLAY} mm {what} without meeting its holder"
                                + (" or the cover" if dy > 0 else ""))
        near = fan.local(-w / 2 - FAN_COVER_GAP, w / 2 + FAN_COVER_GAP, -t - FAN_COVER_GAP, FAN_COVER_GAP,
                         -FAN_COVER_GAP, ht + FAN_COVER_GAP)
        if any(common(s, near) > VOLUME_TOL for s in covers):
            failures.append(f"{fan.name}: the cover comes within {FAN_COVER_GAP} mm of it (the cover carries no fan)")
        out = np.array(tuple(Vector(fan.outward).normalized()))
        pts = np.array([tuple(pl.from_local_coords((x, -0.01, z)))
                        for x in np.linspace(-w / 2 + 2, w / 2 - 2, 13) for z in np.linspace(2, ht - 2, 13)])
        share = 1.0 - ray_hits(pts, out, cover_tris).mean() if len(cover_tris) else 1.0
        if covers and share < FAN_VENT_OPEN:
            failures.append(f"{fan.name}: only {share:.0%} of its face sees out through the cover and the bumpers "
                            f"(needs {FAN_VENT_OPEN:.0%}): no vent in front of it, or a bumper over it")
        facing = out[0] * fan.origin[0] + out[1] * fan.origin[1]
        if facing <= 0:
            failures.append(f"{fan.name}: its vent side faces into the box, not out of it")
    return failures


def check_cover(asm: Assembly) -> list[str]:
    """The cover: every screen that must show sits below its top; it comes
    nearer above each held-down part than that part's stops are tall; its lid
    has no opening above the parts in CLOSED_OVER; and it lifts straight off,
    with the bumpers on it: nothing else lies above any face of it that looks
    up (rays up from them), and raised by each of LIFT_STEPS it overlaps
    nothing else."""
    covers = [p for p in asm.parts if p.role == "cover"]
    failures = []
    for cover in covers:
        solids = cover.solids()
        top = max(s.bounding_box().max.Z for s in solids)
        for h in asm.held:
            for e in h.exposed:
                if e.must_show and tuple(e.direction) == (0, 0, 1) and e.region[5] > top - 1e-6:
                    failures.append(f"{h.name}: its exposed face at z {e.region[5]:.2f} is not below "
                                    f"the {cover.name}'s top at z {top:.2f}")
            if h.held_down > 0:
                b = bbox6(h.shape)
                probe = box(b[0], b[1], b[2], b[3], b[5], b[5] + h.held_down)
                if sum(common(s, probe) for s in solids) <= VOLUME_TOL:
                    failures.append(f"{h.name}: the {cover.name} leaves {h.held_down} mm or more above it, "
                                    f"so it can lift out over its stops")
        tris = triangles(solids)
        for name in CLOSED_OVER:
            h = next((x for x in asm.held if x.name == name), None)
            if h is None:
                continue
            b = bbox6(h.shape)
            xs = np.arange(b[0] + CLOSED_STEP / 2, b[1], CLOSED_STEP)
            ys = np.arange(b[2] + CLOSED_STEP / 2, b[3], CLOSED_STEP)
            pts = np.array([(x, y, b[5] + 0.01) for x in xs for y in ys])
            open_ = ~ray_hits(pts, (0, 0, 1), tris)
            if open_.any():
                failures.append(f"{cover.name}: an opening in it above the {name} "
                                f"({int(open_.sum())} points, first at {fmt(pts[open_][0])})")
        # The bumpers ride on the cover's rails and lift off with it.
        riders = {p.name for p in asm.parts if p.role == "bumper"}
        others = [(label, shape) for label, shape in labelled_solids(asm)
                  if base_name(label) != cover.name and base_name(label) not in riders]
        # Lifting it, every face of it that looks up sweeps the space above:
        # nothing may lie there. Rays up from those faces meet nothing else.
        above = triangles(shape for _, shape in others)
        ups = np.array([(p.X, p.Y, p.Z) for s in solids for face in s.faces()
                        for p, n in face_samples(face) if n.Z > 1e-3])
        if len(ups) and len(above):
            hit = ray_hits(ups, (0, 0, 1), above)
            if hit.any():
                failures.append(f"{cover.name}: something lies above it ({int(hit.sum())} points, first over "
                                f"{fmt(ups[hit][0])}): it does not lift straight off")
        for d in LIFT_STEPS:
            lifted = [s.moved(Location((0, 0, d))) for s in solids]
            hit = next(((label, v) for label, shape in others
                        for v in [sum(common(m, shape) for m in lifted)] if v > VOLUME_TOL), None)
            if hit:
                failures.append(f"{cover.name}: lifted {d} mm, it meets {hit[0]} ({hit[1]:.2f} mm³): "
                                "it does not lift straight off")
                break
    return failures


def rect_gap(a: Box6, b: Box6) -> float:
    """The distance between two boxes' footprints (0 when they overlap)."""
    dx = max(a[0] - b[1], b[0] - a[1], 0.0)
    dy = max(a[2] - b[3], b[2] - a[3], 0.0)
    return math.hypot(dx, dy)


def wires_through(footprint, wires: int) -> bool:
    """Whether `wires` wires of SENSOR_WIRE_D, each with WIRE_CLEARANCE round
    it, pass side by side through a rectangular footprint."""
    d = M.SENSOR_WIRE_D + 2 * WIRE_CLEARANCE
    w, l = sorted((footprint[1] - footprint[0], footprint[3] - footprint[2]))
    return math.floor(w / d + 1e-9) * math.floor(l / d + 1e-9) >= wires


def route_between(asm: Assembly, a, b) -> bool:
    """Whether one route ends at a terminal matching a and at one matching b
    (each a predicate on a Terminal)."""
    return any(any(a(e) for e in r.ends) and any(b(e) for e in r.ends) for r in asm.routes)


def check_sensors(asm: Assembly, frame: FrameMap | None = None) -> list[str]:
    """The sensor provisions: the IMU breakout near the centre, clear of the
    SZBK07 and of every other held part, on a pad with two bosses, with a
    route to J6; each pass-through over open frame, taking its wires; each
    wire entry taking its wires, with a route from it to the Servo 2040's
    sensor header."""
    failures = []
    held = {h.name: h for h in asm.held}
    imu = held.get(IMU)
    if imu is not None or asm.imu_pad is not None:
        if imu is None:
            failures.append(f"IMU pad: no held part '{IMU}' on it")
        elif asm.imu_pad is None:
            failures.append(f"{IMU}: no pad")
        else:
            failures += imu_failures(asm, imu)
    for o in asm.openings:
        if o.kind not in ("entry", "pass-through"):
            continue
        if not wires_through(o.footprint, o.wires):
            failures.append(f"{o.name}: does not take {o.wires} wires of {M.SENSOR_WIRE_D} mm")
        if o.kind == "pass-through":
            frame = frame or FrameMap()
            top = frame.highest(*o.footprint)
            if top is not None:
                failures.append(f"{o.name}: the frame is not open under it (it rises to z {top:.2f})")
        elif not route_between(asm, lambda e, o=o: e.held is None and inside_footprint(e.region, o.footprint),
                               lambda e: e.held == SERVO_2040 and e.name == SENSOR_HEADER):
            failures.append(f"{o.name}: no route from it to the {SERVO_2040}'s {SENSOR_HEADER}")
    return failures


def inside_footprint(region: Box6, footprint) -> bool:
    return (footprint[0] - 1e-3 <= region[0] and region[1] <= footprint[1] + 1e-3
            and footprint[2] - 1e-3 <= region[2] and region[3] <= footprint[3] + 1e-3)


def imu_failures(asm: Assembly, imu) -> list[str]:
    failures = []
    pad = asm.imu_pad
    b = bbox6(imu.shape)
    far = max(math.hypot(x, y) for x in b[:2] for y in b[2:4])
    if far > IMU_MAX_R + 1e-6:
        failures.append(f"{IMU}: reaches {far:.1f} mm from the centre (at most {IMU_MAX_R})")
    szbk07 = next((h for h in asm.held if h.name == SZBK07), None)
    if szbk07 is None:
        failures.append(f"{IMU}: no held part '{SZBK07}' to keep clear of")
    else:
        gap = rect_gap(b, bbox6(szbk07.shape))
        if gap < IMU_SZBK07_GAP - 1e-6:
            failures.append(f"{IMU}: {gap:.1f} mm from the {SZBK07}'s outline (at least {IMU_SZBK07_GAP})")
    for other in asm.held:
        if other is not imu and common(imu.shape, other.shape) > VOLUME_TOL:
            failures.append(f"{IMU}: overlaps {other.name}")
    part = next((p for p in asm.parts if p.name == pad.part), None)
    if part is None:
        failures.append(f"IMU pad: no part '{pad.part}'")
        return failures
    if len(pad.bosses) != 2:
        failures.append(f"IMU pad: {len(pad.bosses)} bosses, not two")
    if not b[4] - 0.2 <= pad.top <= b[4] + 1e-6:
        failures.append(f"IMU pad: its bosses' top at z {pad.top} does not carry the {IMU} at z {b[4]:.2f}")
    for x, y in pad.bosses:
        if not (b[0] <= x <= b[1] and b[2] <= y <= b[3]):
            failures.append(f"IMU pad: boss at ({x}, {y}) is not under the {IMU}")
        ring = cylinder(pad.radius - 0.05, (x, y, pad.top - 1), (0, 0, 1), 1) - cylinder(
            pad.pilot_r + 0.05, (x, y, pad.top - 2), (0, 0, 1), 3)
        if common(part.shape, ring) < 0.5 * volume(ring):
            failures.append(f"IMU pad: no boss at ({x}, {y})")
        if common(part.shape, cylinder(pad.pilot_r - 0.05, (x, y, pad.top - 2), (0, 0, 1), 2.5)) > VOLUME_TOL:
            failures.append(f"IMU pad: no pilot hole in the boss at ({x}, {y})")
    if not route_between(asm, lambda e: e.held == IMU, lambda e: e.held == SERVO_2040 and e.name == J6):
        failures.append(f"{IMU}: no route to the {SERVO_2040}'s J6")
    return failures


def check_bumpers(asm: Assembly) -> list[str]:
    """The bumpers stand BUMPER_PROUD proud of the cover, every held part and
    the cover's screw heads in every horizontal direction and upwards (their
    tops above the lid and the screen); each is held on its rails by a screw
    into the cover, rests on its stop on the cover, slides on its rails, and
    with its screws out lifts off them with the cover in place, meeting
    nothing; each half of it, pulled off the cover, meets it within
    BUMPER_PULL (its rails keep it on)."""
    bumpers = [p for p in asm.parts if p.role == "bumper"]
    if not bumpers:
        return []
    failures = []
    pts = np.concatenate([vertices(s) for p in bumpers for s in p.instances()])
    covers = [p for p in asm.parts if p.role == "cover"]
    cover_names = {c.name for c in covers}
    instances = [s for p in bumpers for s in p.instances()]

    def on_a_bumper(screw) -> bool:
        return any(head_on(screw, s) for s in instances)

    others = [(c.name, vertices(s)) for c in covers for s in c.solids()]
    others += [(h.name, vertices(h.shape)) for h in asm.held]
    # The screws driven with the cover on (the cover's own), not the bumpers'.
    others += [(s.name, np.array([s.head])) for s in asm.screws
               if not cover_names & set(s.after) and not on_a_bumper(s)]
    dirs = [(math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0) for a in np.arange(0.0, 360.0, BUMPER_STEP)]
    for d in dirs + [(0.0, 0.0, 1.0)]:
        v = np.asarray(d)
        reach = float((pts @ v).max())
        name, far = max(((n, float((p @ v).max())) for n, p in others), key=lambda t: t[1], default=(None, -math.inf))
        if reach < far + BUMPER_PROUD - 1e-6:
            where = "upwards (their tops)" if d[2] else f"towards {math.degrees(math.atan2(d[1], d[0])) % 360:.0f}°"
            failures.append(f"bumpers: {where} they reach {reach:.2f}, not {BUMPER_PROUD} past the {name} at {far:.2f}")
    solids = list(labelled_solids(asm))
    pulls = np.arange(BUMPER_PULL_STEP, BUMPER_PULL + 1e-9, BUMPER_PULL_STEP)
    for part in bumpers:
        locs = [Location()] + list(part.copies)
        for i, inst in enumerate(part.instances()):
            label = part.name if i == 0 else f"{part.name} copy {i + 1}"
            rest = [(lb, s) for lb, s in solids if lb != label]
            on = [s for lb, s in rest if base_name(lb) in cover_names]
            if part.outward is None:
                failures.append(f"{label}: it declares no way off the cover to check its rails against")
            elif on:
                origin = Vertex(0, 0, 0).moved(locs[i]).center()
                out = Vertex(*part.outward).moved(locs[i]).center() - origin
                # Each half (either side of its centre, across the pull) must
                # catch on its own: one rail holding cannot hide the other's.
                side = Vector(-out.Y, out.X, 0.0).normalized()
                for sign, end in ((1.0, "one end"), (-1.0, "the other end")):
                    plane = Plane(origin=inst.center(), x_dir=side * sign, z_dir=(0, 0, 1))
                    half = inst & Solid.make_box(400, 800, 800).moved(Location((0, -400, -400))).moved(plane.location)
                    if not any(common(half.moved(Location(tuple(out * float(d)))), s) > VOLUME_TOL
                               for d in pulls for s in on):
                        failures.append(f"{label}: {end}, pulled up to {BUMPER_PULL} mm off the cover, meets nothing: "
                                        "it comes off its rail")
            if covers and not any(retains(sc, inst, covers) for sc in asm.screws):
                failures.append(f"{label}: no screw holds it on its rails: turned over, it slides off")
            lowered = inst.moved(Location((0, 0, -BUMPER_STOP)))
            if covers and not any(common(lowered, s) > VOLUME_TOL for lb, s in rest if base_name(lb) in cover_names):
                failures.append(f"{label}: lowered {BUMPER_STOP} mm it meets no cover: nothing stops it on its rails")
            for dz in LIFT_STEPS:
                hit = next((lb for lb, s in rest if common(inst.moved(Location((0, 0, dz))), s) > VOLUME_TOL), None)
                if hit:
                    failures.append(f"{label}: lifted {dz} mm it meets {hit}: it does not come off with the cover on")
                    break
            for dx, dy in ((BUMPER_PLAY, 0), (-BUMPER_PLAY, 0), (0, BUMPER_PLAY), (0, -BUMPER_PLAY)):
                hit = next((lb for lb, s in rest if common(inst.moved(Location((dx, dy, 0))), s) > VOLUME_TOL), None)
                if hit:
                    failures.append(f"{label}: moved {fmt((dx, dy))} it meets {hit}: no sliding fit")
                    break
    return failures


def head_on(screw, shape: Shape) -> bool:
    return Vertex(*screw.head).distance_to(shape) <= max(SCREW_ON, screw.shank[0])


def retains(screw, inst: Shape, covers) -> bool:
    """Whether a screw's head lies on the bumper `inst` and its shank reaches
    from there into the cover."""
    d, length = screw.shank
    if length <= 0 or not head_on(screw, inst):
        return False
    axis = -np.asarray(screw.axis, dtype=float)
    shank = cylinder(d / 2, screw.head, tuple(axis), length)
    return any(common(shank, s) > VOLUME_TOL for c in covers for s in c.solids())


def check_captive(asm: Assembly) -> list[str]:
    """A loose part in a pocket (`held_down`) cannot lift out of it: it can
    rise less than its pocket's depth. A part rises until something fixed
    stops it; meeting a loose part or a held part it or another loose part
    carries, it pushes that up too, which rises only as far as it can itself
    (a stack: the LM2596's standoffs under the module, under the T-Display's
    holder, under the board, under the lid). Clearances are measured with
    rays straight up from every face that looks up."""
    solids = dict(labelled_solids(asm))
    loose = {}
    for part in asm.parts:
        if part.held_down > 0:
            for i in range(len(part.instances())):
                loose[part.name if i == 0 else f"{part.name} copy {i + 1}"] = part
    movable = set(loose) | {n for part in loose.values() for n in part.carries if n in solids}
    tris = {label: triangles([shape]) for label, shape in solids.items()}
    boxes = {label: shape.bounding_box() for label, shape in solids.items()}
    ups = {}

    def up_points(label: str) -> np.ndarray:
        if label not in ups:
            pts = [(p.X, p.Y, p.Z) for face in solids[label].faces() for p, n in face_samples(face) if n.Z > 1e-3]
            ups[label] = np.array(pts) if pts else np.zeros((0, 3))
        return ups[label]

    memo: dict[str, float] = {}

    def rise(label: str, path: tuple = ()) -> float:
        """How far `label` can rise, pushing what lies over it."""
        if label not in movable:
            return 0.0
        if label in memo:
            return memo[label]
        pts, b = up_points(label), boxes[label]
        best = math.inf
        for other, ob in boxes.items():
            if other == label or other in path or ob.max.Z <= b.min.Z or ob.max.X <= b.min.X or b.max.X <= ob.min.X \
                    or ob.max.Y <= b.min.Y or b.max.Y <= ob.min.Y:
                continue
            gap = float(ray_distances(pts, (0, 0, 1), tris[other]).min()) if len(pts) else math.inf
            if gap < best:
                best = min(best, gap + rise(other, path + (label,)))
        memo[label] = best
        return best

    failures = []
    for label, part in loose.items():
        r = rise(label)
        if r >= part.held_down - 1e-6:
            what = "nothing stops it" if math.isinf(r) else f"it rises {r:.2f} mm before it is stopped"
            failures.append(f"{label}: {what}, not less than its pocket's {part.held_down} mm: it can lift out")
    return failures


def check_supports(asm: Assembly) -> list[str]:
    """Each support bears on its held part's underside in every one of its
    regions (at least CONTACT_AREA where the part comes within CONTACT_GAP
    under `face` and the held part within CONTACT_GAP over it), nowhere else
    under it, and locates it: moved LOCATE along x or y, the held part meets
    it."""
    failures = []
    parts = {p.name: p for p in asm.parts}
    held = {h.name: h for h in asm.held}
    for sup in asm.supports:
        part, h = parts.get(sup.part), held.get(sup.held)
        if part is None or h is None:
            failures.append(f"support: no part '{sup.part}' or held part '{sup.held}'")
            continue
        z0 = sup.face - CONTACT_GAP
        for r in sup.regions:
            # Where the part reaches into the slab under `face` and the held
            # part into the slab over it, one over the other.
            below = part.shape & box(*r, z0, sup.face)
            above = h.shape & box(*r, sup.face, sup.face + CONTACT_GAP)
            area = 0.0
            if volume(below) > 0 and volume(above) > 0:
                area = common(below.moved(Location((0, 0, CONTACT_GAP))), above) / CONTACT_GAP
            if area < CONTACT_AREA:
                failures.append(f"{part.name}: bears on the {h.name} over {area:.1f} mm² at {fmt(r)} "
                                f"(at least {CONTACT_AREA})")
        b = bbox6(h.shape)
        band = box(b[0], b[1], b[2], b[3], z0, sup.face)
        for r in sup.regions:
            band = band - box(*r, z0 - 1, sup.face + 1)
        v = common(part.shape, band)
        if v > VOLUME_TOL:
            failures.append(f"{part.name}: bears on the {h.name} outside its regions ({v:.2f} mm³)")
        for dx, dy in ((LOCATE, 0), (-LOCATE, 0), (0, LOCATE), (0, -LOCATE)):
            if common(h.shape.moved(Location((dx, dy, 0))), part.shape) <= VOLUME_TOL:
                failures.append(f"{part.name}: the {h.name} moved {fmt((dx, dy))} does not meet it: not located")
    return failures


def run_checks(asm: Assembly, frame: FrameMap | None = None) -> list[str]:
    if frame is None and asm.openings:
        frame = FrameMap()
    failures = []
    failures += check_single_solid(asm)
    failures += check_fit(asm)
    failures += check_insert_holes(asm)
    failures += check_overlaps(asm)
    failures += check_enclosure(asm)
    failures += check_routes(asm)
    failures += check_route_ends(asm)
    failures += check_frame_clearance(asm, frame)
    failures += check_sensors(asm, frame)
    failures += check_overhangs(asm)
    failures += check_screwdriver_paths(asm)
    failures += check_air_channel(asm)
    failures += check_fans(asm)
    failures += check_cover(asm)
    failures += check_bumpers(asm)
    failures += check_captive(asm)
    failures += check_supports(asm)
    return failures

