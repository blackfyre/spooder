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
from build123d import Compound, GeomType, Plane, Shape, Solid, Vector, Vertex

import measurements as M
from model import FANS, LM2596_OUTPUT_ROUTE, SCHOTTKY, Assembly, Box6, box, box6

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
# Surface samples are spaced about this far apart. Source: chosen, finer than
# the narrowest feature the checks look for (a 3 mm bridge).
SAMPLE_STEP = 1.5

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


def labelled_solids(asm: Assembly):
    """(label, shape) for every printed copy, its inlays and every held part."""
    for part in asm.parts:
        for i, shape in enumerate(part.instances()):
            yield (part.name if i == 0 else f"{part.name} copy {i + 1}"), shape
        if part.inlays is not None:
            yield f"{part.name} inlays", part.inlays
    for held in asm.held:
        yield held.name, held.shape


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
    """Every held part is enclosed by the cover: rays from its surface along
    ±x, ±y and +z all meet the cover, except from its exposed regions in
    their direction, which (when they must show) meet nothing of it."""
    covers = [p for p in asm.parts if p.role == "cover"]
    if not covers or not asm.held:
        return []
    tris = triangles(s for c in covers for s in c.solids())
    failures = []
    for held in asm.held:
        open_dirs: dict[tuple, list] = {}
        shown: dict[int, int] = {}
        for face in held.shape.faces():
            for p, n in face_samples(face, step=3.0, minimum=3):
                for d in RAY_DIRECTIONS:
                    if n.dot(Vector(d)) <= 0.5:
                        continue
                    exp = next((k for k, e in enumerate(held.exposed)
                                if tuple(e.direction) == d and in_box(p, e.region)), None)
                    hit = bool(ray_hits(np.array([[p.X, p.Y, p.Z]]), d, tris)[0])
                    if exp is not None:
                        if held.exposed[exp].must_show:
                            shown[exp] = shown.get(exp, 0) + 1
                            if hit:
                                open_dirs.setdefault(("covered", d), []).append(p)
                    elif not hit:
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
    for route in asm.routes:
        for label, shape in labelled_solids(asm):
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


def overhang_failures(name: str, shapes: list[Shape], up) -> list[str]:
    """Downward faces steeper than OVERHANG_DEG in the print orientation
    `up`. A planar one passes on the bed or if at most BRIDGE wide; a curved
    one never does (a round horizontal hole needs a teardrop)."""
    u = Vector(up).normalized()
    pts = np.concatenate([vertices(s) for s in shapes])
    bed = float((pts @ np.array([u.X, u.Y, u.Z])).min())
    failures = []
    for shape in shapes:
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
                if width <= BRIDGE + 1e-3:
                    continue
                failures.append(f"{name}: a downward face {width:.1f} mm wide near {fmt(p0)} needs support")
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
            if label.split(" copy ")[0].removesuffix(" inlays") in screw.after:
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


def junction(a: Box6, b: Box6, axis_a: str, axis_b: str) -> float:
    """The area air passes through from box a into box b: the smaller of
    their shared region's cross-sections across each one's flow."""
    o = overlap_box(a, b)
    if o is None:
        return 0.0
    return min(cross_section(o, axis_a), cross_section(o, axis_b))


def check_air_channel(asm: Assembly) -> list[str]:
    """The air channel runs connected from the inlet past the SZBK07 to the
    outlet, no part enters it, and its cross-section, along each segment
    and at each junction, is nowhere smaller than the inlet's net open area."""
    ch = asm.channel
    if ch is None:
        return []
    failures = []
    area = ch.inlet_open_area
    if area <= 0 or not ch.segments:
        return ["air channel: no inlet area or no segments"]
    for i, (b, axis) in enumerate(ch.segments):
        if cross_section(b, axis) < area - 1e-6:
            failures.append(f"air channel segment {i + 1}: cross-section {cross_section(b, axis):.0f} mm², "
                            f"smaller than the inlet's {area:.0f} mm²")
    regions = [("inlet", ch.inlet)] + [(f"segment {i + 1}", b) for i, (b, _) in enumerate(ch.segments)]
    regions.append(("outlet", ch.outlet))
    solids = list(labelled_solids(Assembly(parts=asm.parts)))
    solids += [(h.name, h.shape) for h in asm.held if h.name not in FANS]
    for name, b in regions:
        region = box6(b)
        for label, shape in solids:
            v = common(shape, region)
            if v > VOLUME_TOL:
                failures.append(f"air channel {name}: {label} enters it ({v:.2f} mm³)")
    joins = [("inlet", ch.inlet, ch.segments[0][1], ch.segments[0][0], ch.segments[0][1], "segment 1")]
    for i in range(len(ch.segments) - 1):
        (a, xa), (b, xb) = ch.segments[i], ch.segments[i + 1]
        joins.append((f"segment {i + 1}", a, xa, b, xb, f"segment {i + 2}"))
    last, xl = ch.segments[-1]
    joins.append((f"segment {len(ch.segments)}", last, xl, ch.outlet, xl, "outlet"))
    for na, a, xa, b, xb, nb in joins:
        j = junction(a, b, xa, xb)
        if j < area - 1e-6:
            failures.append(f"air channel: {na} to {nb} passes {j:.0f} mm², less than the inlet's {area:.0f} mm²")
    past = next((h for h in asm.held if h.name == ch.past), None)
    if past is None:
        failures.append(f"air channel: no held part '{ch.past}' to pass")
    else:
        gap = min(box6(b).distance_to(past.shape) for b, _ in ch.segments)
        if gap > AIR_PAST:
            failures.append(f"air channel: passes {ch.past} no nearer than {gap:.1f} mm (needs {AIR_PAST})")
    return failures


def run_checks(asm: Assembly, frame: FrameMap | None = None) -> list[str]:
    failures = []
    failures += check_single_solid(asm)
    failures += check_fit(asm)
    failures += check_insert_holes(asm)
    failures += check_overlaps(asm)
    failures += check_enclosure(asm)
    failures += check_routes(asm)
    failures += check_frame_clearance(asm, frame)
    failures += check_overhangs(asm)
    failures += check_screwdriver_paths(asm)
    failures += check_air_channel(asm)
    return failures

