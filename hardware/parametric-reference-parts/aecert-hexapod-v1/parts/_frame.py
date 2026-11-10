"""The frame's shared plan: Frame Top and Frame Bottom have the same outline,
the same six arms and the same coordinate frame (the body frame: y forward,
z up, the coxa pivots at z = 0).

Each arm is modelled once, along +x, and placed at the gait's mount angles:
the middle arms along the x axis, the corner arms at ±56° from it, set
ARM_OFFSET further out (README: the corner pivots are 108.03 mm from the
centre, the middle ones 97.50). Between the arms the outline is closed by
concave arcs tangent to the arms' sides. Dimensions are measured from
`Frame Top.stl` and `Frame Bottom.stl` with trimesh (horizontal sections at
z 18 and -14.5, line and circle fits to the section loops); the two STLs
agree on every value here to 0.001 mm.
"""

import math

from build123d import (Align, Axis, CenterArc, Kind, Line, Plane, Polygon, Pos, Rectangle, Rot, Vector, extrude,
                       make_face, offset, revolve)

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The arms. Source: the middle arm's end at x 116.10 (sections at z 18 and
# -14.5), the corner arm's at 126.63 along its axis; both 37 mm wide with r7
# end corners.
MIDDLE_END = 116.1
ARM_OFFSET = 10.53
CORNER_ANGLE = 56.0  # degrees from the x axis (README: the gait's mount angles)
ARM_HALF_WIDTH = 18.5
ARM_END_RADIUS = 7.0
# Each arm's coxa servo slot, open at its end: 20 mm wide from 67.6 mm out
# (middle arm, section at z 18).
SLOT_START = 67.6
SLOT_HALF_WIDTH = 10.0
# The bearing axle mount's hole pair on each arm (README): 28.5 mm apart,
# 87.60 mm out on the middle arms.
MOUNT_HOLE_X = 87.6
MOUNT_HOLE_HALF_SPACING = 14.25

# The outline between the arms (section at z 18), each arc tangent to an arm's
# side: from the middle arm an r25 arc (its centre at x 96.82), a straight
# tangent, then an r40 arc onto the corner arm (its centre 107.398 mm out
# along that arm); between the two front (or rear) corner arms an r29 arc onto
# the straight front edge at y 83.5.
SIDE_ARC_RADIUS = 25.0
SIDE_ARC_X = 96.82
CORNER_ARC_RADIUS = 40.0
CORNER_ARC_AXIAL = 107.398
FRONT_ARC_RADIUS = 29.0
FRONT_EDGE_Y = 83.5


# The open middle, inside Frame Top's rim and Frame Bottom's walls, one
# quadrant (Frame Top, sections at z 20.5 to 24.5): a line at
# x 51.1 from the middle arm's channel, an arc A tangent to it, then an arc C
# that crosses the corner arm's channel and turns tangent to the front at
# y 56.70. A's vertices fit a circle to 0.001 mm; C's fit one to 0.02 mm (the
# STL's own facets on it are not exactly circular), so C is the fitted
# circle.
INNER_SIDE_X = 51.1
ARC_A_CENTRE = (8.8, 12.109)
ARC_A_RADIUS = 42.3
ARC_C_CENTRE = (6.713, 12.478)
ARC_C_RADIUS = 44.22

# The screws that hold Frame Top to Frame Bottom (Frame Top's countersunk
# holes, Frame Bottom's insert holes; sections at z 8 and 18): on the middle
# arms' axes 55.1 mm out, on the corner arms' 63.97 mm out, and at
# (±60.481, ±32).
SCREW_MIDDLE_X = 55.1
SCREW_CORNER_R = 63.97
SCREW_BETWEEN = (60.481, 32.0)


def polar(radius, angle):
    a = math.radians(angle)
    return radius * math.cos(a), radius * math.sin(a)


def arm_places():
    """(angle, axial offset) for each arm: the middle arms at 0° and 180°, the
    corner arms at ±56° and 180° ∓ 56°, ARM_OFFSET further out."""
    c = CORNER_ANGLE
    return [(0.0, 0.0), (180.0, 0.0), (c, ARM_OFFSET), (-c, ARM_OFFSET), (180.0 - c, ARM_OFFSET),
            (180.0 + c, ARM_OFFSET)]


def place(shape, angle, out):
    """A shape modelled on the middle arm along +x, moved `out` further along
    the arm and turned to `angle`."""
    return Rot(0, 0, angle) * Pos(out, 0, 0) * shape


def mirrored4(shape):
    """A shape modelled in the +x +y quadrant with its mirror images in both
    axes."""
    return [shape, shape.mirror(Plane.YZ), shape.mirror(Plane.XZ), shape.mirror(Plane.YZ).mirror(Plane.XZ)]


def _tangent_side_corner():
    """The straight tangent between the r25 and r40 arcs: its two points."""
    c1 = (SIDE_ARC_X, ARM_HALF_WIDTH + SIDE_ARC_RADIUS)
    u = polar(1, CORNER_ANGLE)
    v = (-u[1], u[0])
    lateral = -(ARM_HALF_WIDTH + CORNER_ARC_RADIUS)
    c2 = (CORNER_ARC_AXIAL * u[0] + lateral * v[0], CORNER_ARC_AXIAL * u[1] + lateral * v[1])
    # Both arcs lie outside the frame, so the tangent is the one whose normal
    # n gives (c2 - c1) . n = r1 - r2, on the frame's side.
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    alpha = math.atan2(dy, dx)
    phi = alpha + math.acos((SIDE_ARC_RADIUS - CORNER_ARC_RADIUS) / math.hypot(dx, dy))
    n = (math.cos(phi), math.sin(phi))
    return c1, c2, (c1[0] + SIDE_ARC_RADIUS * n[0], c1[1] + SIDE_ARC_RADIUS * n[1]), \
        (c2[0] + CORNER_ARC_RADIUS * n[0], c2[1] + CORNER_ARC_RADIUS * n[1])


def arm_end(lower_from, upper_to):
    """One arm's end, modelled on the middle arm along +x: its two sides
    from `lower_from` and to `upper_to` along it, the r7 corners and the
    end face, as edges in order."""
    w, r, e = ARM_HALF_WIDTH, ARM_END_RADIUS, MIDDLE_END
    return [
        Line((lower_from, -w), (e - r, -w)),
        CenterArc((e - r, -(w - r)), r, -90, 90),
        Line((e, -(w - r)), (e, w - r)),
        CenterArc((e - r, w - r), r, 0, 90),
        Line((e - r, w), (upper_to, w)),
    ]


def outline():
    """The frame's outline: the arm's end placed at each of the six arms,
    and between them the hub's arcs, drawn in the +x +y quadrant and
    mirrored in both axes."""
    c1, c2, t1, t2 = _tangent_side_corner()
    u = polar(1, CORNER_ANGLE)
    v = (-u[1], u[0])

    def corner(axial, lateral):
        return (axial * u[0] + lateral * v[0], axial * u[1] + lateral * v[1])

    def angle(c, p):
        return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))

    w = ARM_HALF_WIDTH
    # The r40 arc meets the corner arm's side opposite its centre; the r29
    # arc's centre is tangent to the corner arm's other side and the front.
    t3 = corner(CORNER_ARC_AXIAL, -w)
    lateral = w + FRONT_ARC_RADIUS
    front_axial = (FRONT_EDGE_Y + FRONT_ARC_RADIUS - lateral * v[1]) / u[1]
    c3 = corner(front_axial, lateral)
    t4 = corner(front_axial, w)
    t5 = (c3[0], FRONT_EDGE_Y)
    a2 = angle(c1, t1)
    a3 = angle(c2, t2)
    hub = [
        CenterArc(c1, SIDE_ARC_RADIUS, -90, a2 - (-90) - 360 if a2 > -90 else a2 + 90),
        Line(t1, t2),
        CenterArc(c2, CORNER_ARC_RADIUS, a3, (CORNER_ANGLE + 90) - (a3 + 360 if a3 < 0 else a3)),
        CenterArc(c3, FRONT_ARC_RADIUS, CORNER_ANGLE - 90, -CORNER_ANGLE),
        Line(t5, (0, FRONT_EDGE_Y)),
    ]
    hub += [e.mirror(Plane.YZ) for e in hub]
    hub += [e.mirror(Plane.XZ) for e in hub]
    # Each arm's sides run in to the hub's arcs: a middle arm's to the r25
    # arcs, a corner arm's to the r40 arc on the side towards its middle
    # arm and to the r29 arc on the other.
    r40, r29 = CORNER_ARC_AXIAL - ARM_OFFSET, front_axial - ARM_OFFSET
    sides = {0.0: (SIDE_ARC_X, SIDE_ARC_X), 180.0: (SIDE_ARC_X, SIDE_ARC_X),
             CORNER_ANGLE: (r40, r29), 180.0 + CORNER_ANGLE: (r40, r29),
             180.0 - CORNER_ANGLE: (r29, r40), -CORNER_ANGLE: (r29, r40)}
    arms = [place(e, a, off) for a, off in arm_places() for e in arm_end(*sides[a])]
    return make_face(hub + arms).clean()


def slots():
    """The six arms' coxa servo slots, open at the arms' ends."""
    length = MIDDLE_END - SLOT_START + 1.0  # past the end
    slot = Pos(SLOT_START + length / 2, 0) * Rectangle(length, 2 * SLOT_HALF_WIDTH)
    return [place(slot, a, off) for a, off in arm_places()]


def mount_holes():
    """The centres of the bearing axle mounts' hole pairs, two per arm."""
    out = []
    for a, off in arm_places():
        for y in (-MOUNT_HOLE_HALF_SPACING, MOUNT_HOLE_HALF_SPACING):
            p = Rot(0, 0, a) * Pos(MOUNT_HOLE_X + off, y, 0)
            out.append((p.position.X, p.position.Y))
    return out



def halfplane(point, angle, size=1000.0):
    """The half-plane to the left of the line through `point` running at
    `angle` degrees, as a large square."""
    return Pos(*point) * Rot(0, 0, angle) * Pos(0, size / 2) * Rectangle(size, size)


def sharp_corners(face, tolerance=1.0):
    """The vertices of a face where its boundary turns by more than
    `tolerance` degrees (not where an arc meets a line tangentially)."""
    out = []
    for v in face.vertices():
        tangents = []
        for e in face.edges():
            for t in (0.0, 1.0):
                p = e @ t
                if (p - v.center()).length < 1e-6:
                    d = e % t
                    tangents.append(d if t == 0.0 else -d)
        if len(tangents) == 2:
            a, b = tangents
            # Opposite directions (a = -b) mean the boundary runs straight on.
            if a.get_angle(-b) > tolerance:  # degrees
                out.append(v)
    return out


def mirror_points(point):
    """A point in the +x +y quadrant and its mirror images in both axes."""
    x, y = point
    return [(x, y), (-x, y), (x, -y), (-x, -y)]


def chamfer_tool(face, size, z, up=True):
    """The material a 45° chamfer of `size` removes round the edges of a hole
    with outline `face` in a face at height `z`: the outline grown by `size`
    at `z`, tapering to the outline `size` above it (or below, `up=False`)."""
    tool = extrude(offset(face, size, kind=Kind.ARC), size, taper=45)
    if not up:
        tool = tool.mirror(Plane.XY)
    return Pos(0, 0, z) * tool


def circle_intersection(c1, r1, c2, r2, upper=True):
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(r1 * r1 - a * a)
    mx, my = c1[0] + a * dx / d, c1[1] + a * dy / d
    p = (mx - h * dy / d, my + h * dx / d)
    q = (mx + h * dy / d, my - h * dx / d)
    return max(p, q, key=lambda t: t[1]) if upper else min(p, q, key=lambda t: t[1])


def inner_quadrant():
    """The open middle inside the rim, in the +x +y quadrant."""
    ax, ay = ARC_A_CENTRE
    cx, cy = ARC_C_CENTRE
    meet = circle_intersection(ARC_A_CENTRE, ARC_A_RADIUS, ARC_C_CENTRE, ARC_C_RADIUS)
    a_end = math.degrees(math.atan2(meet[1] - ay, meet[0] - ax))
    c_start = math.degrees(math.atan2(meet[1] - cy, meet[0] - cx))
    front = cy + ARC_C_RADIUS
    return make_face([
        Line((0, 0), (INNER_SIDE_X, 0)),
        Line((INNER_SIDE_X, 0), (INNER_SIDE_X, ay)),
        CenterArc(ARC_A_CENTRE, ARC_A_RADIUS, 0, a_end),
        CenterArc(ARC_C_CENTRE, ARC_C_RADIUS, c_start, 90 - c_start),
        Line((cx, front), (0, front)),
        Line((0, front), (0, 0)),
    ])


def frame_screws():
    """The centres of the ten screws between Frame Top and Frame Bottom."""
    return ([(SCREW_MIDDLE_X, 0.0), (-SCREW_MIDDLE_X, 0.0)] + mirror_points(polar(SCREW_CORNER_R, CORNER_ANGLE))
            + mirror_points(SCREW_BETWEEN))


def front_arc_centre():
    """The centre of the r29 arc between the front corner arms, in the +x +y
    quadrant."""
    u = polar(1, CORNER_ANGLE)
    v = (-u[1], u[0])
    lateral = ARM_HALF_WIDTH + FRONT_ARC_RADIUS
    axial = (FRONT_EDGE_Y + FRONT_ARC_RADIUS - lateral * v[1]) / u[1]
    return (axial * u[0] + lateral * v[0], axial * u[1] + lateral * v[1])


def side_tangent():
    """The straight piece of the outline between the r25 and r40 arcs, in
    the +x +y quadrant: its two ends."""
    return _tangent_side_corner()[2:]


def union(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out + s
    return out


def _edge_faces(solid):
    faces_of = {}
    for f in solid.faces():
        for e in f.edges():
            faces_of.setdefault(hash(e), []).append(f)
    return faces_of


def convex_horizontal_edges(solid, skip_radius=None):
    """The edges of a solid that lie in a horizontal plane and where its
    surface turns outward (a convex corner): the ones a chamfer cuts. A
    circular edge of `skip_radius` (a hole) is left out."""
    faces_of = _edge_faces(solid)
    out = []
    for e in solid.edges():
        if e.bounding_box().size.Z > 1e-6:
            continue
        if skip_radius is not None and e.geom_type.name == "CIRCLE" and abs(e.radius - skip_radius) < 1e-6:
            continue
        faces = faces_of.get(hash(e), [])
        if len(faces) != 2:
            continue
        # At a convex edge the material lies behind both faces, so a point in
        # front of one face and behind the other is outside it; at a concave
        # edge that point is inside.
        p = e @ 0.5
        n1, n2 = faces[0].normal_at(p), faces[1].normal_at(p)
        if (n1 - n2).length < 1e-6:
            continue
        if not solid.is_inside(p + (n2 - n1).normalized() * 0.05):
            out.append(e)
    return out


# How far, in chamfer sizes, a chamfer's wedge runs on past a tangent join
# with an edge that is not chamfered.
RUN_ON = 6.0


def chamfered(solid, size, edges=None, clip=None, rounded=False):
    """The solid with a 45° chamfer of `size` on each of `edges` (by default
    every convex horizontal edge); see `chamfer_wedges`. With `clip`, a
    solid, the chamfers cut only inside it: where a chamfered edge runs
    into a wall standing on the chamfered face, the chamfer stops at the
    wall. With `rounded`, a round of radius `size` instead of a chamfer."""
    edges = convex_horizontal_edges(solid) if edges is None else edges
    tools = chamfer_wedges(solid, size, edges, rounded)
    if clip is not None:
        tools = [t & clip for t in tools]
        tools = [t for t in tools if t.volume > 1e-9]
    if not tools:
        return solid
    # Overlapping wedges cut in one operation can leave a defective shape;
    # then fuse them first, or cut them one at a time.
    result = solid.cut(*tools)
    if result.is_valid:
        return result
    result = solid.cut(tools[0].fuse(*tools[1:]).clean() if len(tools) > 1 else tools[0])
    if result.is_valid:
        return result
    for t in tools:
        solid = solid.cut(t)
    return solid


def chamfer_wedges(solid, size, edges, rounded=False):
    """The wedges a 45° chamfer of `size` on each of `edges` cuts: for each
    edge a triangle swept along it (turned about its centre if it is round);
    at each re-entrant corner between two chamfered edges, where the
    chamfers meet beyond the corner, the meet of the two wedges run on past
    it; over a convex round smaller than the chamfer, the sector outside a
    45° cone that shrinks the round to a point. Where a chamfered edge runs
    on tangent into an edge that is not chamfered, its wedge runs on past
    the join (RUN_ON chamfer sizes), as the chamfer's face does until it
    leaves the material."""
    faces_of = _edge_faces(solid)
    eps = 0.01
    keys = {hash(e) for e in edges}
    z_level = edges[0].center().Z if edges else 0.0
    level = [e for e in solid.edges() if e.bounding_box().size.Z < 1e-6 and abs(e.center().Z - z_level) < 1e-6]

    def run_on(e, end):
        v = e @ end
        t = (e % end) * (1 if end == 1.0 else -1)
        for o in level:
            if hash(o) in keys:
                continue
            for oe in (0.0, 1.0):
                if ((o @ oe) - v).length < 1e-6:
                    to = (o % oe) * (-1 if oe == 1.0 else 1)
                    if t.get_angle(to) < 1.0:
                        return RUN_ON * size
        return 0.0

    def side_normal(e, p):
        for f in faces_of[hash(e)]:
            n = f.normal_at(p)
            if abs(n.Z) < 0.5:
                return Vector(n.X, n.Y, 0).normalized()
        return None

    def up(e, p):
        for f in faces_of[hash(e)]:
            n = f.normal_at(p)
            if abs(n.Z) > 0.5:
                return 1.0 if n.Z > 0 else -1.0
        return None

    def triangle(plane, zv, origin):
        """The section a chamfer (or round) removes, in the local (outward,
        up) plane at the edge, a little oversize past the faces."""
        lu = plane.to_local_coords(origin + zv).Y
        if not rounded:
            return Polygon((eps, eps * lu), (-size - eps, eps * lu), (eps, -(size + eps) * lu), align=None)
        r = size
        return make_face([
            Line((eps, eps * lu), (-r, eps * lu)),
            Line((-r, eps * lu), (-r, 0)),
            CenterArc((-r, -r * lu), r, 90 if lu > 0 else -90, -90 if lu > 0 else 90),
            Line((0, -r * lu), (eps, -r * lu)),
            Line((eps, -r * lu), (eps, eps * lu)),
        ])

    def wedge(e, ext0=0.0, ext1=0.0):
        """The edge's wedge, run on `ext0` before its start and `ext1` past
        its end (as lengths; a round's by the matching angle)."""
        p = e @ 0.5
        n, z = side_normal(e, p), up(e, p)
        if n is None or z is None:
            return None
        zv = Vector(0, 0, z)
        if e.geom_type.name == "LINE":
            a, b = e @ 0, e @ 1
            t = (b - a).normalized()
            start = a - t * ext0
            plane = Plane(origin=start, x_dir=n, z_dir=t)
            return extrude(plane * triangle(plane, zv, start), (b - a).length + ext0 + ext1, dir=t)
        if e.geom_type.name != "CIRCLE":
            return None
        c, r = e.arc_center, e.radius
        radial = Vector(p.X - c.X, p.Y - c.Y, 0).normalized()
        inside = n.dot(radial) > 0  # the material lies inside the circle
        a0, am, a1 = (math.degrees(math.atan2((e @ t_).Y - c.Y, (e @ t_).X - c.X)) for t_ in (0.0, 0.5, 1.0))
        sweep = (a1 - a0) % 360
        if not 0 < (am - a0) % 360 < sweep:
            sweep -= 360
        if abs(sweep) < 1e-6:
            sweep = 360.0
        axis = Axis((c.X, c.Y, c.Z), (0, 0, 1))
        if inside and r <= size:
            # A round smaller than the chamfer: over its sector the chamfer
            # leaves a 45° cone, radius r at the chamfer's foot and a point r
            # above (or below) it; the cut is the sector less that cone.
            sr = Vector(math.cos(math.radians(a0)), math.sin(math.radians(a0)), 0)
            plane = Plane(origin=(c.X, c.Y, p.Z), x_dir=sr, z_dir=Vector(-sr.Y, sr.X, 0))
            lu = plane.to_local_coords(Vector(c.X, c.Y, p.Z) + zv).Y
            far = size + 4
            profile = Polygon((0, (r - size) * lu), (r + eps, (-size - eps) * lu), (far, (-size - eps) * lu),
                              (far, eps * lu), (0, eps * lu), align=None)
            return revolve(plane * profile, axis, sweep)
        if abs(sweep) < 360:
            # Run 0.02 mm on past both ends as well, so that no end face lies
            # on a neighbouring face (as where an arc ends tangent to another).
            sign = 1 if sweep > 0 else -1
            pad0 = math.degrees((ext0 + 0.02) / r) * sign
            pad1 = math.degrees((ext1 + 0.02) / r) * sign
            a0 -= pad0
            sweep += pad0 + pad1
        sr = Vector(math.cos(math.radians(a0)), math.sin(math.radians(a0)), 0)
        start = Vector(c.X + r * sr.X, c.Y + r * sr.Y, p.Z)
        plane = Plane(origin=start, x_dir=sr if inside else -sr, z_dir=Vector(-sr.Y, sr.X, 0))
        return revolve(plane * triangle(plane, zv, start), axis, sweep)

    tools = [w for w in (wedge(e, run_on(e, 0.0), run_on(e, 1.0)) for e in edges) if w is not None]
    # Re-entrant corners between two chamfered edges.
    done = set()
    for i, e in enumerate(edges):
        for end in (0.0, 1.0):
            v = e @ end
            t = (e % end) * (1 if end == 1.0 else -1)  # leaving e past that end
            n = side_normal(e, v)
            for j, o in enumerate(edges):
                if j == i or (min(i, j), max(i, j), round(v.X, 6), round(v.Y, 6)) in done:
                    continue
                for oe in (0.0, 1.0):
                    if ((o @ oe) - v).length > 1e-6:
                        continue
                    to = (o % oe) * (-1 if oe == 1.0 else 1)  # leaving the corner along o
                    angle = t.get_angle(to)
                    if angle < 1.0 or abs(180 - angle) < 1.0 or n is None or to.dot(n) <= 0:
                        continue  # straight on, tangent, or a convex corner
                    done.add((min(i, j), max(i, j), round(v.X, 6), round(v.Y, 6)))
                    run = size / math.tan(math.radians((-t).get_angle(to)) / 2) + size
                    we = wedge(e, *((run, 0.0) if end == 0.0 else (0.0, run)))
                    wo = wedge(o, *((run, 0.0) if oe == 0.0 else (0.0, run)))
                    if we is not None and wo is not None:
                        piece = we & wo
                        if piece.volume > 1e-9:
                            tools.append(piece)
    return tools
