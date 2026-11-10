"""Frame Bottom: the frame's lower half, holding the battery and the coxa
servos.

A 2 mm floor (z -15.75 to -13.75) with the arms rising to z 1.0, and walls
from z 1.0 to 16.0 round the open middle, on which Frame Top sits. In each
arm the coxa servo's slot runs out to the arm's end, and above the arm's top
a pocket takes its Servo Mount's ring; the battery lies in a channel down
the middle, closed at the front and open at the rear. Between each middle
and corner arm two hexagonal tunnels run through the walls. The outline, the
arms, the slots, the mount holes and the open middle are the frame's shared
plan (`_frame.py`). Every convex horizontal edge has a 1 mm chamfer, except
where the STL has none (see `build`).
Dimensions are measured from `Frame Bottom.stl` with trimesh: horizontal
sections at z -15.2 to 15.5, vertical sections across the tunnels (line and
circle fits to the section loops), and the levels of its horizontal faces.
Coordinates are the STL's (the body frame).
"""

import math

from build123d import (CenterArc, Cone, Cylinder, Line, Plane, Polygon, Pos, Rectangle, RectangleRounded, Rot,
                       Vector, extrude, fillet, make_face)

import _frame

# Levels. Source: the downward face at z -15.75 and the upward faces at
# -13.75 (the floor), -13.0 (the floor behind the battery channel), -5.75
# (the blocks either side of the channel), 1.0 (the arms) and 16.0 (the
# walls).
BOTTOM_Z = -15.75
FLOOR_Z = -13.75
REAR_FLOOR_Z = -13.0
BLOCK_Z = -5.75
ARM_TOP_Z = 1.0
TOP_Z = 16.0
EDGE_CHAMFER = 1.0

# The hole in the floor under the battery (section at z -14.5): between
# x ±19.5, closed front and rear by the open middle's arcs.
FLOOR_HOLE_HALF_WIDTH = 19.5
# The battery channel (sections at z -13.4 to 8; README): x ±24.65, its
# front end at y 75.65 and, down to the rear floor's level, its rear end at
# y -75.65, the corners rounded r 2. It meets the open middle's arc C in an
# r 2 round.
CHANNEL_HALF_WIDTH = 24.65
CHANNEL_END_Y = 75.65
CHANNEL_CORNER_RADIUS = 2.0
# The pockets beside the channel, through the floor up to the blocks' tops
# (section at z -14.5): the open middle beyond x 29.5 and y 5, the corner at
# (29.5, 5) rounded r 6, the corner on arc C r 2.
SIDE_POCKET_X = 29.5
SIDE_POCKET_Y = 5.0
SIDE_POCKET_CORNER_RADIUS = 6.0
SIDE_POCKET_ARC_RADIUS = 2.0
# Behind the channel, above the rear floor, the frame is open between the
# walls (section at z -9): from x ±24.65 at y -67.99 an r 20 arc (its centre
# at x 44.65), a straight tangent and a second arc, its centre the circle
# fitted to its vertices (to 0.024 mm, radius 20.51), its radius (20.49) the
# one that runs it tangent into the outline's rear r 29 arc.
REAR_OPEN_Y = -67.99
REAR_ARC_1_RADIUS = 20.0
REAR_ARC_2_CENTRE = (47.32, -72.2)

# The pockets for the Servo Mounts' rings above the arms (section at z 8):
# the ring's 57 × 37 mm with r 7 corners, 0.15 mm larger all round, centred
# on the mount holes; where a pocket meets the outline the wall ends in an
# r 0.7 round.
POCKET_LENGTH = 57.3
POCKET_WIDTH = 37.3
POCKET_CORNER_RADIUS = 7.15
WALL_TIP_RADIUS = 0.7
# The coxa servo slot in Frame Bottom starts at the pocket's inner end
# (README: 57.15 mm to the arm's end).
SLOT_START = _frame.MOUNT_HOLE_X - POCKET_LENGTH / 2

# The two hexagonal tunnels through the walls between each middle and corner
# arm, square to the outline's straight piece there and 16 mm apart, their
# axes 9.5 and 6.5 mm either side of its middle (vertical sections at 17.18°
# through (60, 37)): 11 mm wide, vertical sides from z -8.575 to 3.075,
# closing at 30° to points at z -11.75 and 6.25.
TUNNEL_OFFSETS = (-9.5, 6.5)
TUNNEL_HALF_WIDTH = 5.5
TUNNEL_SIDE_BOTTOM_Z = -8.575
TUNNEL_SIDE_TOP_Z = 3.075
TUNNEL_SLOPE = 30.0  # degrees from horizontal

# Holes: the mount holes, radius 1.6 through the arms with a 45° countersink
# to radius 2.75 underneath (sections at z -15.2 and -14.5); the frame
# screws' insert holes, radius 2.2, from the walls' top down to the upward
# face at z 6 (section at z 8).
SCREW_RADIUS = 1.6
COUNTERSINK_RADIUS = 2.75
INSERT_RADIUS = 2.2
INSERT_BOTTOM_Z = 6.0


def slab(face, z0, z1):
    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def channel(rear_y):
    length = CHANNEL_END_Y - rear_y
    shape = Pos(0, (CHANNEL_END_Y + rear_y) / 2) * Rectangle(2 * CHANNEL_HALF_WIDTH, length)
    return fillet([v for v in shape.vertices()], CHANNEL_CORNER_RADIUS)


def floor_hole(inner):
    return inner & Rectangle(2 * FLOOR_HOLE_HALF_WIDTH, 400)


def side_pockets():
    q = _frame.inner_quadrant() & Pos(SIDE_POCKET_X + 200, SIDE_POCKET_Y + 200) * Rectangle(400, 400)
    corner = [v for v in q.vertices() if abs(v.X - SIDE_POCKET_X) < 1e-6 and abs(v.Y - SIDE_POCKET_Y) < 1e-6]
    q = fillet(corner, SIDE_POCKET_CORNER_RADIUS)
    arc = [v for v in q.vertices() if abs(v.X - SIDE_POCKET_X) < 1e-6 and v.Y > 20]
    q = fillet(arc, SIDE_POCKET_ARC_RADIUS)
    return _frame.union(_frame.mirrored4(q))


def open_middle(inner):
    """The open middle and the battery channel, from the blocks' tops up,
    rounded where the channel's sides meet arc C."""
    void = inner + channel(-CHANNEL_END_Y)
    corners = [v for v in void.vertices() if abs(abs(v.X) - CHANNEL_HALF_WIDTH) < 1e-6 and 30 < abs(v.Y) < 70]
    return fillet(corners, CHANNEL_CORNER_RADIUS)


def rear_opening():
    """The open rear between the walls, above the rear floor, in the +x half
    and mirrored."""
    x0, y0 = CHANNEL_HALF_WIDTH, REAR_OPEN_Y
    c1 = (x0 + REAR_ARC_1_RADIUS, y0)
    c2 = REAR_ARC_2_CENTRE
    fx, fy = _frame.front_arc_centre()
    c29 = (fx, -fy)
    r1 = REAR_ARC_1_RADIUS
    r2 = math.hypot(c29[0] - c2[0], c29[1] - c2[1]) - _frame.FRONT_ARC_RADIUS  # tangent to the r 29 arc
    # The tangent between the two arcs, both outside the opening.
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    phi = math.atan2(dy, dx) - math.acos((r1 - r2) / math.hypot(dx, dy))
    n = (math.cos(phi), math.sin(phi))
    t1 = (c1[0] + r1 * n[0], c1[1] + r1 * n[1])
    t2 = (c2[0] + r2 * n[0], c2[1] + r2 * n[1])
    a1 = math.degrees(phi)
    a2 = math.degrees(math.atan2(c29[1] - c2[1], c29[0] - c2[0]))
    far = -150.0
    edges = [
        Line((0, y0), (x0, y0)),
        CenterArc(c1, r1, 180, (a1 - 180) % 360),
        Line(t1, t2),
        CenterArc(c2, r2, a1, (a2 - a1) % 360),
    ]
    tip = edges[-1] @ 1
    edges += [Line(tip, c29), Line(c29, (c29[0], far)), Line((c29[0], far), (0, far)), Line((0, far), (0, y0))]
    half = make_face(edges)
    return half + half.mirror(Plane.YZ)


def tunnel_section():
    """A tunnel's section in the local (across, z) plane."""
    w = TUNNEL_HALF_WIDTH
    rise = w * math.tan(math.radians(TUNNEL_SLOPE))
    pts = [(w, TUNNEL_SIDE_BOTTOM_Z), (w, TUNNEL_SIDE_TOP_Z), (0, TUNNEL_SIDE_TOP_Z + rise),
           (-w, TUNNEL_SIDE_TOP_Z), (-w, TUNNEL_SIDE_BOTTOM_Z), (0, TUNNEL_SIDE_BOTTOM_Z - rise)]
    return Polygon(*pts, align=None)


def tunnels():
    t1, t2 = _frame.side_tangent()
    mid = ((t1[0] + t2[0]) / 2, (t1[1] + t2[1]) / 2)
    along = math.atan2(t2[1] - t1[1], t2[0] - t1[0])
    ux, uy = math.cos(along), math.sin(along)
    normal = (uy, -ux)  # out of the frame
    out = []
    for s in TUNNEL_OFFSETS:
        origin = (mid[0] + s * ux, mid[1] + s * uy, 0)
        plane = Plane(origin=origin, x_dir=(ux, uy, 0), z_dir=(normal[0], normal[1], 0))
        out.append(extrude(plane * tunnel_section(), 40, both=True))
    return [m for t in out for m in _frame.mirrored4(t)]


def servo_pockets():
    pocket = Pos(_frame.MOUNT_HOLE_X, 0) * RectangleRounded(POCKET_LENGTH, POCKET_WIDTH, POCKET_CORNER_RADIUS)
    return [_frame.place(pocket, a, off) for a, off in _frame.arm_places()]


def slots():
    length = _frame.MIDDLE_END - SLOT_START + 1.0
    slot = Pos(SLOT_START + length / 2, 0) * Rectangle(length, 2 * _frame.SLOT_HALF_WIDTH)
    return [_frame.place(slot, a, off) for a, off in _frame.arm_places()]


def walls_plan(outline, middle):
    """The walls' plan: the outline less the servo pockets, the open middle
    and the open rear, with sharp tips where a pocket meets the outline."""
    return outline - _frame.union(servo_pockets()) - middle - rear_opening()


def tips(plan):
    """The walls' tips: the corners of their plan sharper than a right
    angle."""
    out = []
    for v in plan.vertices():
        tangents = []
        for e in plan.edges():
            for t in (0.0, 1.0):
                if ((e @ t) - v.center()).length < 1e-6:
                    d = e % t
                    tangents.append(d if t == 0.0 else -d)
        if len(tangents) == 2 and tangents[0].get_angle(tangents[1]) < 90:
            out.append(v)
    return out


def build():
    outline = _frame.outline()
    inner = _frame.union(_frame.mirrored4(_frame.inner_quadrant()))
    middle = open_middle(inner)
    slot_plan = _frame.union(slots())
    pockets = side_pockets()
    channel_plan = channel(-CHANNEL_END_Y)
    rear = rear_opening()

    # Layer by layer, each standing on the one below.
    body = slab(outline - slot_plan - floor_hole(inner) - pockets, BOTTOM_Z, FLOOR_Z)
    body += slab(outline - slot_plan - channel_plan - pockets, FLOOR_Z, REAR_FLOOR_Z)
    body += slab(outline - slot_plan - channel_plan - pockets - rear, REAR_FLOOR_Z, BLOCK_Z)
    body += slab(outline - slot_plan - middle - rear, BLOCK_Z, ARM_TOP_Z)
    plan = walls_plan(outline, middle)
    plan = fillet(tips(plan), WALL_TIP_RADIUS)
    body += slab(plan, ARM_TOP_Z, TOP_Z)
    body = body.clean()

    # Every convex horizontal edge is chamfered 1 mm, except (as the STL has
    # them) the walls' top edges facing the open middle and the open rear,
    # the floor hole's top edges, the corner arms' slot ends and the rear
    # floor's edge round the outline; the rear floor's 0.75 mm step is
    # rounded over its full height. A chamfer on a face stops at the walls
    # standing on it (`clip`: the open space above the face).
    def on(sketch, e):
        c = e.center()
        return min(x.distance_to(Vector(c.X, c.Y, 0)) for x in sketch.edges()) < 1e-4

    corner_slot_ends = []
    for a, off in _frame.arm_places():
        if off:
            p = (Rot(0, 0, a) * Pos(SLOT_START + off, 0, 0)).position
            corner_slot_ends.append(Vector(p.X, p.Y, 0))
    open_top = middle + rear
    big = Pos(0, 0, -50) * extrude(Rectangle(400, 400), 100)
    above = {REAR_FLOOR_Z: big - slab(outline - slot_plan - channel_plan - pockets - rear, -50, 50),
             BLOCK_Z: big - slab(outline - slot_plan - middle - rear, -50, 50),
             ARM_TOP_Z: big - slab(plan, -50, 50)}
    for z in (BOTTOM_Z, REAR_FLOOR_Z, BLOCK_Z, ARM_TOP_Z, TOP_Z):
        edges = [e for e in _frame.convex_horizontal_edges(body) if abs(e.center().Z - z) < 1e-6]
        size = EDGE_CHAMFER
        if z == REAR_FLOOR_Z:
            # Only the rear floor's step at the channel's end, rounded over
            # its full 0.75 mm; its edge round the outline is sharp.
            edges = [e for e in edges if not on(outline, e)]
            size = REAR_FLOOR_Z - FLOOR_Z
        if z in (BOTTOM_Z, ARM_TOP_Z):
            edges = [e for e in edges if not any((Vector(e.center().X, e.center().Y, 0) - p).length < 1e-3
                                                 for p in corner_slot_ends)]
        if z == TOP_Z:
            # Not the walls' tips: the chamfers either side run through them.
            edges = [e for e in edges if not on(open_top, e)
                     and not (e.geom_type.name == "CIRCLE" and e.radius <= EDGE_CHAMFER)]
        body = _frame.chamfered(body, size, edges, clip=above.get(z), rounded=z == REAR_FLOOR_Z)

    for t in tunnels():
        body -= t
    for x, y in _frame.mount_holes():
        body -= Pos(x, y, BOTTOM_Z) * Cylinder(SCREW_RADIUS, ARM_TOP_Z - BOTTOM_Z, align=_frame.BOTTOM)
        body -= Pos(x, y, BOTTOM_Z) * Cone(COUNTERSINK_RADIUS, SCREW_RADIUS, COUNTERSINK_RADIUS - SCREW_RADIUS,
                                         align=_frame.BOTTOM)
    for x, y in _frame.frame_screws():
        body -= Pos(x, y, INSERT_BOTTOM_Z) * Cylinder(INSERT_RADIUS, TOP_Z - INSERT_BOTTOM_Z, align=_frame.BOTTOM)
    return body


PARTS = [
    {
        "name": "Frame Bottom",
        "stl": "hardware/aecert-hexapod-v1/stl/Frame Bottom.stl",
        "build": build,
    },
]
