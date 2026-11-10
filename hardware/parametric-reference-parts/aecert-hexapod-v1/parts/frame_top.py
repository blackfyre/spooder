"""Frame Top: the frame's upper plate.

A 4 mm floor (z 16 to 20) under a rim that rises to z 25.75 round the
outline; inside the rim the floor is open in six rounded triangles between
12 mm spokes and a central hole, and channels run from the open middle into
each arm's coxa servo slot. The outline, the arms, the slots and the mount
holes are the frame's shared plan (`_frame.py`); every edge of the outline,
the rim and the floor's openings and the slots has a 1 mm chamfer.
Dimensions are measured from `Frame Top.stl` with trimesh: horizontal
sections at z 16.1 to 25.65 (line and circle fits to the section loops),
rays across the slots, and the levels of its horizontal faces. Coordinates
are the STL's (the body frame).
"""

from build123d import Circle, Cylinder, Plane, Pos, Rectangle, Vector, chamfer, extrude, fillet

import _frame

# Levels. Source: the downward face at z 16, the upward faces at z 20 (the
# floor inside the rim) and z 25.75 (the rim's top).
BOTTOM_Z = 16.0
FLOOR_Z = 20.0
TOP_Z = 25.75
# 45° chamfers: the cutouts' edges are 0.9 mm out at z 16.1 and 19.9.
EDGE_CHAMFER = 1.0

# The spokes between the floor's openings: 12 mm wide along the arms' axes;
# the openings' corners rounded r 6 (sections at z 18).
SPOKE_HALF_WIDTH = 6.0
OPENING_CORNER_RADIUS = 6.0
# The channels from the open middle into the slots, as wide as the spokes
# (section at z 23: y ±6 from x 51.1 to the slot at 67.6).
CHANNEL_HALF_WIDTH = 6.0
CHANNEL_START = 40.0  # inside the rim's inner wall on every arm
# The central hole (section at z 18).
CENTRE_HOLE_RADIUS = 8.5

# Screw holes, radius 1.6, each with a 45° countersink to radius 2.75 at its
# upper face (sections at z 19.5 to 25.5): through the rim at the mount
# holes and at the frame screws between the arms; through the floor at the
# other frame screws (`_frame.frame_screws`).
SCREW_RADIUS = 1.6
COUNTERSINK_RADIUS = 2.75
# Plain holes, radius 2.2: through the floor on the corner spokes 23.0 mm out,
# and blind from the rim's top at (±20, ±72.994) to the upward face at
# z 19.75.
INSERT_RADIUS = 2.2
FLOOR_INSERT_R = 23.0
RIM_INSERTS = [(20.0, 72.994)]
RIM_INSERT_BOTTOM_Z = 19.75


def openings(inner):
    """The floor's six rounded triangles between the spokes, and the
    central hole."""
    c = _frame.CORNER_ANGLE
    s = SPOKE_HALF_WIDTH
    u = _frame.polar(1, c)
    v = (-u[1], u[0])
    # Between the middle arm's spoke and the corner arm's: above y = s, below
    # the corner spoke's lower side.
    side = _frame.inner_quadrant() & _frame.halfplane((0, s), 0) & _frame.halfplane((-s * v[0], -s * v[1]), c + 180)
    side = fillet(_frame.sharp_corners(side), OPENING_CORNER_RADIUS)
    # Between the two front corner arms' spokes: above both.
    above = _frame.halfplane((s * v[0], s * v[1]), c)
    front = inner & above & above.mirror(Plane.YZ)
    front = fillet(_frame.sharp_corners(front), OPENING_CORNER_RADIUS)
    shapes = _frame.mirrored4(side) + [front, front.mirror(Plane.XZ), Circle(CENTRE_HOLE_RADIUS)]
    return _frame.union(shapes)


def channels():
    length = _frame.SLOT_START - CHANNEL_START
    channel = Pos(CHANNEL_START + length / 2, 0) * Rectangle(length, 2 * CHANNEL_HALF_WIDTH)
    return [_frame.place(channel, a, off) for a, off in _frame.arm_places()]


def at_z(edges, z):
    return [e for e in edges if abs(e.center().Z - z) < 1e-6 and abs(e.bounding_box().size.Z) < 1e-6]


def build():
    inner = _frame.union(_frame.mirrored4(_frame.inner_quadrant()))
    pocket = _frame.union([inner] + channels())
    body = Pos(0, 0, BOTTOM_Z) * extrude(_frame.outline(), TOP_Z - BOTTOM_Z)
    body -= Pos(0, 0, FLOOR_Z) * extrude(pocket, TOP_Z - FLOOR_Z)
    for slot in _frame.slots():
        body -= Pos(0, 0, BOTTOM_Z) * extrude(slot, TOP_Z - BOTTOM_Z)
    body = chamfer(at_z(body.edges(), BOTTOM_Z) + at_z(body.edges(), TOP_Z), EDGE_CHAMFER)

    # The floor's openings, chamfered underneath all round and on the floor's
    # top where they open from it; where an opening runs on up into the rim's
    # wall, its top chamfer stops against the wall.
    holes = openings(inner)
    body -= Pos(0, 0, BOTTOM_Z) * extrude(holes, FLOOR_Z - BOTTOM_Z)
    body -= _frame.chamfer_tool(holes, EDGE_CHAMFER, BOTTOM_Z, up=True)
    body -= _frame.chamfer_tool(holes, EDGE_CHAMFER, FLOOR_Z, up=False) & (
        Pos(0, 0, BOTTOM_Z) * extrude(inner, TOP_Z - BOTTOM_Z))

    def screw(x, y, top):
        nonlocal body
        body -= Pos(x, y, BOTTOM_Z) * Cylinder(SCREW_RADIUS, top - BOTTOM_Z, align=_frame.BOTTOM)
        rim = [e for e in at_z(body.edges(), top) if e.geom_type.name == "CIRCLE"
               and abs(e.radius - SCREW_RADIUS) < 1e-6 and (e.arc_center - Vector(x, y, top)).length < 1e-6]
        body = chamfer(rim, COUNTERSINK_RADIUS - SCREW_RADIUS)

    between = set(_frame.mirror_points(_frame.SCREW_BETWEEN))
    for x, y in _frame.mount_holes():
        screw(x, y, TOP_Z)
    for x, y in _frame.frame_screws():
        screw(x, y, TOP_Z if (x, y) in between else FLOOR_Z)
    for x, y in _frame.mirror_points(_frame.polar(FLOOR_INSERT_R, _frame.CORNER_ANGLE)):
        body -= Pos(x, y, BOTTOM_Z) * Cylinder(INSERT_RADIUS, FLOOR_Z - BOTTOM_Z, align=_frame.BOTTOM)
    for x, y in [p for q in RIM_INSERTS for p in _frame.mirror_points(q)]:
        body -= Pos(x, y, RIM_INSERT_BOTTOM_Z) * Cylinder(INSERT_RADIUS, TOP_Z - RIM_INSERT_BOTTOM_Z,
                                                          align=_frame.BOTTOM)
    return body


PARTS = [
    {
        "name": "Frame Top",
        "stl": "hardware/aecert-hexapod-v1/stl/Frame Top.stl",
        "build": build,
    },
]
