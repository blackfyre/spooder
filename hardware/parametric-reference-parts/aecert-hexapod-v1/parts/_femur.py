"""The femur side plates' shared plan: Femur Horn Side and Femur Bearing Side
are one plate, 8.35 mm thick, its outline a 30 mm wide stadium whose ends are
centred on the two bore axes, with two lightening slots between the bores, a
countersunk middle hole and a recess over the web between the slots. They
differ in their bores (each part file's own) and in the hand of the slots'
45° sides: Femur Horn Side's run back (x falls as y rises), Femur Bearing
Side's forward.

Everything is placed from the first bore's axis (each part file's own origin)
and AXIS_TO_AXIS, read when a plate is built. Dimensions are measured from
`Femur Horn Side.stl` and `Femur Bearing Side.stl` with trimesh (horizontal
sections at z 0.05 to 8.34, line and circle fits to the section loops); the
two STLs agree on every value here to 0.001 mm once placed from their first
bore.
"""

import math

from build123d import (Align, Axis, Bezier, Circle, Cone, Cylinder, Line, Polygon, Pos, SlotCenterToCenter, chamfer,
                       extrude, fillet, make_face)

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The femur's axis-to-axis length: the distance between the two bore axes of
# either side plate, from which both sides place their second bore. Source:
# circle fits to the bores in sections every 0.5 mm, 454.581 - 344.581 on
# Femur Horn Side and 453.226 - 343.226 on Femur Bearing Side, the same on
# every section. `hardware/aecert-hexapod-v1/README.md` gives the femur as
# 108 mm ("Measured on Spooder"); the STLs, which the gates hold these
# solids to, place the axes 110.000 mm apart, 2 mm more (design: "The
# axis-to-axis length"). This is the printed parts' bore spacing, not the
# movement's femur link length (FEMUR_LENGTH, 116 mm).
AXIS_TO_AXIS = 110.0

# The plate: z 0 to 8.35 (the downward and upward faces), its outline a
# stadium of r 15 round each bore axis (sections at z 1.2 and 6: arcs r
# 15.000 centred on the bores, sides at y ±15).
THICKNESS = 8.35
OUTLINE_RADIUS = 15.0
# 45° chamfers round the outline and the slots, top and bottom: the outline
# is 0.95 mm in at z 0.05 and 8.3 (arcs r 14.049) and 0.5 mm in at z 0.5.
# The slots' 45° sides are chamfered at the bottom only: at the top they
# stand 1 mm from the recess, and their sections at z 7.8 and 8.3 are those
# at z 1.2.
EDGE_CHAMFER = 1.0

# The lightening slots, 16 mm wide on the plates' centre line (sections at
# z 1.2 and 6: y ±8.000 off the bores' axis line). Each one's end next to its
# bore is concave, an arc of r 15 round that bore (the vertices fit it to
# 0.006 mm), meeting the slot's long sides in r 2 rounds; the other end is a
# 45° side, crossing the centre line WEB_HALF_WIDTH either side of the middle
# hole (Femur Horn Side: 401.667 and 414.495, about 408.081).
SLOT_HALF_WIDTH = 8.0
SLOT_END_RADIUS = 15.0
SLOT_CORNER_RADIUS = 2.0
WEB_HALF_WIDTH = 6.414

# The middle hole on the centre line, 63.5 mm from the first bore (circle
# fits: 408.081 - 344.581 and 406.726 - 343.226): r 1.6 from the recess
# down, countersunk at 45° from r 2.75 at the bottom face to r 1.6 at
# z 1.15 (r 2.6997 at z 0.05, 2.1479 at z 0.6, 1.5996 at z 1.18).
MIDDLE_HOLE_OFFSET = 63.5
HOLE_RADIUS = 1.6
COUNTERSINK_RADIUS = 2.75
COUNTERSINK_DEPTH = 1.15

# The recess over the web, from its floor (each part file's) to the top: the
# part of a 26.612 mm square round the middle hole (its sides at x and y
# ±13.306) that lies within a band 5 mm either side of the middle hole along
# x, the band's sides parallel to the slots' 45° sides (the web's walls 1 mm
# thick). The square's two corners inside the band are rounded by parabolas
# set back 5 mm along each side, each a quadratic Bézier with its control
# point at the corner (Femur Horn Side, section at z 6: the band's sides
# from (394.775, 180.133) to (416.387, 158.521) and from (421.387, 163.521)
# to (399.775, 185.133); the round's vertices fit the parabola to 0.005 mm,
# its midpoint (420.132, 159.774) against the Bézier's (420.137, 159.771),
# where a circle fits them only to 0.07 mm).
RECESS_HALF_SIZE = 13.306
RECESS_HALF_WIDTH_X = 5.0
RECESS_ROUND = 5.0


def second_bore_x(first_x):
    """The second bore's axis, AXIS_TO_AXIS along x from the first."""
    return first_x + AXIS_TO_AXIS


def bore_centres(first_x, y):
    return [(first_x, y), (second_bore_x(first_x), y)]


def countersunk_hole(x, y, top):
    """The middle hole and the bores' centre holes: r HOLE_RADIUS up to `top`,
    countersunk from the bottom face."""
    return (Pos(x, y, 0) * Cylinder(HOLE_RADIUS, top, align=BOTTOM)
            + Pos(x, y, 0) * Cone(COUNTERSINK_RADIUS, COUNTERSINK_RADIUS - COUNTERSINK_DEPTH, COUNTERSINK_DEPTH,
                                  align=BOTTOM))


def slot(bore_x, y, diagonal_x, hand):
    """One lightening slot's plan: from the r SLOT_END_RADIUS arc round the
    bore at `bore_x` to the 45° side that crosses the centre line at
    `diagonal_x`, its x changing by `hand` per mm of y."""
    h = SLOT_HALF_WIDTH
    outline = Polygon((bore_x, y - h), (diagonal_x - hand * h, y - h), (diagonal_x + hand * h, y + h), (bore_x, y + h),
                      align=None)
    face = outline - Pos(bore_x, y) * Circle(SLOT_END_RADIUS)
    corners = [v for v in face.vertices() if abs(abs(v.Y - y) - h) < 1e-6
               and abs(math.hypot(v.X - bore_x, v.Y - y) - SLOT_END_RADIUS) < 1e-6]
    return fillet(corners, SLOT_CORNER_RADIUS)


def recess(x, y, hand):
    """The recess's plan round the middle hole at (x, y), the band along the
    slots' 45° sides (`hand` as in `plate`)."""
    h, w, r = RECESS_HALF_SIZE, RECESS_HALF_WIDTH_X, RECESS_ROUND
    s = -hand  # +1 where the band runs from upper left to lower right

    def p(u, v):
        return (x + u, y + s * v)

    # One end of the recess, (u, v) relative to the middle hole: from the
    # band's side along the square's side to the round, the round, and the
    # square's other side back to the band's other side. The other end is
    # its point reflection, and the band's sides join the two.
    end = [(h - w, -h), (h - r, -h), (h, -h), (h, -(h - r)), (h, -(h - w))]
    edges = []
    for k in (1, -1):
        b, c, d, e, f = [(k * u, k * v) for u, v in end]
        if b != c:
            edges.append(Line(p(*b), p(*c)))
        edges.append(Bezier(p(*c), p(*d), p(*e)))
        if e != f:
            edges.append(Line(p(*e), p(*f)))
        edges.append(Line(p(*f), p(-k * end[0][0], -k * end[0][1])))
    return make_face(edges)


def plate(first_x, y, hand, recess_floor_z):
    """The plate with its outline, slots, chamfers, middle hole and recess,
    the bores not yet cut. `hand` is the slots' 45° sides' change in x per mm
    of y: -1 on Femur Horn Side, +1 on Femur Bearing Side."""
    second_x = second_bore_x(first_x)
    middle_x = first_x + MIDDLE_HOLE_OFFSET
    body = Pos((first_x + second_x) / 2, y, 0) * extrude(SlotCenterToCenter(AXIS_TO_AXIS, 2 * OUTLINE_RADIUS),
                                                          THICKNESS)
    for bore_x, diagonal_x in ((first_x, middle_x - WEB_HALF_WIDTH), (second_x, middle_x + WEB_HALF_WIDTH)):
        body -= extrude(slot(bore_x, y, diagonal_x, hand), THICKNESS, dir=(0, 0, 1))

    def diagonal(e):
        d = e.tangent_at(0.5) if e.geom_type.name == "LINE" else None
        return d is not None and abs(abs(d.X) - abs(d.Y)) < 1e-6 and abs(d.X) > 0.5

    bottom = body.edges().group_by(Axis.Z)[0]
    top = [e for e in body.edges().group_by(Axis.Z)[-1] if not diagonal(e)]
    body = chamfer(bottom + top, EDGE_CHAMFER)

    body -= countersunk_hole(middle_x, y, THICKNESS)
    body -= Pos(0, 0, recess_floor_z) * extrude(recess(middle_x, y, hand), THICKNESS - recess_floor_z, dir=(0, 0, 1))
    return body
