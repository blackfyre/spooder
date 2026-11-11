"""The coxa group's shared shapes: the side plate both Coxa Horn Side and
Coxa Bearing Side are cut from, and the ring and tongue both Coxa Horn
Connector and Coxa Bearing Connector are built on.

Each side is an 8.3 mm plate lying flat in its STL (z 0 to 8.3), 68.25 mm
long (x) and 48 mm wide (y), placed by the middle of its 28.5 mm hole pair:
the pair runs along y, the plate reaches 34.125 mm either side of the middle
in x, 18.5 mm below it in y and 25.5 mm above it to the ears' centres. The two
ears, 30.10 mm either side of the middle in x, are half discs (r 4.025) with a
screw hole each; between them the edge is cut back to 18.5 mm above the middle
with r 7 corners, and the far corners are rounded r 15. Every edge of the
outline and of the window has a 1 mm 45° chamfer at both faces; the holes are
straight through (r 1.6), some with a 45° countersink at the upper face.
Dimensions are measured from `Coxa Horn Side.stl` and `Coxa Bearing Side.stl`
with trimesh: horizontal sections at z 0.05 to 8.25 (line and circle fits to
the section loops); the two STLs agree on every value here to 0.001 mm.

Each connector is an 8.05 mm ring round the coxa axis (the horn's or the
bearing's seat is cut in it by the part's own file), 31.6 mm wide, its far
end rounded r 15, and a 15 mm tongue running on from it, which carries the
insert hole (r 2.2, across the tongue, 21.5 mm from the axis at the ring's
mid-height) that the side plates' ears screw to. Either side of the tongue
the ring is cut back round the insert hole's axis (r 4.175) to clear the
ears. Beyond the hole the tongue rises in a thin fin between two r 7 coves
and ends in a 45° slope. It is modelled once, in a frame with the axis at the
origin, the ring's underside at z 0 and the tongue along +x, and each
connector places it. Dimensions are measured from `Coxa Horn Connector.stl`
and `Coxa Bearing Connector.stl` with trimesh: horizontal sections at z 0.05
to 8.0 above the underside and vertical sections across and along the tongue
(line and circle fits to the section loops); the two STLs agree on every
value here to 0.003 mm, the Bearing Connector's turned 180° about its axis.
What differs (the plate's or the connector's place, the window, which holes
are countersunk, the seat in the ring) is in each part's own file.
"""

from build123d import (Align, Axis, CenterArc, Circle, Cylinder, Line, Plane, Polygon, Pos, Rectangle, ThreePointArc,
                       chamfer, extrude, fillet, make_face, revolve)

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# Source: the downward face at z 0 and the upward face at z 8.3.
THICKNESS = 8.3
# The outline in the section at z 2, from the hole pair's middle: x ±34.125,
# y -18.5 to the ears' centres at +25.5.
HALF_LENGTH = 34.125
BELOW_MIDDLE = 18.5
EAR_ABOVE_MIDDLE = 25.5
# The far corners: arcs r 15.000 (centres 19.125 in from the ends).
FAR_CORNER_RADIUS = 15.0
# The ears: half discs r 4.025 round their holes, 30.10 mm either side of the
# middle (circle fits, section at z 2: centres 60.200 mm apart).
EAR_OFFSET = 30.10
EAR_RADIUS = 4.025
# Between the ears the edge lies 18.5 mm above the middle, its corners arcs
# r 7.000 meeting the ears' inner sides tangentially.
NOTCH_EDGE_ABOVE_MIDDLE = 18.5
NOTCH_CORNER_RADIUS = 7.0
# 45° chamfers: the outline and the window are 0.95 mm in at z 0.05 and
# 8.25, 0.2 mm at z 7.5.
EDGE_CHAMFER = 1.0
# The holes, radius 1.6 straight through (circle fits, z 1 to 6): the hole
# pair 28.5 mm apart along y, and one in each ear.
HOLE_RADIUS = 1.6
HOLE_PAIR_SPACING = 28.5
# A countersunk hole opens at 45° to radius 2.75 at the upper face (circle
# fits: r 1.949 at z 7.5 and 2.700 at 8.25, radius = z - 5.55).
COUNTERSINK_RADIUS = 2.75


def outline(mx, my):
    """The plate's outline as a face at z 0, for a hole pair whose middle is
    (mx, my)."""
    x0, x1 = mx - HALF_LENGTH, mx + HALF_LENGTH
    y0, ye = my - BELOW_MIDDLE, my + EAR_ABOVE_MIDDLE
    yn = my + NOTCH_EDGE_ABOVE_MIDDLE
    r, f, e = FAR_CORNER_RADIUS, NOTCH_CORNER_RADIUS, EAR_RADIUS
    left_ear, right_ear = mx - EAR_OFFSET, mx + EAR_OFFSET
    edges = [
        Line((x0 + r, y0), (x1 - r, y0)),
        CenterArc((x1 - r, y0 + r), r, 270, 90),
        Line((x1, y0 + r), (x1, ye)),
        CenterArc((right_ear, ye), e, 0, 180),
        CenterArc((right_ear - e - f, ye), f, 0, -90),
        Line((right_ear - e - f, yn), (left_ear + e + f, yn)),
        CenterArc((left_ear + e + f, ye), f, 270, -90),
        CenterArc((left_ear, ye), e, 0, 180),
        Line((x0, ye), (x0, y0 + r)),
        CenterArc((x0 + r, y0 + r), r, 180, 90),
    ]
    return make_face(edges)


def holes(mx, my):
    """The hole centres: the pair, then the two ears."""
    pair = [(mx, my - HOLE_PAIR_SPACING / 2), (mx, my + HOLE_PAIR_SPACING / 2)]
    ears = [(mx - EAR_OFFSET, my + EAR_ABOVE_MIDDLE), (mx + EAR_OFFSET, my + EAR_ABOVE_MIDDLE)]
    return pair, ears


def face_edges(body):
    """The edges of the plate's lower and upper faces."""
    return body.edges().group_by(Axis.Z)[0] + body.edges().group_by(Axis.Z)[-1]


def plate(mx, my, window, countersunk, window_chamfer=EDGE_CHAMFER):
    """The side plate: the outline chamfered at both faces, less `window` (a
    face at z 0, cut through) chamfered by `window_chamfer`, with its four
    holes; the holes at the centres in `countersunk` open in a countersink at
    the upper face."""
    body = chamfer(face_edges(extrude(outline(mx, my), THICKNESS)), EDGE_CHAMFER)
    body -= extrude(window, THICKNESS)
    box = window.bounding_box()
    rims = [e for e in face_edges(body)
            if box.min.X - 1e-6 <= e.center().X <= box.max.X + 1e-6
            and box.min.Y - 1e-6 <= e.center().Y <= box.max.Y + 1e-6]
    body = chamfer(rims, window_chamfer)
    pair, ears = holes(mx, my)
    for x, y in pair + ears:
        body -= Pos(x, y, 0) * Cylinder(HOLE_RADIUS, THICKNESS, align=BOTTOM)
    rims = [e for e in body.edges().filter_by_position(Axis.Z, THICKNESS - 1e-6, THICKNESS + 1e-6)
            if e.geom_type.name == "CIRCLE" and abs(e.radius - HOLE_RADIUS) < 1e-6
            and any(abs(e.arc_center.X - x) < 1e-6 and abs(e.arc_center.Y - y) < 1e-6 for x, y in countersunk)]
    if rims:
        body = chamfer(rims, COUNTERSINK_RADIUS - HOLE_RADIUS)
    return body


# --- the connectors -----------------------------------------------------------

# The ring. Source: the downward face at the underside and the upward face
# 8.05 mm above it (Horn Connector z -43.35 and -35.3).
RING_HEIGHT = 8.05
# The outline (section 4 mm above the underside): y ±15.8; the far end, past
# the axis, arcs r 15.000 centred 0.8 mm either side of the axis, joined by a
# flat 1.6 mm long, 15 mm behind the axis.
RING_HALF_WIDTH = 15.8
RING_END_RADIUS = 15.0
# 45° chamfers round the outline's edges at both faces and along the tongue's
# lower side edges: the outline is 0.95 mm in 0.05 mm from either face.
CONNECTOR_CHAMFER = 1.0

# The tongue: y ±7.5 (the side faces at y ±7.5, section 4 mm up).
TONGUE_HALF_WIDTH = 7.5
# The insert hole across the tongue (circle fits in sections at y 0 to 7):
# r 2.2, its axis 21.5 mm from the coxa axis at the ring's mid-height.
INSERT_OFFSET = 21.5
INSERT_HEIGHT = RING_HEIGHT / 2
INSERT_RADIUS = 2.2
# Either side of the tongue the ring ends in a cylinder round the insert
# hole's axis (sections at y 9 to 14.5: arcs r 4.175 centred on it), its
# edges with the upper face rounded r 0.2 and with the underside r 0.1.
EAR_CLEARANCE_RADIUS = 4.175
EAR_CLEARANCE_UPPER_ROUND = 0.2
EAR_CLEARANCE_LOWER_ROUND = 0.1
# The tongue's profile along the axis (section at y 0, from the axis and the
# underside): the underside runs to 36.375, then a 45° slope rises to a flat
# 5.625 up, its corner with the slope rounded r 0.2.
SLOPE_START = 36.375
END_FLAT_HEIGHT = 5.625
END_ROUND = 0.2
# The fin: a wall's face at 28.5 from the axis, with an r 7 cove each side,
# the inner one from the ring's upper face (centre 21.5 out, 15.05 up) and
# the outer one from the end flat (centre 35.5 out, 12.625 up); the fin's top
# a round r 0.2 centred 28.3 out and 12.684 up (Horn Connector z -30.466 at
# its top).
FIN_FACE = 28.5
COVE_RADIUS = 7.0
FIN_ROUND = 0.2
FIN_ROUND_CENTRE = (28.3, 12.684)


def tongue_profile():
    """The tongue's profile in the (x, z) plane, from the axis outward."""
    end = SLOPE_START + END_FLAT_HEIGHT
    cap_z = FIN_ROUND_CENTRE[1]
    outline = Polygon((0, 0), (SLOPE_START, 0), (end, END_FLAT_HEIGHT), (FIN_FACE, END_FLAT_HEIGHT),
                      (FIN_FACE, cap_z), (INSERT_OFFSET, cap_z), (INSERT_OFFSET, RING_HEIGHT), (0, RING_HEIGHT),
                      align=None)
    outline = fillet([v for v in outline.vertices() if abs(v.X - end) < 1e-6], END_ROUND)
    inner_cove = Pos(INSERT_OFFSET, RING_HEIGHT + COVE_RADIUS) * Circle(COVE_RADIUS)
    outer = FIN_FACE + COVE_RADIUS
    outer_fill = Pos(FIN_FACE + COVE_RADIUS / 2, END_FLAT_HEIGHT + COVE_RADIUS / 2) * Rectangle(COVE_RADIUS,
                                                                                                COVE_RADIUS)
    outer_cove = Pos(outer, END_FLAT_HEIGHT + COVE_RADIUS) * Circle(COVE_RADIUS)
    return (outline - inner_cove) + (outer_fill - outer_cove) + Pos(*FIN_ROUND_CENTRE) * Circle(FIN_ROUND)


def ring_outline(end):
    """The ring's outline at z 0, from behind the axis to `end` along +x."""
    r, w = RING_END_RADIUS, RING_HALF_WIDTH
    c = w - r
    return make_face([
        Line((0, -w), (end, -w)), Line((end, -w), (end, w)), Line((end, w), (0, w)),
        CenterArc((0, c), r, 90, 90), Line((-r, c), (-r, -c)), CenterArc((0, -c), r, 180, 90),
    ])


def connector_body():
    """The ring and the tongue, in the connector's own frame: the coxa axis
    at the origin, the underside at z 0, the tongue along +x. The seat in
    the ring is left to each connector.

    The STL's order of operations is kept, as it shows in its sections: the
    ring is cut back round the insert hole's axis and those edges rounded
    first, straight across the ring; the outline's chamfers then cut through
    the rounds where they meet (so the rounds' ends are flat, not swept along
    the chamfers); the tongue, chamfered along its lower side edges, then
    fills the ring between the ears."""
    ring = extrude(ring_outline(INSERT_OFFSET), RING_HEIGHT) - Pos(INSERT_OFFSET, 0, INSERT_HEIGHT) * (
        Plane.XZ * Cylinder(EAR_CLEARANCE_RADIUS, 2 * RING_HALF_WIDTH + 1))

    def clearance_edges(upper):
        z = RING_HEIGHT if upper else 0.0
        return [e for e in ring.edges().filter_by(Axis.Y) if abs(e.center().Z - z) < 1e-6
                and abs(e.center().X - INSERT_OFFSET) < EAR_CLEARANCE_RADIUS]

    ring = fillet(clearance_edges(True), EAR_CLEARANCE_UPPER_ROUND)
    ring = fillet(clearance_edges(False), EAR_CLEARANCE_LOWER_ROUND)
    # The outline's chamfers, as the chamfered outline (run on past the
    # clearance) that the ring is cut to.
    reach = INSERT_OFFSET + 2 * EAR_CLEARANCE_RADIUS
    outline = extrude(ring_outline(reach), RING_HEIGHT)
    ring &= chamfer(outline.edges().group_by(Axis.Z)[0] + outline.edges().group_by(Axis.Z)[-1], CONNECTOR_CHAMFER)

    tongue = extrude(Plane.XZ * tongue_profile(), TONGUE_HALF_WIDTH, both=True)
    lower = [e for e in tongue.edges().filter_by(Axis.X) if abs(e.center().Z) < 1e-6]
    tongue = chamfer(lower, CONNECTOR_CHAMFER)
    hole = Pos(INSERT_OFFSET, 0, INSERT_HEIGHT) * (Plane.XZ * Cylinder(INSERT_RADIUS, 2 * TONGUE_HALF_WIDTH))
    return ring + tongue - hole


def turned(points):
    """A solid turned about the z axis from a profile in the (r, z) plane:
    `points` are (r, z) corners, or ((r, z), (cr, cz)) for a quarter arc to
    (r, z) centred on (cr, cz). Seats are cut with it rather than rounded
    with fillets: a fillet's torus comes out of the STEP as an open shell in
    solidcraft-cli 0.30.1, a surface of revolution does not (as the Bearing
    Axle Mount's boss foot)."""
    edges, here = [], None
    for p in points + [points[0]]:
        end, centre = (p, None) if not isinstance(p[0], tuple) else p
        if here is not None:
            if centre is None:
                edges.append(Line(here, end))
            else:
                mid = [(a + b) / 2 - c for a, b, c in zip(here, end, centre)]
                k = (sum((a - c) ** 2 for a, c in zip(here, centre)) ** 0.5) / (sum(m * m for m in mid) ** 0.5)
                edges.append(ThreePointArc(here, tuple(c + m * k for c, m in zip(centre, mid)), end))
        here = end
    return revolve(Plane.XZ * make_face(edges), Axis.Z, 360)
