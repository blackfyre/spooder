"""The foot half that Foot Left and Foot Right are built from.

A plate lying flat, printed as it lies: a block at the low-y end, the
serpentine flexure, and a raised boss at the high-y end with a screw hole
and a round post on top. A tented channel runs along y under the whole
plate, leaving a thin hinge over it, and a tented pocket crosses it in the
block. The flexure is a meander of 1 mm beams running along x, joined by
turns that alternate between the two sides; the cuts between the beams
are what its path (`path`) and widths leave, so retuning the spring is a
change of the FLEXURE_ constants. The block's outer edge and the boss's top edge are
chamfered.

Dimensions are measured from `Foot Left.stl` with trimesh: horizontal
sections at z 0.05 to 12 (line and circle fits to the section loops),
vertical sections across x at 110, 114.5, 116.04 and 119 and across y at
58, 58.5, 60, 63, 80.3, 94, 95 and 97.811, and the levels of its faces.
Coordinates are Foot Left's STL's.
"""

import math

from build123d import (Axis, Box, CenterArc, Circle, Line, Plane, Polygon, Pos, Rectangle, extrude, fillet,
                       make_face, revolve)

# The plate's sides: the outline's straight sides and the flexure's outer
# turns at x 104.765 and 121.479 (section at z 3).
LEFT_X = 104.765
RIGHT_X = 121.479
# Source: the downward face at z 0 and the plate's upward face at z 6.5.
PLATE_TOP_Z = 6.5

# The block, below the flexure. Its bottom-left and bottom-right corners are
# r 4 rounds (circle fits at z 3: centres (108.765, 59.322) and (117.480,
# 60.759), r 4.00, the side lines ending at their tangents), joined by their
# common tangent below (through (109.416, 55.376) and (118.130, 56.813));
# its top-right corner is a r 1 round (circle fit at z 3: centre (120.479,
# 63.061), r 0.999). Its top is the first cut's lower side (FLEXURE_).
BOTTOM_ROUND = 4.0
BOTTOM_LEFT_Y = 59.322
BOTTOM_RIGHT_Y = 60.760
BLOCK_CORNER_ROUND = 1.0

# The flexure, a meander along x between LEFT_X and RIGHT_X. Its path starts
# at the first beam's lower side, 0.75 mm above the block (section at z 3:
# the block's top at y 64.061, the first beam from 64.811), and runs up in
# beams 1 mm wide (64.811 to 65.811, ...), FLEXURE_PITCH apart (their lower
# sides at 64.811 + 1.5 k, the last at 90.311: 18 beams), each joined to the
# next by one of FLEXURE_TURNS turns, alternately on the right and the left,
# the first on the right (the cut between the first two beams ends at x
# 119.479). The
# first beam is joined to the block and the last to the boss on the left,
# where the cuts below and above them end. So the cuts between the beams are
# FLEXURE_PITCH - FLEXURE_BEAM_WIDTH (0.5 mm) wide, the boss starts one
# pitch above the last beam (y 91.811), and each cut ends FLEXURE_TURN_WIDTH
# in from the side it is closed on (x 106.765 and 119.479), rounded where it
# meets a beam with half its width (circle fits at z 3: r 0.25, centres x
# 107.015 and 119.229; the first and last cuts have that round on the beam's
# side only, square against the block and the boss). The turns' outer
# corners are FLEXURE_TURN_RADIUS rounds (circle fits at z 3: r 0.875,
# leaving 0.75 mm straight on the 2.5 mm turn).
FLEXURE_START_X = LEFT_X
FLEXURE_START_Y = 64.811
FLEXURE_RUN_LENGTH = RIGHT_X - LEFT_X
FLEXURE_TURNS = 17
FLEXURE_PITCH = 1.5
FLEXURE_BEAM_WIDTH = 1.0
FLEXURE_TURN_WIDTH = 2.0
FLEXURE_TURN_RADIUS = 0.875
# The first cut, between the block and the first beam (section at z 3: y
# 64.061 to 64.811).
FLEXURE_BLOCK_GAP = 0.75

# The boss, from the flexure to the far end, raised to its upward face at
# z 7.45 (vertical sections across x at 110 and 119). Its top is the line y
# 103.811 between two r 4 rounds (circle fits at z 3: centres x 109.923 and
# 116.090, y 99.811, r 4.00). Its sides are shallow arcs (circle fits to
# their vertices at z 3, to 0.0002 mm): the left one bulging out, r 194.42,
# from the flexure's corner (104.765, 91.811) to the left round, tangent to
# it inside (centre 190.42 mm from the round's); the right one hollow, r
# 296.47, tangent outside to the right round (centre 300.47 mm from it) and
# to a r 1 round on the boss's bottom-right corner (circle fit at z 3:
# centre (120.283, 92.811), r 0.999; 297.48 mm from the side's centre).
BOSS_TOP_Z = 7.45
BOSS_END_Y = 103.811
BOSS_ROUND = 4.0
BOSS_LEFT_ROUND_X = 109.923
BOSS_RIGHT_ROUND_X = 116.090
BOSS_CORNER_X = 120.283
BOSS_CORNER_ROUND = 1.0
BOSS_LEFT_SIDE_RADIUS = 194.42
BOSS_RIGHT_SIDE_RADIUS = 296.47

# 45° chamfers, 1 mm: round the block's outer edge from z 5.5 to the plate's
# top (vertical sections across y at 58.5 and 63: x 104.852 at z 5.5 to
# 105.881 at 6.5), and round the boss's top from z 6.45 (section across y
# at 94: x 105.036 at z 6.45 to 106.043 at 7.45). Each is the cut of its
# sides carried on past the rounds where it stops, the block's on the left
# up through the first beam's turn and on the right to the first cut, the
# boss's down to the flexure: the block's r 1 corner and the boss's r 1
# corner keep their rounds where the cut leaves them (section at z 6: the
# block's right side at x 120.979 meets the unchamfered round at y 63.926;
# at z 7 the boss's corner round runs unchamfered to (120.883, 92.013)).
EDGE_CHAMFER = 1.0
CHAMFER_ANGLE = 45.0  # degrees from vertical: 1 mm in for 1 mm up

# The channel along y under the whole plate: 4.6 mm wide (x 112.252 to
# 116.852), its walls 2.624 mm high, under a 45° roof meeting at z 4.924
# above x 114.552 (vertical sections across y at 80.3 and 95).
CHANNEL_X = 114.552
CHANNEL_HALF_WIDTH = 2.3
CHANNEL_WALL_Z = 2.624
CHANNEL_ROOF_SLOPE = 1.0  # 45°: from (112.252, 2.624) to (114.552, 4.924)
# Below the pocket's ridge (y 59.893) the channel keeps only its far half: a
# wall at x 114.252, 0.3 mm short of the ridge, up to the far slope carried
# on (vertical sections across y at 56.4 and 57: x 114.252 from z 0 to 5.224;
# at 57.4 to 58.0 the same wall above the pocket's roof, which reaches 5.224
# at y 58.404, the vertex there).
CHANNEL_END_X = 114.252

# The pocket across the channel in the block: 4.6 mm wide (y 57.593 to
# 62.193), its walls 4.656 mm high, under a roof sloping 0.70 (35°) to its
# ridge above y 59.893 (vertical section across x at 109: (57.857, 4.841) to
# (59.409, 5.927)); from its end wall at x 106.465 to the channel's far wall
# (vertical sections across y at 58.5).
POCKET_Y = 59.893
POCKET_HALF_WIDTH = 2.3
POCKET_WALL_Z = 4.656
POCKET_ROOF_SLOPE = 0.70
POCKET_X_MIN = 106.465

# The screw hole through the boss (circle fits at z 3 to 7: centre (110.064,
# 97.811), r 1.599).
HOLE_X = 110.064
HOLE_Y = 97.811
HOLE_RADIUS = 1.6

# The post on the boss: r 2 (circle fits at z 9 to 11: centre (116.041,
# 97.811), r 1.999) to its top at z 12.45, the top edge a r 1 round (the
# vertical section across y at 97.811: r 2 up to z 11.45, the flat from r 1),
# its foot a r 1.2 fillet onto the boss (the vertices between z 7.46 and
# 8.55 lie 1.200 to 1.211 mm from a centre 3.2 mm out at z 8.65).
POST_X = 116.041
POST_Y = 97.811
POST_RADIUS = 2.0
POST_TOP_Z = 12.45
POST_TOP_ROUND = 1.0
POST_FOOT_ROUND = 1.2

FAR = 5.0  # mm, how far the cutting tools reach past the plate


def runs():
    """The flexure's beams, one more than its turns."""
    return FLEXURE_TURNS + 1


def boss_start_y():
    """The boss's lower side: one pitch above the last beam's lower side."""
    return FLEXURE_START_Y + runs() * FLEXURE_PITCH


def block_top_y():
    return FLEXURE_START_Y - FLEXURE_BLOCK_GAP


def intersect(p, d, q, e):
    """The point where the line through p along d meets the line through q
    along e."""
    det = d[0] * -e[1] + e[0] * d[1]
    t = ((q[0] - p[0]) * -e[1] + e[0] * (q[1] - p[1])) / det
    return (p[0] + t * d[0], p[1] + t * d[1])


def outer_tangent(c1, r1, c2, r2, right):
    """A point and direction of the tangent to two circles that keeps both on
    one side: on the left of the line from c1 to c2 when `right` (the line
    lies to their right), else on its right."""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    dist = math.hypot(dx, dy)
    base = math.atan2(dy, dx)
    turn = math.acos((r1 - r2) / dist)
    a = base - turn if right else base + turn
    n = (math.cos(a), math.sin(a))
    p1 = (c1[0] + r1 * n[0], c1[1] + r1 * n[1])
    p2 = (c2[0] + r2 * n[0], c2[1] + r2 * n[1])
    return p1, (p2[0] - p1[0], p2[1] - p1[1])


def rounded(points, radii):
    """The polygon through `points`, each corner listed in `radii` (by index)
    rounded with its radius."""
    face = Polygon(*points, align=None)
    for i, r in sorted(radii.items(), key=lambda kv: kv[1]):
        x, y = points[i]
        face = fillet([v for v in face.vertices() if abs(v.X - x) < 1e-6 and abs(v.Y - y) < 1e-6], r)
    return face


def block_corners(top):
    """The block's outline corners, from top left anticlockwise, with its
    top at `top`: the bottom corners where the sides meet the common tangent
    of the bottom rounds."""
    c1 = (LEFT_X + BOTTOM_ROUND, BOTTOM_LEFT_Y)
    c2 = (RIGHT_X - BOTTOM_ROUND, BOTTOM_RIGHT_Y)
    p, d = outer_tangent(c1, BOTTOM_ROUND, c2, BOTTOM_ROUND, right=True)
    return [(LEFT_X, top), intersect((LEFT_X, 0), (0, 1), p, d), intersect((RIGHT_X, 0), (0, 1), p, d),
            (RIGHT_X, top)]


def circles_meet(c1, r1, c2, r2, right):
    """Where the circles at c1 and c2 meet: the point on the right of the line
    from c1 to c2 when `right`, else on its left."""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    dist = math.hypot(dx, dy)
    a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist)
    h = math.sqrt(r1 * r1 - a * a)
    mx, my = c1[0] + a * dx / dist, c1[1] + a * dy / dist
    s = -1 if right else 1
    return (mx - s * h * dy / dist, my + s * h * dx / dist)


def angle(c, p):
    return math.degrees(math.atan2(p[1] - c[1], p[0] - c[0]))


def arc(c, r, p, q):
    """The shorter arc of the circle at c from the angle of p to that of q."""
    a0 = angle(c, p)
    size = (angle(c, q) - a0 + 180) % 360 - 180
    return CenterArc(c, r, a0, size)


def boss_outline(bottom_y, corner):
    """The boss's outline, from its lower side at y `bottom_y`: with the
    bottom-right corner round when `corner`, else with both sides carried
    down to `bottom_y`."""
    start = boss_start_y()
    centre_y = BOSS_END_Y - BOSS_ROUND
    tl, tr = (BOSS_LEFT_ROUND_X, centre_y), (BOSS_RIGHT_ROUND_X, centre_y)
    cc = (BOSS_CORNER_X, start + BOSS_CORNER_ROUND)
    rl, rr = BOSS_LEFT_SIDE_RADIUS, BOSS_RIGHT_SIDE_RADIUS
    # The left side's centre: rl from the flexure's corner and rl - 4 from
    # the left round's, inside the boss; the right side's: rr + 1 from the
    # corner round's and rr + 4 from the right round's, outside it.
    cl = circles_meet((LEFT_X, start), rl, tl, rl - BOSS_ROUND, right=True)
    cr = circles_meet(cc, rr + BOSS_CORNER_ROUND, tr, rr + BOSS_ROUND, right=True)

    def towards(c, r, d):
        k = r / math.hypot(d[0] - c[0], d[1] - c[1])
        return (c[0] + k * (d[0] - c[0]), c[1] + k * (d[1] - c[1]))

    left_t = towards(tl, BOSS_ROUND, (2 * tl[0] - cl[0], 2 * tl[1] - cl[1]))  # tangent inside
    right_t = towards(tr, BOSS_ROUND, cr)  # tangent outside
    top_l, top_r = (tl[0], BOSS_END_Y), (tr[0], BOSS_END_Y)
    edges = []
    if corner:
        bottom_l = (LEFT_X, start)
        corner_t = towards(cc, BOSS_CORNER_ROUND, cr)
        edges += [Line(bottom_l, (cc[0], start)), arc(cc, BOSS_CORNER_ROUND, (cc[0], start), corner_t),
                  arc(cr, rr, corner_t, right_t)]
    else:
        bottom_l = (cl[0] - math.sqrt(rl * rl - (bottom_y - cl[1]) ** 2), bottom_y)
        bottom_r = (cr[0] - math.sqrt(rr * rr - (bottom_y - cr[1]) ** 2), bottom_y)
        edges += [Line(bottom_l, bottom_r), arc(cr, rr, bottom_r, right_t)]
    edges += [arc(tr, BOSS_ROUND, right_t, top_r), Line(top_r, top_l), arc(tl, BOSS_ROUND, top_l, left_t),
              arc(cl, rl, left_t, bottom_l)]
    return make_face(edges)


def rect(xa, xb, ya, yb):
    return Pos((xa + xb) / 2, (ya + yb) / 2) * Rectangle(abs(xb - xa), abs(yb - ya))


def cut_end(x_end, toward, y0, y1, round_low, round_high, r):
    """The closed end of a cut between two beams from y0 to y1, at x_end,
    reaching 2 r towards `toward` (+1 right, -1 left), its corners rounded
    with r where `round_low` and `round_high`."""
    inner = x_end + toward * r
    out = x_end + toward * 2 * r
    shapes = [rect(inner, out, y0, y1)]
    lo = y0 + (r if round_low else 0)
    hi = y1 - (r if round_high else 0)
    if hi - lo > 1e-6:
        shapes.append(rect(x_end, out, lo, hi))
    centres = set()
    if round_low:
        centres.add(round(y0 + r, 9))
    if round_high:
        centres.add(round(y1 - r, 9))
    shapes += [Pos(inner, cy) * Circle(r) for cy in sorted(centres)]
    face = shapes[0]
    for shape in shapes[1:]:
        face = face + shape
    return face


def path():
    """The meander's path from the block to the boss, as its runs and turns
    in order. A run is ("run", y), the beam's lower side; a turn is ("turn",
    side, low, high, gap_low, gap_high, corners): on the left (-1) or right
    (+1), joining the beams (or the block or boss) between y low and high
    round the cut from gap_low to gap_high, with its outer corners rounded at
    the ends named in `corners` ("low", "high"). The block's and the boss's
    turns are on the side away from their beams' other turns."""
    p, w, n = FLEXURE_PITCH, FLEXURE_BEAM_WIDTH, runs()
    beam = [FLEXURE_START_Y + i * p for i in range(n)]
    side = [1 if i % 2 == 0 else -1 for i in range(n - 1)]  # turn i joins beams i and i + 1
    first = -side[0] if side else -1
    last = -side[-1] if side else -first
    segments = [("turn", first, block_top_y(), beam[0] + w, block_top_y(), beam[0], ("high",))]
    for i in range(n):
        segments.append(("run", beam[i]))
        if i < n - 1:
            segments.append(("turn", side[i], beam[i], beam[i + 1] + w, beam[i] + w, beam[i + 1], ("low", "high")))
    segments.append(("turn", last, beam[-1], boss_start_y(), beam[-1] + w, boss_start_y(), ("low",)))
    return segments


def flexure():
    """The flexure: its beams along the path's runs, and its turns, each a
    block FLEXURE_TURN_WIDTH wide past the cut it closes, the cut's end
    rounded with half its width where it meets a beam."""
    w, r = FLEXURE_BEAM_WIDTH, (FLEXURE_PITCH - FLEXURE_BEAM_WIDTH) / 2
    left = FLEXURE_START_X
    right = FLEXURE_START_X + FLEXURE_RUN_LENGTH
    face = None
    for seg in path():
        if seg[0] == "run":
            shape = rect(left + FLEXURE_TURN_WIDTH, right - FLEXURE_TURN_WIDTH, seg[1], seg[1] + w)
        else:
            _, side, low, high, gap_low, gap_high, corners = seg
            edge = right if side > 0 else left
            x_end = edge - side * FLEXURE_TURN_WIDTH
            shape = rect(edge, x_end - side * r, low, high)
            outer = [(edge, low)] * ("low" in corners) + [(edge, high)] * ("high" in corners)
            vertices = [v for v in shape.vertices() if any(abs(v.X - x) < 1e-6 and abs(v.Y - y) < 1e-6
                                                           for x, y in outer)]
            if vertices:
                shape = fillet(vertices, FLEXURE_TURN_RADIUS)
            # The cut's end, rounded where a beam (not the block or boss) meets it.
            shape = shape - cut_end(x_end, -side, gap_low, gap_high, gap_low > block_top_y(),
                                    gap_high < boss_start_y(), r)
        face = shape if face is None else face + shape
    return face


def plan():
    """The plate's outline: the block, the flexure and the boss."""
    block = rounded(block_corners(block_top_y()), {1: BOTTOM_ROUND, 2: BOTTOM_ROUND, 3: BLOCK_CORNER_ROUND})
    boss = boss_outline(boss_start_y(), corner=True)
    return block + flexure() + boss, boss


def chamfer_cut(outline, z, box_min, box_max):
    """The material a 1 mm chamfer at height z removes inside the box: all of
    the box outside `outline`, tapering in by EDGE_CHAMFER up to
    z + EDGE_CHAMFER."""
    keep = Pos(0, 0, z) * extrude(outline, EDGE_CHAMFER, taper=CHAMFER_ANGLE)
    size = [b - a for a, b in zip(box_min, box_max)]
    centre = [(a + b) / 2 for a, b in zip(box_min, box_max)]
    return Pos(*centre) * Box(*size) - keep


def tent(half_width, wall, slope):
    """A tented section, centred on local x = 0 and standing on local y = 0."""
    return Polygon((-half_width, 0), (half_width, 0), (half_width, wall), (0, wall + slope * half_width),
                   (-half_width, wall), align=None)


def post():
    foot = POST_RADIUS + POST_FOOT_ROUND
    shoulder = POST_TOP_Z - BOSS_TOP_Z - POST_TOP_ROUND
    flat = POST_RADIUS - POST_TOP_ROUND
    below = -1.0  # into the boss, so the two fuse
    profile = make_face([
        Line((0, below), (foot, below)),
        Line((foot, below), (foot, 0)),
        CenterArc((foot, POST_FOOT_ROUND), POST_FOOT_ROUND, 270, -90),
        Line((POST_RADIUS, POST_FOOT_ROUND), (POST_RADIUS, shoulder)),
        CenterArc((flat, shoulder), POST_TOP_ROUND, 0, 90),
        Line((flat, POST_TOP_Z - BOSS_TOP_Z), (0, POST_TOP_Z - BOSS_TOP_Z)),
        Line((0, POST_TOP_Z - BOSS_TOP_Z), (0, below)),
    ])
    return Pos(POST_X, POST_Y, BOSS_TOP_Z) * revolve(Plane.XZ * profile, Axis.Z, 360)


def foot_half():
    """Foot Left's solid, in its STL's coordinates."""
    outline, boss = plan()
    body = extrude(outline, PLATE_TOP_Z) + extrude(boss, BOSS_TOP_Z)

    # The block's chamfer: its sides carried up past the block (FAR), cut on
    # the left up to the middle of the cut above the first beam and on the
    # right up to the middle of the first cut.
    low = BOTTOM_LEFT_Y - BOTTOM_ROUND - FAR
    left_end = FLEXURE_START_Y + (FLEXURE_PITCH + FLEXURE_BEAM_WIDTH) / 2
    right_end = block_top_y() + FLEXURE_BLOCK_GAP / 2
    block = rounded(block_corners(left_end + FAR), {1: BOTTOM_ROUND, 2: BOTTOM_ROUND})
    z = PLATE_TOP_Z - EDGE_CHAMFER
    body -= chamfer_cut(block, z, (LEFT_X - FAR, low, z), (CHANNEL_X, left_end, PLATE_TOP_Z))
    body -= chamfer_cut(block, z, (CHANNEL_X, low, z), (RIGHT_X + FAR, right_end, PLATE_TOP_Z))

    # The boss's chamfer, its sides carried down below the boss.
    z = BOSS_TOP_Z - EDGE_CHAMFER
    body -= chamfer_cut(boss_outline(boss_start_y() - FAR, corner=False), z, (LEFT_X - FAR, boss_start_y(), z),
                        (RIGHT_X + FAR, BOSS_END_Y + FAR, BOSS_TOP_Z))

    # The channel along y, from the pocket's ridge on, its far half below it,
    # and the pocket across it. Plane.XZ's normal is -y.
    end_y = POCKET_Y
    far_x = CHANNEL_X + CHANNEL_HALF_WIDTH
    channel = Plane.XZ * tent(CHANNEL_HALF_WIDTH, CHANNEL_WALL_Z, CHANNEL_ROOF_SLOPE)
    body -= Pos(CHANNEL_X, BOSS_END_Y + FAR, 0) * extrude(channel, BOSS_END_Y + FAR - end_y)
    end_z = CHANNEL_WALL_Z + CHANNEL_ROOF_SLOPE * (far_x - CHANNEL_END_X)
    half = Plane.XZ * Polygon((CHANNEL_END_X, 0), (far_x, 0), (far_x, CHANNEL_WALL_Z), (CHANNEL_END_X, end_z),
                              align=None)
    low = BOTTOM_LEFT_Y - BOTTOM_ROUND - FAR
    body -= Pos(0, end_y, 0) * extrude(half, end_y - low)
    pocket_end = far_x
    pocket = Plane.YZ * tent(POCKET_HALF_WIDTH, POCKET_WALL_Z, POCKET_ROOF_SLOPE)
    body -= Pos(POCKET_X_MIN, POCKET_Y, 0) * extrude(pocket, pocket_end - POCKET_X_MIN)

    hole = Pos(HOLE_X, HOLE_Y, -1) * extrude(Circle(HOLE_RADIUS), BOSS_TOP_Z + 2)
    body -= hole
    return body + post()
