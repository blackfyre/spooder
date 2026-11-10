"""Servo Mount: the frame an MG996R servo sits in, clamped by its ears.

A 15 mm ring round the servo's body (57 × 37 mm outside, r7 corners, 1 mm
chamfers round its top and bottom edges), its window 40.3 × 20.3 mm with a
cable notch at one end; two 5 mm ear pads on top, each with two blind screw
holes; four hexagonal windows through each long side, a hexagonal tunnel
along the mount through both ends, and the two vertical screw holes, 28.5 mm
apart, by which the mount is screwed to the frame or a coxa link. Dimensions
are measured from `Servo Mount.stl` with trimesh: horizontal sections at z
6.02 to 25.9, vertical sections at x 192 to 244.6 and y 125 to 146 (line and
circle fits to the section loops, to 0.002 mm), and the levels of its
horizontal faces. Coordinates are the STL's.
"""

from build123d import (Align, Axis, Box, Cone, Cylinder, Plane, Polygon, Pos, RectangleRounded, chamfer,
                       extrude, fillet)

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The mount's centre in plan (the ring's and the window's centre lines).
CENTRE_X = 219.605
CENTRE_Y = 139.404
# The ring. Source: the downward face at z 6 and the upward face at z 21; the
# outline in the section at z 7.5 (x 191.105 to 248.105, y 120.904 to
# 157.904, corner arcs r 7.000).
RING_BOTTOM_Z = 6.0
RING_HEIGHT = 15.0
RING_LENGTH = 57.0
RING_WIDTH = 37.0
RING_CORNER_RADIUS = 7.0
# 45° chamfers round every edge of the ring's top and bottom faces, outside
# and in the window: the outline is 0.5 mm in at z 6.5 (corner arcs r 6.5 and
# the window's r 1.5).
EDGE_CHAMFER = 1.0
# The servo's window (section at z 7.5): x 199.455 to 239.755, y 129.254 to
# 149.554, corner arcs r 1.
WINDOW_LENGTH = 40.3
WINDOW_WIDTH = 20.3
WINDOW_CORNER_RADIUS = 1.0
# The cable notch at the window's -x end (sections at z 6.5 to 23.5): it
# narrows from 8.0 mm wide at the window to 7.0 mm at its end, x 192.855,
# its corners rounded r 1 both at the end and where it meets the window. Not
# chamfered.
NOTCH_END_X = 192.855
NOTCH_WIDTH_AT_WINDOW = 8.0
NOTCH_WIDTH_AT_END = 7.0
NOTCH_CORNER_RADIUS = 1.0

# The ear pads on the ring's ends (sections at z 21.1 to 25.9 and x 192 to
# 244.6): from the ring's ends to the window, y 129.554 to 149.254, up to
# z 26. The ring's top chamfers stop at them.
PAD_TOP_Z = 26.0
PAD_WIDTH = 19.7
# Each pad's two screw holes: radius 1.05, 3.5 mm in from the ring's end and
# 5.2 mm either side of the centre line, down to z 21.2, ending in a cone to
# a point at z 19.1 (radius = (z - 19.1) / 2, sections at z 19.3 to 21.1).
PAD_HOLE_RADIUS = 1.05
PAD_HOLE_INSET = 3.5
PAD_HOLE_SPACING = 10.4
PAD_HOLE_BOTTOM_Z = 21.2
PAD_HOLE_TIP_Z = 19.1

# The hexagonal windows through each long side (section at y 125), their
# centres at x 219.605 ± 7.2 and ± 16.2 and z 13.5: 7 mm wide with vertical
# sides 8.804 mm tall, closing to a 1 mm flat at z 6.998 and 20.002.
SIDE_WINDOW_OFFSETS = (-16.2, -7.2, 7.2, 16.2)
SIDE_WINDOW_Z = 13.5
SIDE_WINDOW_WIDTH = 7.0
SIDE_WINDOW_SIDE_HEIGHT = 8.804
SIDE_WINDOW_HEIGHT = 13.004
HEX_FLAT = 1.0
# The hexagonal tunnel along the mount, through both ends (section at x 192):
# centred on y 139.404 and z 15, 9.8 mm wide with vertical sides 7 mm tall,
# closing to a 1 mm flat at z 8 and 22.
TUNNEL_Z = 15.0
TUNNEL_WIDTH = 9.8
TUNNEL_SIDE_HEIGHT = 7.0
TUNNEL_HEIGHT = 14.0

# The vertical screw holes, 28.5 mm apart across the mount (circle fits,
# z 6.02 to 20.98): radius 2.2, straight through the ring.
SCREW_SPACING = 28.5
SCREW_RADIUS = 2.2


def hexagon(width, side_height, height):
    """A hexagon in the local (u, v) plane, centred on the origin: vertical
    sides `side_height` tall, closing to a flat HEX_FLAT wide at ±height/2."""
    w, f, s, h = width / 2, HEX_FLAT / 2, side_height / 2, height / 2
    return Polygon((f, -h), (w, -s), (w, s), (f, h), (-f, h), (-w, s), (-w, -s), (-f, -h), align=None)


def build():
    top = RING_BOTTOM_Z + RING_HEIGHT
    ring = Pos(CENTRE_X, CENTRE_Y, RING_BOTTOM_Z) * extrude(
        RectangleRounded(RING_LENGTH, RING_WIDTH, RING_CORNER_RADIUS), RING_HEIGHT)
    window = Pos(CENTRE_X, CENTRE_Y, RING_BOTTOM_Z) * extrude(
        RectangleRounded(WINDOW_LENGTH, WINDOW_WIDTH, WINDOW_CORNER_RADIUS), PAD_TOP_Z - RING_BOTTOM_Z)
    ring -= window
    ring = chamfer(ring.edges().group_by(Axis.Z)[0] + ring.edges().group_by(Axis.Z)[-1], EDGE_CHAMFER)

    # The pads, filling the top chamfers beneath them.
    window_x0 = CENTRE_X - WINDOW_LENGTH / 2
    pad_length = window_x0 - (CENTRE_X - RING_LENGTH / 2) + WINDOW_CORNER_RADIUS
    pad_depth = PAD_TOP_Z - (top - EDGE_CHAMFER)
    for side in (-1, 1):
        x = CENTRE_X + side * (RING_LENGTH / 2 - pad_length / 2)
        ring += Pos(x, CENTRE_Y, top - EDGE_CHAMFER) * Box(pad_length, PAD_WIDTH, pad_depth, align=BOTTOM)
    ring -= window

    # The cable notch: a tapered slot from the window's end, its corners
    # rounded where it ends and where it meets the window.
    a, b = NOTCH_WIDTH_AT_WINDOW / 2, NOTCH_WIDTH_AT_END / 2
    overlap = 2 * WINDOW_CORNER_RADIUS
    slope = (a - b) / (window_x0 - NOTCH_END_X)
    reach = a + slope * overlap
    notch = Polygon((NOTCH_END_X, CENTRE_Y - b), (window_x0 + overlap, CENTRE_Y - reach),
                    (window_x0 + overlap, CENTRE_Y + reach), (NOTCH_END_X, CENTRE_Y + b), align=None)
    ring -= Pos(0, 0, RING_BOTTOM_Z) * extrude(notch, PAD_TOP_Z - RING_BOTTOM_Z)
    corners = [e for e in ring.edges().filter_by(Axis.Z)
               if abs(e.center().X - NOTCH_END_X) < 1e-6 or abs(e.center().X - window_x0) < 1e-6
               and abs(abs(e.center().Y - CENTRE_Y) - a) < 1e-6]
    ring = fillet(corners, NOTCH_CORNER_RADIUS)

    # The tunnel along the mount and the side windows across it.
    tunnel_plane = Plane(origin=(CENTRE_X, CENTRE_Y, TUNNEL_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    ring -= extrude(tunnel_plane * hexagon(TUNNEL_WIDTH, TUNNEL_SIDE_HEIGHT, TUNNEL_HEIGHT), RING_LENGTH,
                    both=True)
    for dx in SIDE_WINDOW_OFFSETS:
        plane = Plane(origin=(CENTRE_X + dx, CENTRE_Y, SIDE_WINDOW_Z), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
        ring -= extrude(plane * hexagon(SIDE_WINDOW_WIDTH, SIDE_WINDOW_SIDE_HEIGHT, SIDE_WINDOW_HEIGHT),
                        RING_WIDTH, both=True)

    for side in (-1, 1):
        ring -= Pos(CENTRE_X, CENTRE_Y + side * SCREW_SPACING / 2, RING_BOTTOM_Z) * Cylinder(
            SCREW_RADIUS, RING_HEIGHT, align=BOTTOM)
        x = CENTRE_X + side * (RING_LENGTH / 2 - PAD_HOLE_INSET)
        for dy in (-PAD_HOLE_SPACING / 2, PAD_HOLE_SPACING / 2):
            ring -= Pos(x, CENTRE_Y + dy, PAD_HOLE_BOTTOM_Z) * Cylinder(
                PAD_HOLE_RADIUS, PAD_TOP_Z - PAD_HOLE_BOTTOM_Z, align=BOTTOM)
            ring -= Pos(x, CENTRE_Y + dy, PAD_HOLE_TIP_Z) * Cone(
                0, PAD_HOLE_RADIUS, PAD_HOLE_BOTTOM_Z - PAD_HOLE_TIP_Z, align=BOTTOM)
    return ring


PARTS = [
    {
        "name": "Servo Mount",
        "stl": "hardware/aecert-hexapod-v1/stl/Servo Mount.stl",
        "build": build,
    },
]
