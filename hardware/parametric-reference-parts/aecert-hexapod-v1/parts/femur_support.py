"""Femur Support: the bar between the femur side plates, screwed to each
through its ends.

A bar along y, its cross-section (in x and z) a rectangle whose two sides
bulge outward in a parabola, widest at mid-height: the outline widens from
29.58 mm at the top and bottom faces to 33.198 mm at mid-height. Two
rounded windows through it, chamfered top and bottom, and a blind screw hole
into each end. Dimensions are measured from `Femur Support.stl` with trimesh
(vertical sections at y 52 to 108 and x 276.5 and 278.446, horizontal
sections at z -0.6 to 6; line, circle and parabola fits to the section
loops). Coordinates are the STL's.
"""

from build123d import Align, Axis, Bezier, Cylinder, Line, Pos, RectangleRounded, Rot, chamfer, extrude, make_face

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The bar's centre line in plan. Source: the sections' outlines, x 261.847 to
# 295.045 and y 51.765 to 108.465.
CENTRE_X = 278.446
START_Y = 51.765
LENGTH = 56.7
# Its downward and upward faces, z -0.675 and 6.096.
BOTTOM_Z = -0.675
TOP_Z = 6.096
# The flat faces' width: their edges at x 263.656 and 293.236 (sections at
# y 53, 56 and 80).
FLAT_WIDTH = 29.58
# Each side between those edges is a parabola about the horizontal at
# mid-height, its vertex BULGE beyond them: least-squares fits x = x0 +
# k (z - zc)^2 to the vertices of each side in sections at y 52, 60, 80 and
# 108 give vertices at x 261.8485 to 261.8493 and 295.0433 to 295.0440,
# k 0.1579 to 0.1581, to 0.006 mm (a circle fits them only to 0.081 mm).
# A quadratic Bézier through the side's two edges is that parabola.
BULGE = 1.809

# The two windows, 20 by 18 mm with r 4 corners (section at z 2.7: x 268.446
# to 288.446, y 59.115 to 77.115 and 83.115 to 101.115), 12 mm either side
# of the bar's middle along y, chamfered 1 mm at 45° top and bottom (the
# corner arcs r 4.904 at z 6.0, section at x 278.446).
WINDOW_LENGTH = 20.0
WINDOW_WIDTH = 18.0
WINDOW_CORNER_RADIUS = 4.0
WINDOW_OFFSET = 12.0
WINDOW_CHAMFER = 1.0

# The screw hole into each end, along y on the bar's centre line at
# mid-height: r 2.198 (circle fits at y 53 and 56: centre x 278.446,
# z 2.711), 6 mm deep with a flat end (section at x 278.446: y 51.765 to
# 57.765 and 102.465 to 108.465).
END_HOLE_RADIUS = 2.198
END_HOLE_DEPTH = 6.0


def build():
    mid_z = (BOTTOM_Z + TOP_Z) / 2
    half = FLAT_WIDTH / 2
    edges = [Line((CENTRE_X - half, START_Y, BOTTOM_Z), (CENTRE_X + half, START_Y, BOTTOM_Z)),
             Line((CENTRE_X + half, START_Y, TOP_Z), (CENTRE_X - half, START_Y, TOP_Z))]
    for side in (-1, 1):
        x = CENTRE_X + side * half
        # The control point that puts the parabola's vertex BULGE out:
        # a quadratic Bézier's midpoint lies halfway to its control point.
        edges.append(Bezier((x, START_Y, BOTTOM_Z), (x + side * 2 * BULGE, START_Y, mid_z), (x, START_Y, TOP_Z)))
    bar = extrude(make_face(edges), LENGTH, dir=(0, 1, 0))

    middle_y = START_Y + LENGTH / 2
    for dy in (-WINDOW_OFFSET, WINDOW_OFFSET):
        bar -= Pos(CENTRE_X, middle_y + dy, BOTTOM_Z) * extrude(
            RectangleRounded(WINDOW_LENGTH, WINDOW_WIDTH, WINDOW_CORNER_RADIUS), TOP_Z - BOTTOM_Z)
    centres = [middle_y + dy for dy in (-WINDOW_OFFSET, WINDOW_OFFSET)]
    windows = [e for e in bar.edges().group_by(Axis.Z)[0] + bar.edges().group_by(Axis.Z)[-1]
               if abs(e.center().X - CENTRE_X) <= WINDOW_LENGTH / 2 + 1e-6
               and any(abs(e.center().Y - c) <= WINDOW_WIDTH / 2 + 1e-6 for c in centres)]
    bar = chamfer(windows, WINDOW_CHAMFER)

    for y, direction in ((START_Y, 1), (START_Y + LENGTH, -1)):
        bar -= Pos(CENTRE_X, y, mid_z) * Rot(-90 * direction, 0, 0) * Cylinder(
            END_HOLE_RADIUS, END_HOLE_DEPTH, align=BOTTOM)
    return bar


PARTS = [
    {
        "name": "Femur Support",
        "stl": "hardware/aecert-hexapod-v1/stl/Femur Support.stl",
        "build": build,
    },
]
