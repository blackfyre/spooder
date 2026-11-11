"""Foot Shaft: the rod of the sprung foot, printed lying on its side.

Along x, from the low end: a half-hexagon pin with a slanted end, a flat
collar standing upright, the hexagonal shaft, and a square tongue with
rounded corners at the far end. The shaft's section is a hexagon with flats
at the top and bottom and 45° sides meeting in a point on each side, those
two points rounded; the pin is its lower half, the two upper edges rounded.
Dimensions are measured from `Foot Shaft.stl` with trimesh: sections across
x at 943.3 to 1018.3, horizontal sections at z 9.9 to 19.5 and vertical ones
along x at y 327 and 328.699 (circle fits to the section loops' rounds, to
0.002 mm), and the levels of its faces. Coordinates are the STL's.
"""

from build123d import Plane, Polygon, Pos, extrude, fillet

# The shaft's axis, along x. Source: circle fits to the rounds in sections
# across x at 950 (centres y 326.130 and 331.267, z 10.988) and 990 (y
# 325.326 and 332.071, z 11.788): their midpoints; the pin's flat top lies
# on the axis (section at x 944, z 11.788).
AXIS_Y = 328.699
AXIS_Z = 11.788

# The hexagon, from the axis: the flats at z ±2 (the downward face at z 9.788,
# the upward one at 13.788), 5 mm wide (section at x 1015 and 970: y 326.199
# to 331.199), the 45° sides meeting in a point 4.5 mm out (in the section at
# x 970 the sides run from y 326.199, z 9.788 to y 324.764, z 11.222; on the
# line |y| + |z| = 4.5).
HEX_HALF_HEIGHT = 2.0
HEX_FLAT_HALF_WIDTH = 2.5
HEX_POINT = 4.5
# The rounds on the side points (shaft, circle fits at x 990: r 0.796 with
# centres 3.373 mm out, a 0.8 mm round tangent to both sides, the facets
# cutting inside it) and on the pin's upper edges (circle fits at x 950: r
# 0.800, centres 2.569 mm out and 0.8 mm below the axis).
HEX_ROUND = 0.8

# The pin: the hexagon's lower half, from its slanted end to the collar. The
# end face is a plane parallel to y, running from x 943.066 at the pin's top
# (z 11.788) to x 943.809 at its bottom (z 9.788): the same line in the
# sections along x at y 327 and 328.699.
PIN_END_TOP_X = 943.066
PIN_END_BOTTOM_X = 943.809

# The collar: an upright plate, 4 mm thick in x (horizontal sections at z
# 9.9 to 19.5: x 961.484 to 965.484), from the shaft's underside to the
# upward face at z 19.576.
COLLAR_X_MIN = 961.484
COLLAR_X_MAX = 965.484
COLLAR_TOP_Z = 19.576
# Its outline in the section at z 16: a hexagon, its sides x = 961.484 and
# 965.484 ending 4.5 mm either side of the axis, sloping at 35° (vertices
# (961.826, 333.438) to (963.026, 334.278), a slope of 0.70) to a point on
# the plate's middle, 4.5 + 2 * 0.7 = 5.9 mm out. Every corner is rounded:
# circle fits r 0.798 at the points (centres 4.925 mm out: 5.9 less 0.8 /
# sin 55°) and r 0.806 at the four side corners.
COLLAR_CORNER_Y = 4.5
COLLAR_POINT_Y = 5.9
COLLAR_ROUND = 0.8

# The shaft ends at x 1013.252, where the tongue starts (section at z 9.9:
# the outline steps from y 326.087 to 326.199 there).
SHAFT_END_X = 1013.252
# The tongue: 5 mm wide, between the shaft's flats (section at x 1015: y
# 326.199 to 331.199, z 9.788 to 13.788), to x 1018.402, its two end corners
# rounded (circle fits at z 12: r 0.500, centre x 1017.903).
TONGUE_END_X = 1018.402
TONGUE_HALF_WIDTH = 2.5
TONGUE_ROUND = 0.5


def section(points, rounded):
    """A face in the plane square to x, its local x along y and local y
    along z, from the axis; `rounded` are the points whose corners are
    rounded by HEX_ROUND."""
    face = Polygon(*points, align=None)
    corners = [v for v in face.vertices() if any(abs(v.X - u) < 1e-6 and abs(v.Y - w) < 1e-6 for u, w in rounded)]
    return fillet(corners, HEX_ROUND)


def along_x(face, x_from, x_to):
    """The face, placed on the axis at x_from, extruded to x_to."""
    plane = Plane(origin=(x_from, AXIS_Y, AXIS_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return extrude(plane * face, x_to - x_from)


def build():
    h, f, p = HEX_HALF_HEIGHT, HEX_FLAT_HALF_WIDTH, HEX_POINT
    collar_mid = (COLLAR_X_MIN + COLLAR_X_MAX) / 2
    hexagon = section([(-f, -h), (f, -h), (p, 0), (f, h), (-f, h), (-p, 0)], [(p, 0), (-p, 0)])
    shaft = along_x(hexagon, collar_mid, SHAFT_END_X)

    half = section([(-f, -h), (f, -h), (p, 0), (-p, 0)], [(p, 0), (-p, 0)])
    pin = along_x(half, PIN_END_TOP_X, collar_mid)
    base = AXIS_Z - h
    end = Plane.XZ * Polygon((PIN_END_BOTTOM_X, base), (collar_mid, base), (collar_mid, AXIS_Z),
                             (PIN_END_TOP_X, AXIS_Z), align=None)
    # Plane.XZ's normal is -y: extruding both ways covers the pin's width.
    pin = pin & extrude(Pos(0, AXIS_Y, 0) * end, p, both=True)

    outline = Polygon((collar_mid, AXIS_Y - COLLAR_POINT_Y), (COLLAR_X_MAX, AXIS_Y - COLLAR_CORNER_Y),
                      (COLLAR_X_MAX, AXIS_Y + COLLAR_CORNER_Y), (collar_mid, AXIS_Y + COLLAR_POINT_Y),
                      (COLLAR_X_MIN, AXIS_Y + COLLAR_CORNER_Y), (COLLAR_X_MIN, AXIS_Y - COLLAR_CORNER_Y),
                      align=None)
    outline = fillet(outline.vertices(), COLLAR_ROUND)
    collar = Pos(0, 0, base) * extrude(outline, COLLAR_TOP_Z - base)

    # The tongue starts 1 mm inside the shaft, so the two overlap.
    tongue = Polygon((SHAFT_END_X - 1, AXIS_Y - TONGUE_HALF_WIDTH), (TONGUE_END_X, AXIS_Y - TONGUE_HALF_WIDTH),
                     (TONGUE_END_X, AXIS_Y + TONGUE_HALF_WIDTH), (SHAFT_END_X - 1, AXIS_Y + TONGUE_HALF_WIDTH),
                     align=None)
    tongue = fillet([v for v in tongue.vertices() if abs(v.X - TONGUE_END_X) < 1e-6], TONGUE_ROUND)
    tongue = Pos(0, 0, base) * extrude(tongue, 2 * h)

    return pin + collar + shaft + tongue


PARTS = [
    {
        "name": "Foot Shaft",
        "stl": "hardware/aecert-hexapod-v1/stl/Foot Shaft.stl",
        "build": build,
    },
]
