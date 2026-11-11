"""Foot Tip: the rounded cap of the sprung foot.

A 10.4 mm cylinder standing upright, its top edge rounded with a 4.5 mm
round that leaves a small flat on top, and an octagonal socket up from its
underside. Dimensions are measured from `Foot Tip.stl` with
trimesh: horizontal sections at z -52.1 to -42.75 (circle fits to the
section loops, to 0.001 mm), the radius of each ring of vertices on the
round, and the levels of its horizontal faces. Coordinates are the STL's.
"""

from build123d import Axis, CenterArc, Line, Plane, Polygon, Pos, extrude, make_face, revolve

# The axis. Source: circle fits to the outer loop, z -52.1 to -47.1 (centre
# x 564.0624 to 564.0638, y 128.0000); the socket's octagon is centred on it.
CENTRE_X = 564.063
CENTRE_Y = 128.0
# Source: the downward face at z -52.187 and the top, the upward face at
# z -42.685.
BASE_Z = -52.187
TOP_Z = -42.685
# Source: the outer circle fit, z -52.1 to -47.5: r 5.200.
RADIUS = 5.2
# The round on the top edge: the vertex rings from z -47.187 to -42.685 lie
# on a 4.50 mm circle (radius 4.502 to 4.506 fitted from ring pairs) centred
# 0.7 mm from the axis at z -47.187, tangent to the side there; the topmost
# ring, r 0.70 at z -42.685, bounds the flat on top.
TOP_ROUND = 4.5

# The socket, up from the underside to its roof, the downward face at
# z -47.036. Its section (z -52.1 to -47.1) is an octagon: a 4.4 by 6.596 mm
# rectangle, long in y, with 45° corners cut 0.786 mm along each side
# (vertices at x ±2.200, y ±2.512 and x ±1.414, y ±3.298 from the axis).
SOCKET_ROOF_Z = -47.036
SOCKET_HALF_X = 2.2
SOCKET_HALF_Y = 3.298
SOCKET_CORNER = 0.786


def build():
    # The profile, turned about the axis: the side up to the round, the
    # round, and the flat on top. A surface of revolution rather than a
    # fillet, as for the Bearing Axle Mount's boss.
    height = TOP_Z - BASE_Z
    shoulder = height - TOP_ROUND
    flat = RADIUS - TOP_ROUND
    profile = make_face([
        Line((0, 0), (RADIUS, 0)),
        Line((RADIUS, 0), (RADIUS, shoulder)),
        CenterArc((flat, shoulder), TOP_ROUND, 0, 90),
        Line((flat, height), (0, height)),
        Line((0, height), (0, 0)),
    ])
    tip = Pos(CENTRE_X, CENTRE_Y, BASE_Z) * revolve(Plane.XZ * profile, Axis.Z, 360)

    x, y, c = SOCKET_HALF_X, SOCKET_HALF_Y, SOCKET_CORNER
    octagon = Polygon((x - c, -y), (x, c - y), (x, y - c), (x - c, y), (c - x, y), (-x, y - c), (-x, c - y),
                      (c - x, -y), align=None)
    socket = Pos(CENTRE_X, CENTRE_Y, BASE_Z) * extrude(octagon, SOCKET_ROOF_Z - BASE_Z)
    return tip - socket


PARTS = [
    {
        "name": "Foot Tip",
        "stl": "hardware/aecert-hexapod-v1/stl/Foot Tip.stl",
        "build": build,
    },
]
