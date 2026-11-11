"""Femur Bearing Side: the femur side plate opposite Femur Horn Side, holding
the bearings on which the femur and tibia turn.

The shared plate (`_femur.py`), its slots of the other hand, with a bearing
pocket at each end: an r 11.2 pocket from z 1.35 to the top, an r 1 fillet
round its floor, and an r 9.0 hole through the floor, its top edge rounded
r 1. Dimensions are measured from `Femur Bearing Side.stl` with trimesh
(horizontal sections at z 0.02 to 8.34, circle fits to the section loops).
Coordinates are the STL's.
"""

import _femur
from build123d import Axis, CenterArc, Line, Plane, Pos, make_face, revolve

# The first (-x) bore's axis. Source: circle fits to the pocket, the hole and
# the outline's end arc, z 0.02 to 8.34. The second is _femur.AXIS_TO_AXIS
# along x.
BORE_X = 343.226
BORE_Y = 205.315
# The slots' 45° sides run forward: x rises 1 mm per mm of y (section at
# z 8.3: from (391.361, 196.365) to (409.261, 214.265)).
HAND = 1
# The recess's floor: the upward face at z 5.35.
RECESS_FLOOR_Z = 5.35

# The bearing pocket: r 11.2 (fits r 11.1964 to 11.2000, z 2.4 to 8.34) down
# to its floor at z 1.35 (the upward face). The hole through the floor: r 9.0
# (fits r 8.9965 to 9.0026, z 0.02 to 0.4).
POCKET_RADIUS = 11.2
POCKET_FLOOR_Z = 1.35
HOLE_RADIUS = 9.0
# Both edges of the floor are rounded r 1: the hole's widens along an arc
# centred at r 10.0, z 0.35 (fits r 9.1081 at z 0.8, 9.4755 at z 1.2), the
# pocket's narrows along one centred at r 10.2, z 2.35 (r 10.5012 at z 1.4,
# 11.0278 at z 1.8), leaving the floor flat from r 10.0 to 10.2.
FLOOR_ROUND = 1.0


def bearing_bore():
    """One bearing bore's cutter, on the z axis: its profile in (r, z),
    revolved. (OpenCascade's fillet of these edges gives a solid that
    solidcraft reads as an open shell; the revolved profile does not.)"""
    r0, r1, z0, r = HOLE_RADIUS, POCKET_RADIUS, POCKET_FLOOR_Z, FLOOR_ROUND
    below, above = -1.0, _femur.THICKNESS + 1.0
    profile = make_face([
        Line((0, below), (r0, below)),
        Line((r0, below), (r0, z0 - r)),
        CenterArc((r0 + r, z0 - r), r, 180, -90),  # the hole's round
        Line((r0 + r, z0), (r1 - r, z0)),  # the flat floor
        CenterArc((r1 - r, z0 + r), r, 270, 90),  # the pocket's fillet
        Line((r1, z0 + r), (r1, above)),
        Line((r1, above), (0, above)),
        Line((0, above), (0, below)),
    ])
    return revolve(Plane.XZ * profile, Axis.Z, 360)


def build():
    body = _femur.plate(BORE_X, BORE_Y, HAND, RECESS_FLOOR_Z)
    for x, y in _femur.bore_centres(BORE_X, BORE_Y):
        body -= Pos(x, y, 0) * bearing_bore()
    return body


PARTS = [
    {
        "name": "Femur Bearing Side",
        "stl": "hardware/aecert-hexapod-v1/stl/Femur Bearing Side.stl",
        "build": build,
    },
]
