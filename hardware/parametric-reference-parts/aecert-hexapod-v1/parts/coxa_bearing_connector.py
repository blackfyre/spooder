"""Coxa Bearing Connector: the coxa link's connector on the coxa bearing.

The ring and tongue of `_coxa.py`, turned 180° so that the tongue runs along
-x, with the bearing's seat in the ring: a bore r 11.2 down to a 0.9 mm floor,
its rim rounded r 1 and its floor's edge filleted r 0.3, and through the floor
a hole r 9, chamfered at the underside and rounded where it meets the seat's
floor. Dimensions are measured from `Coxa Bearing Connector.stl` with trimesh:
horizontal sections at z 36.92 to 44.92 (circle fits to the section loops,
every 0.1 mm), the levels of its horizontal faces and a vertical section
along the tongue. Coordinates are the STL's.
"""

from build123d import Pos, Rot

import _coxa

# The coxa axis and the ring's underside. Source: circle fits to the bore
# (z 38.2 to 43.9), centred on (309.467, 94.013); the downward face at
# z 36.908.
AXIS_X = 309.467
AXIS_Y = 94.013
UNDERSIDE_Z = 36.908
# The tongue runs along -x: the connector's frame turned 180° about the axis.
TURN = 180.0

# The bearing's seat: r 11.200 from its floor, the upward face at z 37.808,
# 0.9 mm above the underside, to the ring's upper face. Its rim is rounded
# r 1 (r 11.373 at z 44.52, 11.934 at 44.92) and its floor's edge filleted
# r 0.3 (r 10.977 at z 37.82, 11.132 at 37.92).
SEAT_RADIUS = 11.2
FLOOR_THICKNESS = 0.9
SEAT_RIM_ROUND = 1.0
SEAT_FLOOR_ROUND = 0.3
# The hole through the floor: r 9.0 (z 37.42 to 37.52), opening at 45° to
# r 9.5 at the underside (r 9.488 at z 36.92 to 9.088 at 37.32), its edge
# with the seat's floor rounded r 0.3 (r 9.021 at z 37.62, 9.088 at 37.72).
FLOOR_HOLE_RADIUS = 9.0
FLOOR_HOLE_CHAMFER = 0.5
FLOOR_HOLE_ROUND = 0.3


def build():
    part = _coxa.connector_body()
    r, h, c, top = SEAT_RADIUS, FLOOR_HOLE_RADIUS, FLOOR_HOLE_CHAMFER, _coxa.RING_HEIGHT
    rim, f, g, floor = SEAT_RIM_ROUND, SEAT_FLOOR_ROUND, FLOOR_HOLE_ROUND, FLOOR_THICKNESS
    # The seat and the floor's hole in one profile, the chamfer run on below
    # the underside.
    part -= _coxa.turned([(0, -1), (h + c + 1, -1), (h, c), (h, floor - g), ((h + g, floor), (h + g, floor - g)),
                          (r - f, floor), ((r, floor + f), (r - f, floor + f)), (r, top - rim),
                          ((r + rim, top), (r + rim, top - rim)), (r + rim, top + 1), (0, top + 1)])
    return Pos(AXIS_X, AXIS_Y, UNDERSIDE_Z) * Rot(0, 0, TURN) * part


PARTS = [
    {
        "name": "Coxa Bearing Connector",
        "stl": "hardware/aecert-hexapod-v1/stl/Coxa Bearing Connector.stl",
        "build": build,
    },
]
