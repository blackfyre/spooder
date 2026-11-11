"""Coxa Horn Connector: the coxa link's connector on the coxa servo's horn.

The ring and tongue of `_coxa.py`, the tongue along +x, with the horn's seat
in the ring: a recess r 10.1 down to a 1.95 mm floor, its rim and its floor's
edge rounded r 0.3; through the floor a centre hole (r 1.6, countersunk at
the underside) and the horn's four screw holes on a 6.8 mm radius, each
counterbored from the underside and opening into the recess through a cone.
Dimensions are measured from `Coxa Horn Connector.stl` with trimesh:
horizontal sections at z -43.33 to -35.33 (circle fits to the section loops,
every 0.04 to 0.1 mm), the levels of its horizontal faces and vertical
sections at y 0 to 15. Coordinates are the STL's.
"""

from build123d import Align, Axis, Cone, Cylinder, Pos, chamfer

import _coxa

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The coxa axis and the ring's underside. Source: circle fits to the recess
# (z -40 to -36) and the screw holes' pattern, centred on (96.5, 0); the
# downward face at z -43.35.
AXIS_X = 96.5
AXIS_Y = 0.0
UNDERSIDE_Z = -43.35

# The horn's recess: r 10.100 from its floor, the upward face at z -41.4,
# 1.95 mm above the underside, to the ring's upper face. Its rim is rounded
# r 0.3 (r 10.108 at z -35.53, 10.270 at -35.33) and its floor's edge filleted
# r 0.3 (r 10.068 at z -41.23, 9.993 at -41.33).
RECESS_RADIUS = 10.1
FLOOR_THICKNESS = 1.95
RECESS_ROUND = 0.3
# The centre hole: r 1.6 through the floor, opening at 45° to r 2.75 at the
# underside (r 2.730 at z -43.33 to 1.630 at -42.23).
CENTRE_HOLE_RADIUS = 1.6
CENTRE_COUNTERSINK_RADIUS = 2.75
# The horn's screw holes, on the axes 6.8 mm from the axis (circle fits):
# counterbored r 3.025 from the underside to the downward faces at z -42.05,
# then a cone (line fit to r at z -42.04 to -41.44, to 0.001 mm) from r 2.96
# there to r 1.76 at the recess's floor.
SCREW_PATTERN_RADIUS = 6.8
SCREW_COUNTERBORE_RADIUS = 3.025
SCREW_COUNTERBORE_DEPTH = 1.3
SCREW_CONE_BASE_RADIUS = 2.96
SCREW_CONE_TOP_RADIUS = 1.76


def build():
    part = _coxa.connector_body()
    r, f, floor, top = RECESS_RADIUS, RECESS_ROUND, FLOOR_THICKNESS, _coxa.RING_HEIGHT
    part -= _coxa.turned([(0, floor), (r - f, floor), ((r, floor + f), (r - f, floor + f)), (r, top - f),
                          ((r + f, top), (r + f, top - f)), (r + f, top + 1), (0, top + 1)])
    part -= Cylinder(CENTRE_HOLE_RADIUS, FLOOR_THICKNESS, align=BOTTOM)
    hole_rim = [e for e in part.edges().filter_by_position(Axis.Z, -1e-6, 1e-6)
                if e.geom_type.name == "CIRCLE" and abs(e.radius - CENTRE_HOLE_RADIUS) < 1e-6]
    part = chamfer(hole_rim, CENTRE_COUNTERSINK_RADIUS - CENTRE_HOLE_RADIUS)
    cone_height = FLOOR_THICKNESS - SCREW_COUNTERBORE_DEPTH
    for x, y in ((1, 0), (0, 1), (-1, 0), (0, -1)):
        at = (x * SCREW_PATTERN_RADIUS, y * SCREW_PATTERN_RADIUS)
        part -= Pos(*at, 0) * Cylinder(SCREW_COUNTERBORE_RADIUS, SCREW_COUNTERBORE_DEPTH, align=BOTTOM)
        part -= Pos(*at, SCREW_COUNTERBORE_DEPTH) * Cone(SCREW_CONE_BASE_RADIUS, SCREW_CONE_TOP_RADIUS, cone_height,
                                                         align=BOTTOM)
    return Pos(AXIS_X, AXIS_Y, UNDERSIDE_Z) * part


PARTS = [
    {
        "name": "Coxa Horn Connector",
        "stl": "hardware/aecert-hexapod-v1/stl/Coxa Horn Connector.stl",
        "build": build,
    },
]
