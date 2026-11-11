"""Femur Horn Side: the femur side plate that carries the femur and tibia
servos' horns.

The shared plate (`_femur.py`), with a horn bore at each end: an r 10.098
recess from z 2.1 to the top, its floor pierced by a countersunk centre hole
and four countersunk screw holes 6.8 mm off the axis, along x and y.
Dimensions are measured from `Femur Horn Side.stl` with trimesh (horizontal
sections at z 0.05 to 8.3, circle fits to the section loops, to 0.004 mm).
Coordinates are the STL's.
"""

import _femur
from build123d import Align, Cone, Cylinder, Pos

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The first (-x) bore's axis. Source: circle fits to the bore, the outline's
# end arc and the screw pattern, z 0.05 to 8.3. The second is
# _femur.AXIS_TO_AXIS along x.
BORE_X = 344.581
BORE_Y = 171.827
# The slots' 45° sides run back: x falls 1 mm per mm of y (section at z 1.2:
# from (393.667, 179.827) to (409.667, 163.827)).
HAND = -1
# The recess's floor: the upward face at z 5.3.
RECESS_FLOOR_Z = 5.3

# The horn bore. Source: circle fits at z 2.2 to 8.3 (r 10.0965 to 10.0999,
# 168 segments); the bore's floor, the upward face at z 2.1.
HORN_BORE_RADIUS = 10.098
HORN_FLOOR_Z = 2.1
# The four horn screw holes, 6.8 mm off the axis along x and y (circle fits:
# 337.781, 351.381, 165.027, 178.627 round 344.581, 171.827): r 3.023 from
# the bottom face to z 1.3 (fits r 3.0215 to 3.0245), then a cone from r 2.96
# at z 1.3 to r 1.76 at the floor (a line through the fits at z 1.35 to 2.05,
# r 2.8846 to 1.8343).
SCREW_OFFSET = 6.8
SCREW_RADIUS = 3.023
SCREW_HEAD_DEPTH = 1.3
SCREW_CONE_RADIUS = 2.96
SCREW_THROAT_RADIUS = 1.76


def bore(x, y):
    """One horn bore's cutter, at the axis (x, y)."""
    cutter = Pos(x, y, HORN_FLOOR_Z) * Cylinder(HORN_BORE_RADIUS, _femur.THICKNESS - HORN_FLOOR_Z, align=BOTTOM)
    cutter += _femur.countersunk_hole(x, y, HORN_FLOOR_Z + 1)
    for dx, dy in ((SCREW_OFFSET, 0), (-SCREW_OFFSET, 0), (0, SCREW_OFFSET), (0, -SCREW_OFFSET)):
        cutter += Pos(x + dx, y + dy, 0) * Cylinder(SCREW_RADIUS, SCREW_HEAD_DEPTH, align=BOTTOM)
        cutter += Pos(x + dx, y + dy, SCREW_HEAD_DEPTH) * Cone(SCREW_CONE_RADIUS, SCREW_THROAT_RADIUS,
                                                               HORN_FLOOR_Z - SCREW_HEAD_DEPTH, align=BOTTOM)
    return cutter


def build():
    body = _femur.plate(BORE_X, BORE_Y, HAND, RECESS_FLOOR_Z)
    for x, y in _femur.bore_centres(BORE_X, BORE_Y):
        body -= bore(x, y)
    return body


PARTS = [
    {
        "name": "Femur Horn Side",
        "stl": "hardware/aecert-hexapod-v1/stl/Femur Horn Side.stl",
        "build": build,
    },
]
