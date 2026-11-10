"""Hubcap: the round cap over a coxa or femur horn screw.

A 26 mm disc, chamfered round its top edge, with a square pocket in its
underside and a countersunk screw hole through its centre. Dimensions are
measured from `Hubcap.stl` with trimesh: horizontal sections at z 45.3 to
48.95 (circle fits to the section loops, to 0.001 mm), and the levels of its
horizontal faces. Coordinates are the STL's.
"""

from build123d import Align, Axis, Box, Cylinder, Pos, Rot, chamfer

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The disc's axis and its underside. Source: circle fits, z 45.3 to 48.95; the
# downward face at z 45.225.
CENTRE_X = 159.037
CENTRE_Y = 194.727
BASE_Z = 45.225
# Source: the upward face at z 48.975, less the base.
THICKNESS = 3.75
# Source: the outer circle fit below the chamfer, z 45.3 to 47.6.
RADIUS = 13.0
# A 45° chamfer: the radius falls from 13.0 at z 47.975 to 12.0 at the top
# (sections at z 48.3 to 48.95, radius = 13 - (z - 47.975)).
TOP_CHAMFER = 1.0
# The square pocket in the underside, its corners on the x and y axes through
# the centre (section at z 46: corners 4.262 mm from the centre). Its roof is
# the downward face at z 47.475.
POCKET_SIDE = 6.027
POCKET_DEPTH = 2.25
# The screw hole: radius 1.6 above the pocket (section at z 47.6), opening in
# a 45° countersink to radius 2.75 at the top (sections at z 48.3 to 48.95,
# radius = z - 46.225).
HOLE_RADIUS = 1.6
COUNTERSINK_RADIUS = 2.75


def build():
    top = BASE_Z + THICKNESS
    disc = Pos(CENTRE_X, CENTRE_Y, BASE_Z) * Cylinder(RADIUS, THICKNESS, align=BOTTOM)
    disc = chamfer(disc.edges().filter_by(Axis.Z, reverse=True).sort_by(Axis.Z)[-1], TOP_CHAMFER)
    pocket = Pos(CENTRE_X, CENTRE_Y, BASE_Z) * Rot(0, 0, 45) * Box(POCKET_SIDE, POCKET_SIDE, POCKET_DEPTH,
                                                                    align=BOTTOM)
    hole = Pos(CENTRE_X, CENTRE_Y, BASE_Z) * Cylinder(HOLE_RADIUS, THICKNESS, align=BOTTOM)
    part = disc - pocket - hole
    rim = part.edges().filter_by(Axis.Z, reverse=True).filter_by_position(Axis.Z, top - 1e-6, top + 1e-6)
    rim = [e for e in rim if abs(e.radius - HOLE_RADIUS) < 1e-6]
    return chamfer(rim, COUNTERSINK_RADIUS - HOLE_RADIUS)


PARTS = [
    {
        "name": "Hubcap",
        "stl": "hardware/aecert-hexapod-v1/stl/Hubcap.stl",
        "build": build,
    },
]
