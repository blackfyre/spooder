"""Bearing Axle Mount: the plate that screws to a frame arm (or a coxa link)
and carries a bearing's axle.

A 9.4 mm plate with two screw tabs on one side and the axle boss 9.90 mm off
the tabs' screw line, on the side where the plate reaches furthest. The boss
is a 7.9 mm cylinder with a 1 mm fillet at its foot, a blind 4.4 mm hole from
the top, and an octagonal key at its top. Dimensions are measured from
`Bearing Axle Mount.stl` with trimesh: horizontal sections at z 0.01 to 19.0
(line and circle fits to the section loops, to 0.002 mm) and the levels of
its horizontal faces. Coordinates are the STL's.
"""

from build123d import (Align, Axis, CenterArc, Cylinder, Line, Plane, Polygon, Pos, Rot, chamfer, extrude, fillet,
                       make_face, revolve)

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)

# The plate's outline at mid-height (section at z 5): x 345.966 to 380.966,
# y 116.388 to 153.388; the two tabs, each 8.5 mm wide (y), reach to x
# 345.966, the recess between them to x 356.316.
PLATE_X_MIN = 345.966
PLATE_X_MAX = 380.966
PLATE_Y_MIN = 116.388
PLATE_Y_MAX = 153.388
TAB_WIDTH = 8.5
RECESS_X = 356.316
# Source: the downward face at z 0 and the upward face at z 9.4.
PLATE_THICKNESS = 9.4
# Corner radii in the section at z 5: the tab ends (and the outer corners
# they meet) 4 mm, the recess's inner corners 4 mm, the far corners 7 mm.
TAB_CORNER_RADIUS = 4.0
RECESS_CORNER_RADIUS = 4.0
FAR_CORNER_RADIUS = 7.0
# 45° chamfers round the plate's bottom and top edges: the outline is 0.99 mm
# in at z 0.01 and 0.5 mm in at z 0.5.
EDGE_CHAMFER = 1.0

# The screw holes, one in each tab's middle, 28.5 mm apart (circle fits,
# z 5): radius 1.6, straight through, opening in a 45° countersink to radius
# 2.75 at the top (sections at z 8.4 to 9.38, radius = z - 6.65).
SCREW_X = 352.466
SCREW_Y = (120.638, 149.138)
SCREW_RADIUS = 1.6
COUNTERSINK_RADIUS = 2.75

# The axle boss, 9.90 mm from the screw line (circle fits, z 11 to 19).
AXLE_X = SCREW_X + 9.90
AXLE_Y = 134.888
BOSS_RADIUS = 3.95
# Source: the upward faces at z 17.6 (the boss's top round its key) and 19.1
# (the key's top).
BOSS_TOP_Z = 17.6
KEY_TOP_Z = 19.1
# The fillet at the boss's foot: radius 4.642 at z 9.45, 4.152 at 9.8,
# 3.972 at 10.2, a 1 mm round.
BOSS_FILLET = 1.0
# The key (sections at z 17.8 and 19.0): a square 5.729 mm across, turned
# 45°, its corners cut by an axis-aligned square 6.686 mm across.
KEY_DIAMOND = 5.729
KEY_SQUARE = 6.686
# The blind hole from the top: radius 2.2 (section at z 13.5), its floor the
# upward face at z 13.1.
AXLE_HOLE_RADIUS = 2.2
AXLE_HOLE_FLOOR_Z = 13.1


def outline():
    y0, y1 = PLATE_Y_MIN + TAB_WIDTH, PLATE_Y_MAX - TAB_WIDTH
    points = [(PLATE_X_MIN, PLATE_Y_MIN), (PLATE_X_MAX, PLATE_Y_MIN), (PLATE_X_MAX, PLATE_Y_MAX),
              (PLATE_X_MIN, PLATE_Y_MAX), (PLATE_X_MIN, y1), (RECESS_X, y1), (RECESS_X, y0), (PLATE_X_MIN, y0)]
    face = Polygon(*points, align=None)

    def at(x, y):
        return [v for v in face.vertices() if abs(v.X - x) < 1e-6 and abs(v.Y - y) < 1e-6]

    face = fillet(at(PLATE_X_MAX, PLATE_Y_MIN) + at(PLATE_X_MAX, PLATE_Y_MAX), FAR_CORNER_RADIUS)
    face = fillet([v for v in face.vertices() if abs(v.X - PLATE_X_MIN) < 1e-6], TAB_CORNER_RADIUS)
    return fillet([v for v in face.vertices() if abs(v.X - RECESS_X) < 1e-6], RECESS_CORNER_RADIUS)


def build():
    plate = extrude(outline(), PLATE_THICKNESS)
    plate = chamfer(plate.edges().group_by(Axis.Z)[0] + plate.edges().group_by(Axis.Z)[-1], EDGE_CHAMFER)
    for y in SCREW_Y:
        plate -= Pos(SCREW_X, y, 0) * Cylinder(SCREW_RADIUS, PLATE_THICKNESS, align=BOTTOM)
    rims = [e for e in plate.edges().filter_by_position(Axis.Z, PLATE_THICKNESS - 1e-6, PLATE_THICKNESS + 1e-6)
            if e.geom_type.name == "CIRCLE" and abs(e.radius - SCREW_RADIUS) < 1e-6]
    plate = chamfer(rims, COUNTERSINK_RADIUS - SCREW_RADIUS)
    boss = Pos(AXLE_X, AXLE_Y, 0) * Cylinder(BOSS_RADIUS, BOSS_TOP_Z, align=BOTTOM)
    # The fillet at the boss's foot, turned about the axle as a ring rather
    # than with a fillet operation: the fillet's torus comes out of the STEP
    # as an open shell in solidcraft-cli 0.30.1, a surface of revolution does
    # not.
    r, f = BOSS_RADIUS, BOSS_FILLET
    profile = make_face([Line((r, 0), (r + f, 0)), CenterArc((r + f, f), f, 270, -90), Line((r, f), (r, 0))])
    ring = Pos(AXLE_X, AXLE_Y, PLATE_THICKNESS) * revolve(Plane.XZ * profile, Axis.Z, 360)
    part = plate + boss + ring
    key_height = KEY_TOP_Z - BOSS_TOP_Z
    diamond = Pos(AXLE_X, AXLE_Y, BOSS_TOP_Z) * Rot(0, 0, 45) * extrude(
        Polygon((0, 0), (KEY_DIAMOND, 0), (KEY_DIAMOND, KEY_DIAMOND), (0, KEY_DIAMOND), align=Align.CENTER),
        key_height)
    square = Pos(AXLE_X, AXLE_Y, BOSS_TOP_Z) * extrude(
        Polygon((0, 0), (KEY_SQUARE, 0), (KEY_SQUARE, KEY_SQUARE), (0, KEY_SQUARE), align=Align.CENTER),
        key_height)
    part = part + (diamond & square)
    return part - Pos(AXLE_X, AXLE_Y, AXLE_HOLE_FLOOR_Z) * Cylinder(AXLE_HOLE_RADIUS, KEY_TOP_Z - AXLE_HOLE_FLOOR_Z,
                                                                    align=BOTTOM)


PARTS = [
    {
        "name": "Bearing Axle Mount",
        "stl": "hardware/aecert-hexapod-v1/stl/Bearing Axle Mount.stl",
        "build": build,
    },
]
