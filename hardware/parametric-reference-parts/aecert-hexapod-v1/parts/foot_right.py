"""Foot Right: Foot Left's mirror image, moved along y.

Its STL is Foot Left's reflected across x and moved 1.069 mm towards -y,
which a mirror plane cannot express, so this is a part file of its own
rather than the second side of a mirrored pair (the change's design.md,
"Foot Right"): its build mirrors Foot Left's solid and moves it, and has no
model of its own, so a retuned flexure (`_feet`) changes both feet. Both
constants are measured from the two STLs' bounding boxes with trimesh.
"""

from build123d import Plane, Pos

import foot_left

# The mirror plane x = MIRROR_X: midway between Foot Left's x min (104.7645)
# and Foot Right's x max (144.1258), and between Foot Left's x max (121.4789)
# and Foot Right's x min (127.4115), both giving 124.4452.
MIRROR_X = 124.4452
# Foot Right's y min (54.2535) less Foot Left's (55.3226), and its y max
# (102.7422) less Foot Left's (103.8113): both -1.0691. Its z range is Foot
# Left's (0 to 12.45).
RIGHT_OFFSET_Y = -1.0691


def build():
    return Pos(0, RIGHT_OFFSET_Y, 0) * foot_left.build().mirror(Plane.YZ.offset(MIRROR_X))


PARTS = [
    {
        "name": "Foot Right",
        "stl": "hardware/aecert-hexapod-v1/stl/Foot Right.stl",
        "build": build,
    },
]
