"""Coxa Bearing Side: the coxa link's side plate on the bearing side.

The side plate of `_coxa.py` with a window 40 × 20 mm through it, centred on
the hole pair's middle, and a slot 8 mm wide running 7 mm on from the
window's +x end, its four corners rounded r 1; only the ears' holes are
countersunk. Dimensions are measured from `Coxa Bearing Side.stl` with
trimesh: horizontal sections at z 0.05 to 8.25 (line and circle fits to the
section loops). Coordinates are the STL's.
"""

from build123d import Pos, Rectangle, fillet

import _coxa

# The hole pair's middle. Source: circle fits at z 2, the pair's holes at
# (509.947, 166.377) and (509.947, 194.877).
MIDDLE_X = 509.947
MIDDLE_Y = 180.627
# The window (section at z 2): x 489.947 to 529.947, y 170.627 to 190.627,
# square corners.
WINDOW_LENGTH = 40.0
WINDOW_WIDTH = 20.0
# The slot from its +x end (section at z 2): y 176.627 to 184.627, to
# x 536.947; its corners arcs r 1.000, at its end and where it meets the
# window.
SLOT_LENGTH = 7.0
SLOT_WIDTH = 8.0
SLOT_CORNER_RADIUS = 1.0
# The window's chamfer: 1 mm as the plate's (0.95 mm in at z 0.05), which
# takes the slot's r 1 corners where it meets the window down to nothing at
# the faces. OpenCascade's chamfer cannot close an arc to a point, so it is
# cut 0.001 mm short, below the 0.01 mm to which dimensions are measured.
WINDOW_CHAMFER = _coxa.EDGE_CHAMFER - 0.001


def build():
    end = MIDDLE_X + WINDOW_LENGTH / 2
    window = Pos(MIDDLE_X, MIDDLE_Y) * Rectangle(WINDOW_LENGTH, WINDOW_WIDTH)
    window += Pos(end + SLOT_LENGTH / 2, MIDDLE_Y) * Rectangle(SLOT_LENGTH, SLOT_WIDTH)
    corners = [v for v in window.vertices()
               if (abs(v.X - end) < 1e-6 and abs(abs(v.Y - MIDDLE_Y) - SLOT_WIDTH / 2) < 1e-6)
               or abs(v.X - end - SLOT_LENGTH) < 1e-6]
    window = fillet(corners, SLOT_CORNER_RADIUS)
    _, ears = _coxa.holes(MIDDLE_X, MIDDLE_Y)
    return _coxa.plate(MIDDLE_X, MIDDLE_Y, window, ears, WINDOW_CHAMFER)


PARTS = [
    {
        "name": "Coxa Bearing Side",
        "stl": "hardware/aecert-hexapod-v1/stl/Coxa Bearing Side.stl",
        "build": build,
    },
]
