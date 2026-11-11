"""Coxa Horn Side: the coxa link's side plate on the coxa servo's horn side.

The side plate of `_coxa.py` with a rectangular window 57.3 × 20 mm through
it, centred on the hole pair's middle, and all four holes countersunk at the
upper face. Dimensions are measured from `Coxa Horn Side.stl` with trimesh:
horizontal sections at z 0.05 to 8.25 (line and circle fits to the section
loops). Coordinates are the STL's.
"""

from build123d import Pos, Rectangle

import _coxa

# The hole pair's middle. Source: circle fits at z 2, the pair's holes at
# (510.685, 117.019) and (510.685, 145.519).
MIDDLE_X = 510.685
MIDDLE_Y = 131.269
# The window (section at z 2): x 482.035 to 539.335, y 121.269 to 141.269,
# square corners.
WINDOW_LENGTH = 57.3
WINDOW_WIDTH = 20.0


def build():
    window = Pos(MIDDLE_X, MIDDLE_Y) * Rectangle(WINDOW_LENGTH, WINDOW_WIDTH)
    pair, ears = _coxa.holes(MIDDLE_X, MIDDLE_Y)
    return _coxa.plate(MIDDLE_X, MIDDLE_Y, window, pair + ears)


PARTS = [
    {
        "name": "Coxa Horn Side",
        "stl": "hardware/aecert-hexapod-v1/stl/Coxa Horn Side.stl",
        "build": build,
    },
]
