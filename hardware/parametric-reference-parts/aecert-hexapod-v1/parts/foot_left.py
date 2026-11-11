"""Foot Left: the foot half, as `_feet` models it (its docstring gives the
part and how it was measured)."""

import _feet


def build():
    return _feet.foot_half()


PARTS = [
    {
        "name": "Foot Left",
        "stl": "hardware/aecert-hexapod-v1/stl/Foot Left.stl",
        "build": build,
    },
]
