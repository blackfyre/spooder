"""Tests of the Aecert source's femur group (OpenSpec change
parametric-femur-parts, task 1.3).

    uv run --locked --quiet --script hardware/parametric-reference-parts/build.py --test

The femur group's three parts are declared in their own files; each side
plate's two bore axes, found from the built solid's cylindrical faces (not
from the constants that place them), lie the femur's axis-to-axis length
apart; and rebuilt with that length raised, both sides' axes move with it,
so a side that does not place its second bore from `_femur.AXIS_TO_AXIS`
fails.
"""

from __future__ import annotations

import math
import unittest
from pathlib import Path

from build123d import GeomType

import build
from discovery import discover

HERE = Path(__file__).resolve().parent
AECERT = "aecert-hexapod-v1"
FEMUR_FILES = {"femur_horn_side.py": "Femur Horn Side", "femur_bearing_side.py": "Femur Bearing Side",
               "femur_support.py": "Femur Support"}
# Each side plate's bore, by the name of its radius in the part file: the
# horn recess, and the bearing pocket.
BORE_RADIUS = {"Femur Horn Side": "HORN_BORE_RADIUS", "Femur Bearing Side": "POCKET_RADIUS"}
TOLERANCE = 0.01  # mm, between the bore axes' spacing and AXIS_TO_AXIS
RAISE = 1.0  # mm added to AXIS_TO_AXIS in the rebuild


def load_aecert():
    """The Aecert source, discovered afresh: new module objects for its part
    files and their helpers, so a constant changed on them changes nothing
    else."""
    sources, failures = discover(HERE, build.REPO)
    return next((s for s in sources if s.name == AECERT), None), failures


def bore_axes(solid, radius: float) -> list[tuple[float, float]]:
    """The distinct vertical axes, in plan, of the solid's cylindrical faces
    of `radius`."""
    axes: list[tuple[float, float]] = []
    for face in solid.faces():
        if face.geom_type != GeomType.CYLINDER or abs(face.radius - radius) > 1e-6:
            continue
        axis = face.axis_of_rotation
        if abs(abs(axis.direction.Z) - 1) > 1e-9:
            continue
        xy = (axis.position.X, axis.position.Y)
        if not any(math.dist(xy, a) < 1e-6 for a in axes):
            axes.append(xy)
    return axes


class FemurGroup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source, cls.failures = load_aecert()

    def part(self, name, source=None):
        return next(p for p in (source or self.source).parts if p.name == name)

    def spacing(self, part) -> float:
        module = part.build.__globals__
        axes = bore_axes(part.build(), module[BORE_RADIUS[part.name]])
        self.assertEqual(len(axes), 2, f"{part.name}: expected two bore axes, found {axes}")
        (x0, y0), (x1, y1) = sorted(axes)
        self.assertAlmostEqual(y0, y1, delta=1e-6, msg=f"{part.name}: the bore axes are not on one line along x")
        return math.dist((x0, y0), (x1, y1))

    def test_the_femur_group_is_declared(self):
        self.assertEqual(self.failures, [])
        self.assertIsNotNone(self.source, f"no source {AECERT}")
        declared = {p.file.name: p.name for p in self.source.parts if p.file.name in FEMUR_FILES}
        for filename, name in FEMUR_FILES.items():
            self.assertEqual(declared.get(filename), name, f"femur part {name} is not declared in {filename}")
        self.assertEqual(sorted(declared.values()), sorted(FEMUR_FILES.values()))
        for name in FEMUR_FILES.values():
            self.assertEqual(self.part(name).stl, build.REPO / "hardware" / AECERT / "stl" / f"{name}.stl")

    def test_bore_axes_lie_axis_to_axis_apart(self):
        for name in BORE_RADIUS:
            part = self.part(name)
            expected = part.build.__globals__["_femur"].AXIS_TO_AXIS
            with self.subTest(part=name):
                self.assertAlmostEqual(self.spacing(part), expected, delta=TOLERANCE)

    def test_both_sides_follow_a_changed_axis_to_axis(self):
        source, failures = load_aecert()
        self.assertEqual(failures, [])
        parts = [self.part(name, source) for name in BORE_RADIUS]
        helpers = {id(m): m for m in (p.build.__globals__["_femur"] for p in parts)}.values()
        original = {id(m): m.AXIS_TO_AXIS for m in helpers}
        for m in helpers:
            m.AXIS_TO_AXIS += RAISE
        try:
            for part in parts:
                with self.subTest(part=part.name):
                    raised = original[id(part.build.__globals__["_femur"])] + RAISE
                    self.assertAlmostEqual(self.spacing(part), raised, delta=TOLERANCE,
                                           msg=f"{part.name} does not place its bores from _femur.AXIS_TO_AXIS")
        finally:
            for m in helpers:
                m.AXIS_TO_AXIS = original[id(m)]


if __name__ == "__main__":
    unittest.main()
