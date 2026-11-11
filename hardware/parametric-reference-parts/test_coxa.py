"""Tests of the Aecert source's coxa group (change parametric-coxa-parts,
tasks 1.4): the four coxa parts are declared, and the coxa link geometry that
`hardware/aecert-hexapod-v1/README.md` records is measured on the built
solids and compared with the coxa comparison table in the source README
(`aecert-hexapod-v1/README.md`).

    uv run --locked --quiet --script hardware/parametric-reference-parts/build.py --test

Each value is found from the solid's faces, not from the part files'
constants: the connectors' coxa axis is the axis of the seat (the widest
closed vertical cylindrical face), the insert hole the closed cylindrical face
with a level axis, the ring's faces the largest downward and upward planar
faces; on a side plate, the four screw holes are the closed vertical
cylindrical faces of the smallest radius, the ears the two holes furthest
apart, the outline the planar faces at its ends and the ears' half discs.
"""

from __future__ import annotations

import itertools
import math
import unittest

import build
from discovery import discover

HERE = build.HERE
AECERT = "aecert-hexapod-v1"
COXA_FILES = {"coxa_horn_connector.py": "Coxa Horn Connector", "coxa_horn_side.py": "Coxa Horn Side",
              "coxa_bearing_connector.py": "Coxa Bearing Connector", "coxa_bearing_side.py": "Coxa Bearing Side"}
CONNECTORS = ("Coxa Horn Connector", "Coxa Bearing Connector")
SIDES = ("Coxa Horn Side", "Coxa Bearing Side")

# The coxa link geometry as recorded. Source: hardware/aecert-hexapod-v1/
# README.md, "The coxa link is two vertical side plates (68.25 × 48 mm) ...
# their own 28.5 mm hole pair, ... ears 30.10 mm either side of that pair's
# middle ... Each tongue's insert hole lies at its ring's mid-height (4.025 mm
# from either face of the 8.05 mm ring), 21.5 mm from the coxa axis".
RECORDED_INSERT_FROM_AXIS = 21.5
RECORDED_RING_HEIGHT = 8.05
RECORDED_INSERT_HEIGHT = 4.025
RECORDED_EAR_FROM_MIDDLE = 30.10
RECORDED_HOLE_PAIR_SPACING = 28.5
RECORDED_PLATE_LENGTH = 68.25
RECORDED_PLATE_WIDTH = 48.0
# The phrases that hold them there, so that a reworded record fails here too.
RECORDED_PHRASES = ("21.5 mm from the coxa axis", "8.05 mm ring", "4.025 mm from either face",
                    "ears 30.10 mm either side", "28.5 mm hole pair", "(68.25 × 48 mm)")

# The comparison table's rows: the value and, per kind of part, its record.
CONNECTOR_VALUES = {"Insert hole from the coxa axis": RECORDED_INSERT_FROM_AXIS,
                    "Ring height": RECORDED_RING_HEIGHT,
                    "Insert hole above the underside": RECORDED_INSERT_HEIGHT}
SIDE_VALUES = {"Each ear from the hole pair's middle": RECORDED_EAR_FROM_MIDDLE,
               "Hole pair spacing": RECORDED_HOLE_PAIR_SPACING,
               "Side plate length": RECORDED_PLATE_LENGTH,
               "Side plate width": RECORDED_PLATE_WIDTH}
TABLE_HEADER = "| Value | Part | Recorded (mm) | Model (mm) | Disagreement |"
MATCH = 0.01  # mm: a measured value against the table's model value
DISAGREEMENT = 0.05  # mm: the model against the record (design: the 95th percentile limit)
MARKER = "`disagrees`"


def comparison_table() -> dict[tuple[str, str], tuple[float, float, str]]:
    """The source README's coxa comparison: (value, part) -> (recorded, model,
    disagreement cell)."""
    lines = (HERE / AECERT / "README.md").read_text().splitlines()
    start = lines.index(TABLE_HEADER) + 2
    rows = {}
    for line in itertools.takewhile(lambda ln: ln.startswith("| "), lines[start:]):
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        rows[(cells[0], cells[1])] = (float(cells[2]), float(cells[3]), cells[4])
    return rows


# --- measuring the solids -------------------------------------------------------


def closed_cylinders(solid):
    """The cylindrical faces bounded by full circles (holes and bores), as
    (axis, radius)."""
    out = []
    for f in solid.faces():
        if f.geom_type.name == "CYLINDER" and any(e.geom_type.name == "CIRCLE" and e.is_closed for e in f.edges()):
            out.append((f.axis_of_rotation, f.radius))
    return out


def vertical(axis) -> bool:
    return abs(abs(axis.direction.Z) - 1) < 1e-9


def level(axis) -> bool:
    return abs(axis.direction.Z) < 1e-9


def line_distance(a, b) -> float:
    """The shortest distance between two lines that are not parallel."""
    n = a.direction.cross(b.direction)
    return abs((b.position - a.position).dot(n)) / n.length


def planar(solid, normal):
    """The planar faces whose outward normal is `normal`."""
    return [f for f in solid.faces()
            if f.geom_type.name == "PLANE" and (f.normal_at() - normal).length < 1e-9]


def measure_connector(solid) -> dict[str, list[float]]:
    from build123d import Vector

    seat = max((c for c in closed_cylinders(solid) if vertical(c[0])), key=lambda c: c[1])[0]
    hole = next(c for c in closed_cylinders(solid) if level(c[0]))[0]
    underside = max(planar(solid, Vector(0, 0, -1)), key=lambda f: f.area).center().Z
    top = max(planar(solid, Vector(0, 0, 1)), key=lambda f: f.area).center().Z
    return {"Insert hole from the coxa axis": [line_distance(seat, hole)],
            "Ring height": [top - underside],
            "Insert hole above the underside": [hole.position.Z - underside]}


def measure_side(solid) -> dict[str, list[float]]:
    from build123d import Vector

    holes = [c for c in closed_cylinders(solid) if vertical(c[0])]
    smallest = min(r for _, r in holes)
    centres = [Vector(a.position.X, a.position.Y, 0) for a, r in holes if abs(r - smallest) < 1e-9]
    assert len(centres) == 4, f"{len(centres)} screw holes"
    ears = max(itertools.combinations(centres, 2), key=lambda p: (p[0] - p[1]).length)
    pair = [c for c in centres if all((c - e).length > 1e-6 for e in ears)]
    middle = (pair[0] + pair[1]) * 0.5
    u = (ears[1] - ears[0]).normalized()
    v = ((ears[0] + ears[1]) * 0.5 - middle).normalized()
    ends = planar(solid, u) + planar(solid, -u)
    length = max(f.center().dot(u) for f in ends) - min(f.center().dot(u) for f in ends)
    discs = [f for f in solid.faces() if f.geom_type.name == "CYLINDER" and vertical(f.axis_of_rotation)
             and any((Vector(f.axis_of_rotation.position.X, f.axis_of_rotation.position.Y, 0) - e).length < 1e-6
                     for e in ears) and f.radius > smallest + 1e-6]
    far = max(Vector(f.axis_of_rotation.position.X, f.axis_of_rotation.position.Y, 0).dot(v) + f.radius
              for f in discs)
    near = min(f.center().dot(v) for f in planar(solid, -v))
    return {"Each ear from the hole pair's middle": [abs((e - middle).dot(u)) for e in ears],
            "Hole pair spacing": [(pair[0] - pair[1]).length],
            "Side plate length": [length],
            "Side plate width": [far - near]}


class CoxaGroup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sources, cls.failures = discover(HERE, build.REPO)
        cls.source = next((s for s in sources if s.name == AECERT), None)

    def part(self, name):
        return next(p for p in self.source.parts if p.name == name)

    def test_the_coxa_group_is_declared(self):
        self.assertEqual(self.failures, [])
        self.assertIsNotNone(self.source, f"no source {AECERT}")
        declared = {p.file.name: p for p in self.source.parts if p.file.name in COXA_FILES}
        for filename, name in COXA_FILES.items():
            with self.subTest(part=name):
                part = declared.get(filename)
                self.assertTrue(part is not None and part.name == name
                                and part.stl == build.REPO / "hardware" / AECERT / "stl" / f"{name}.stl",
                                f"coxa part {name} is not declared in {filename} with its STL")

    def test_the_comparison_matches_the_built_parts(self):
        table = comparison_table()
        expected = {(value, name) for name in CONNECTORS for value in CONNECTOR_VALUES}
        expected |= {(value, name) for name in SIDES for value in SIDE_VALUES}
        self.assertEqual(sorted(table), sorted(expected), "the coxa comparison table's rows")
        for name in CONNECTORS + SIDES:
            measured = (measure_connector if name in CONNECTORS else measure_side)(self.part(name).build())
            for value, values in measured.items():
                model = table[(value, name)][1]
                for got in values:
                    with self.subTest(part=name, value=value):
                        self.assertLessEqual(abs(got - model), MATCH,
                                             f"{name}: {value} is {got:.3f} mm on the solid, {model} in the table")

    def test_the_recorded_column_and_the_disagreements(self):
        reference = (build.REPO / "hardware" / AECERT / "README.md").read_text()
        for phrase in RECORDED_PHRASES:
            self.assertIn(phrase, reference, "the coxa link's record has changed; update the RECORDED_* values")
        recorded = {**CONNECTOR_VALUES, **SIDE_VALUES}
        for (value, name), (got, model, cell) in comparison_table().items():
            with self.subTest(part=name, value=value):
                self.assertEqual(got, recorded[value], f"{name}: {value}'s recorded value")
                disagrees = abs(model - got) > DISAGREEMENT
                self.assertEqual(cell == MARKER, disagrees,
                                 f"{name}: {value} is {model} against {got} recorded; the marker is {cell!r}")
                self.assertIn(cell, ("", MARKER))


if __name__ == "__main__":
    unittest.main()
