"""Tests of the Aecert source's feet group (the parametric-foot-parts
change's design.md, "Tests").

    uv run --locked --quiet --script hardware/parametric-reference-parts/build.py --test

They discover the real Aecert source, as the frame group's tests do: the
four foot parts declared, a flexure beam width raised and both feet rebuilt,
each foot's exceptions against its STL's chord error, and each foot
exception's amount over the global limit in the source README.
"""

from __future__ import annotations

import math
import unittest

import build
import chord_error
import gates
from discovery import discover
from test_build import AECERT, FEW, failed, quiet

FEET_FILES = {"foot_left.py": "Foot Left", "foot_right.py": "Foot Right", "foot_shaft.py": "Foot Shaft",
              "foot_tip.py": "Foot Tip"}
FLEXURE_RAISE = 0.3  # mm added to one flexure beam width in the retuning test
CHORD_LIMIT = 0.15  # mm: above this chord error a part needs the tessellation exception (../README.md)
CHORD_MARGIN = 0.05  # mm added to the chord error for that exception's limit
FREEFORM = "freeform"  # how a freeform exception's reason starts
# How gates.check names each exception gate in its failures.
GATE_LABELS = {"volume": "volume", "deviation_max": "deviation max", "deviation_p95": "deviation 95th percentile",
               "bbox": "bounding box"}


def tessellation_limit(error: float) -> float:
    """The `deviation_max` limit for a chord error: plus the margin, rounded
    up to 0.01 mm (rounded to 1e-9 first, so that float noise does not add a
    step)."""
    return math.ceil(round((error + CHORD_MARGIN) * 100, 9)) / 100


def amount_over(gate: str, limit: float) -> str:
    """The README's amount for an exception: the limit less the global
    limit, in the gate's unit, followed by "over"."""
    unit = "%" if gate == "volume" else "mm"
    return f"{limit - gates.GLOBAL_LIMITS[gate]:.2f} {unit} over"


class FeetGroup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sources, failures = discover(build.HERE, build.REPO)
        cls.failures = failures
        cls.source = next((s for s in sources if s.name == AECERT), None)

    def feet(self):
        return [p for p in self.source.parts if p.name in FEET_FILES.values()]

    def part(self, name):
        return next(p for p in self.source.parts if p.name == name)

    def test_the_feet_group_is_declared(self):
        self.assertEqual(self.failures, [])
        self.assertIsNotNone(self.source, f"no source {AECERT}")
        declared = {p.file.name: p.name for p in self.source.parts if p.file.name in FEET_FILES}
        for filename, name in FEET_FILES.items():
            self.assertEqual(declared.get(filename), name, f"foot part {name} is not declared in {filename}")
        self.assertEqual(sorted(declared.values()), sorted(FEET_FILES.values()))
        for name in FEET_FILES.values():
            self.assertEqual(self.part(name).stl, build.REPO / "hardware" / AECERT / "stl" / f"{name}.stl")

    def test_a_retuned_flexure_changes_both_feet(self):
        left, right = self.part("Foot Left"), self.part("Foot Right")
        # The module object Foot Left's build reads; Foot Right's build reads
        # it through foot_left, so a change here reaches both feet only if
        # Foot Right is built from the shared model.
        feet = left.build.__globals__["_feet"]
        unchanged = {p.name: p.build().volume for p in (left, right)}
        feet.FLEXURE_BEAM_WIDTH += FLEXURE_RAISE
        try:
            retuned = {p.name: p.build() for p in (left, right)}
        finally:
            feet.FLEXURE_BEAM_WIDTH -= FLEXURE_RAISE
        for p in (left, right):
            with self.subTest(part=p.name):
                self.assertGreater(abs(retuned[p.name].volume - unchanged[p.name]), 1.0,
                                   f"{p.name} did not change with the flexure's beam width")
                report, _ = quiet(gates.check, p.name, retuned[p.name], p.stl, samples=FEW)
                self.assertTrue(failed(report, "volume") or failed(report, "deviation"), report.failures)
        self.assertAlmostEqual(left.build().volume, unchanged["Foot Left"], places=6, msg="the width was not restored")

    def test_exceptions_follow_the_chord_error(self):
        for part in self.feet():
            with self.subTest(part=part.name):
                error = chord_error.measure(part.stl).error
                exceptions = dict(part.exceptions)
                if error > CHORD_LIMIT:
                    self.assertIn("deviation_max", exceptions,
                                  f"{part.name}: chord error {error:.3f} mm needs a deviation_max exception")
                    self.assertAlmostEqual(exceptions["deviation_max"]["limit"], tessellation_limit(error), places=9,
                                           msg=f"{part.name}: deviation_max limit is not chord error + 0.05 mm")
                    del exceptions["deviation_max"]
                for gate, spec in exceptions.items():
                    self.assertTrue(spec["reason"].startswith(FREEFORM),
                                    f"{part.name}: exception {gate} is neither tessellation nor freeform")
                    self.assertTrue(spec["feature"].strip(), f"{part.name}: exception {gate} names no feature")
                    self.assertGreater(spec["limit"], gates.GLOBAL_LIMITS[gate],
                                       f"{part.name}: exception {gate} does not exceed the global limit")
                    others = {g: s for g, s in part.exceptions.items() if g != gate}
                    report, _ = quiet(gates.check, part.name, part.build(), part.stl, others, samples=FEW)
                    self.assertTrue(failed(report, GATE_LABELS[gate]),
                                    f"{part.name}: exception {gate} is declared for a gate that passes without it")

    def test_gate_labels_name_each_exception_gate(self):
        # A part that fails every gate, so the labels above are the ones
        # gates.check gives (a freeform exception's gate must fail without it).
        part = self.part("Foot Tip")
        self.assertEqual(set(GATE_LABELS), set(gates.EXCEPTION_GATES))
        report, _ = quiet(gates.check, part.name, part.build().scale(1.5), part.stl, samples=FEW)
        for gate, label in GATE_LABELS.items():
            with self.subTest(gate=gate):
                self.assertTrue(failed(report, label), report.failures)

    def test_readme_gives_each_exceptions_amount(self):
        rows = [ln for ln in (self.source.path / "README.md").read_text().splitlines() if ln.startswith("| ")]
        for part in self.feet():
            row = next((r for r in rows if r.startswith(f"| {part.name} |")), "")
            for gate, spec in part.exceptions.items():
                with self.subTest(part=part.name, gate=gate):
                    self.assertIn(amount_over(gate, spec["limit"]), row,
                                  f"{part.name}: README row gives no amount over the global limit for {gate}")


if __name__ == "__main__":
    unittest.main()
