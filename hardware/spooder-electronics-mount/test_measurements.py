"""The measurements and the README's "Measure before printing" table agree.

    cd hardware/spooder-electronics-mount && python3 -m unittest -v test_measurements

Standard library only. Checks that:
- every top-level constant in measurements.py has a source in a comment, on
  its line or in the comment block above it (or above its group);
- the constants whose source is "nominal", or that say they are assumed,
  are exactly MEASURE_BEFORE_PRINTING;
- each of them is read by layout.py or checks.py, or listed in REFERENCE_ONLY;
- the README's table lists each of them once, with a source, and lists
  nothing else;
- its measured column is either "not yet", with the nominal value still in
  the constant, or the measured value and how it was measured, with the
  constant changed to it.
"""

import ast
import re
import unittest
from pathlib import Path

import measurements

HERE = Path(__file__).resolve().parent
SOURCE = (HERE / "measurements.py").read_text(encoding="utf-8")
README = (HERE / "README.md").read_text(encoding="utf-8")


def constants_with_sources() -> dict[str, str]:
    """Each top-level UPPER_CASE constant and the comment text that sources it:
    a comment on its own lines, else the nearest comment block above it,
    skipping the constants of its group."""
    lines = SOURCE.splitlines()
    tree = ast.parse(SOURCE)
    spans = [(n.lineno, n.end_lineno) for n in tree.body if isinstance(n, ast.Assign)]
    in_assign = {i for a, b in spans for i in range(a, b + 1)}
    found = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        names = [t.id for t in node.targets if isinstance(t, ast.Name) and t.id.isupper()]
        if not names:
            continue
        own = " ".join(
            lines[i - 1].split("#", 1)[1] for i in range(node.lineno, node.end_lineno + 1) if "#" in lines[i - 1]
        )
        block = []
        i = node.lineno - 1
        while i >= 1 and i in in_assign:
            i -= 1
        while i >= 1 and lines[i - 1].lstrip().startswith("#"):
            block.insert(0, lines[i - 1].lstrip()[1:].strip())
            i -= 1
        text = own if "Source:" in own else " ".join(block)
        for name in names:
            found[name] = text
    return found


def readme_table() -> list[list[str]]:
    """The rows of the table under the "Measure before printing" heading."""
    section = README.split("## Measure before printing", 1)
    if len(section) < 2:
        return []
    body = section[1].split("\n## ", 1)[0]
    rows = [r for r in body.splitlines() if r.startswith("|")]
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
    return [c for c in cells[2:]]  # past the header and the rule


class Sources(unittest.TestCase):
    def test_every_constant_has_a_source(self):
        constants = constants_with_sources()
        self.assertGreater(len(constants), 40)
        for name, text in constants.items():
            with self.subTest(name=name):
                self.assertIn("Source:", text, f"{name} has no source comment")

    def test_nominal_constants_are_the_ones_to_measure(self):
        nominal = {n for n, t in constants_with_sources().items() if re.search(r"Source: nominal|assumed", t)}
        self.assertEqual(nominal, set(measurements.MEASURE_BEFORE_PRINTING))
        self.assertEqual(len(measurements.MEASURE_BEFORE_PRINTING), len(set(measurements.MEASURE_BEFORE_PRINTING)))


class Used(unittest.TestCase):
    def test_every_value_to_measure_reaches_the_build(self):
        # A measured value the build never reads would change nothing when
        # recorded: each one is read by layout.py (sizing a part, or checked by
        # measurement_failures()) or checks.py, or recorded as reference only.
        layout = "".join((HERE / f).read_text(encoding="utf-8") for f in ("layout.py", "checks.py"))
        self.assertLessEqual(set(measurements.REFERENCE_ONLY), set(measurements.MEASURE_BEFORE_PRINTING))
        for name in measurements.MEASURE_BEFORE_PRINTING:
            with self.subTest(name=name):
                used = re.search(rf"\bM\.{name}\b", layout) is not None
                self.assertTrue(used or name in measurements.REFERENCE_ONLY, f"the build never reads {name}")
                self.assertFalse(used and name in measurements.REFERENCE_ONLY, f"{name} is read, not reference only")


class ReadmeTable(unittest.TestCase):
    def test_table_lists_each_value_to_measure(self):
        rows = readme_table()
        self.assertTrue(rows, "README has no 'Measure before printing' table")
        names = []
        for cells in rows:
            with self.subTest(row=cells[0] if cells else ""):
                self.assertEqual(len(cells), 4, "columns: constant, nominal value, source, measured")
                match = re.fullmatch(r"`([A-Z0-9_]+)`", cells[0])
                self.assertIsNotNone(match, f"first cell is a constant name in backticks: {cells[0]}")
                name = match.group(1)
                names.append(name)
                self.assertTrue(hasattr(measurements, name), f"{name} is not in measurements.py")
                value = re.fullmatch(r"`(.+)`", cells[1])
                self.assertIsNotNone(value, f"{name}: nominal value in backticks")
                nominal = ast.literal_eval(value.group(1))
                self.assertTrue(cells[2], f"{name}: no source")
                if cells[3] == "not yet":
                    self.assertEqual(getattr(measurements, name), nominal, f"{name}: constant is not the nominal value")
                else:
                    measured = re.fullmatch(r"`(.+)` \((.+)\)", cells[3])
                    self.assertIsNotNone(measured, f"{name}: measured cell is 'not yet' or `value` (how and when)")
                    self.assertEqual(getattr(measurements, name), ast.literal_eval(measured.group(1)),
                                     f"{name}: constant is not the measured value")
        self.assertEqual(sorted(names), sorted(measurements.MEASURE_BEFORE_PRINTING))
        self.assertEqual(len(names), len(set(names)), "a constant is listed twice")


if __name__ == "__main__":
    unittest.main()
