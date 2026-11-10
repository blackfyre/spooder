"""Tests of the parametric reference parts' build (tasks 1.1 to 1.4) and of
the Aecert source's frame group (tasks 2.1 to 2.3).

    uv run --locked --quiet --script hardware/parametric-reference-parts/build.py --test

Discovery is tested on part files written into temporary sources; each gate
is shown to fail on a bad synthetic shape and pass on the good one, against
STLs exported from synthetic solids; the STEP export and its folder swap on
temporary sources; and the import checks with the tools hidden, with stubbed
tool output, and (when both are installed) with the real tools. The frame
group's tests discover the real Aecert source: its five parts, a named
dimension changed and rebuilt, and the measurements the frame README states.
"""

from __future__ import annotations

import io
import json
import math
import os
import shutil
import tempfile
import textwrap
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from build123d import Box, Compound, Cylinder, Plane, Pos, Solid, export_step, export_stl

import build
import chord_error
import export
import gates
import import_check
from discovery import discover

HERE = Path(__file__).resolve().parent
FEW = 2000  # area samples per side in the gate tests: enough for the 95th percentile here

# --- shapes ---------------------------------------------------------------


def block(x0=0.0, x1=20.0, y0=0.0, y1=20.0, z0=0.0, z1=10.0):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def bored(x=10.0, y=10.0, r=3.0, **kw):
    """A 20 x 20 x 10 block with a vertical bore."""
    return block(**kw) - Pos(x, y, 5) * Cylinder(r, 30)


def write_stl(shape, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, str(path), tolerance=0.002, angular_tolerance=0.05)
    return path


def quiet(fn, *a, **kw):
    out = io.StringIO()
    with redirect_stdout(out):
        value = fn(*a, **kw)
    return value, out.getvalue()


def failed(report, gate: str) -> bool:
    return any(f": {gate}" in f for f in report.failures)


class Temp(unittest.TestCase):
    """A temporary tooling root (`root`) inside a temporary repository (`repo`)."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        self.root = self.repo / "tooling"
        self.root.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def stl(self, rel: str, shape) -> str:
        write_stl(shape, self.repo / rel)
        return rel

    def part_file(self, source: str, filename: str, body: str) -> Path:
        path = self.root / source / "parts" / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(textwrap.dedent(body))
        return path

    def simple_part(self, source: str, filename: str, name: str, stl: str, shape_expr: str = "Box(10, 10, 10)",
                    extra: str = "") -> Path:
        return self.part_file(source, filename, f"""
            from build123d import Box, Cylinder, Pos
            {extra}
            def build():
                return {shape_expr}

            PARTS = [{{"name": {name!r}, "stl": {stl!r}, "build": build}}]
            """)


# --- 1.1 discovery ----------------------------------------------------------


class Discovery(Temp):
    def setUp(self):
        super().setUp()
        self.cube = self.stl("ref/cube.stl", Box(10, 10, 10))
        self.other = self.stl("ref/other.stl", Box(10, 10, 10))

    def discover(self):
        return discover(self.root, self.repo)

    def test_empty_source_reports_no_parts(self):
        (self.root / "src" / "parts").mkdir(parents=True)
        code, out = quiet(build.main, ["--check"], self.root, self.repo)
        self.assertEqual(code, 0)
        self.assertIn("1 source(s), 0 part(s)", out)
        self.assertIn("No parts.", out)

    def test_finds_a_dropped_part_and_ignores_underscore_files(self):
        self.simple_part("src", "cube.py", "Cube", self.cube)
        # A helper is not discovered, even when it has PARTS of its own.
        self.simple_part("src", "_helper.py", "Helper", self.other)
        sources, failures = self.discover()
        self.assertEqual(failures, [])
        self.assertEqual([p.name for s in sources for p in s.parts], ["Cube"])

    def test_helper_without_parts_is_accepted_and_importable(self):
        self.part_file("src", "_shapes.py", """
            from build123d import Box
            SIDE = 10
            def cube():
                return Box(SIDE, SIDE, SIDE)
            """)
        self.part_file("src", "cube.py", f"""
            import _shapes
            def build():
                return _shapes.cube()
            PARTS = [{{"name": "Cube", "stl": {self.cube!r}, "build": build}}]
            """)
        sources, failures = self.discover()
        self.assertEqual(failures, [])
        self.assertAlmostEqual(sources[0].parts[0].build().volume, 1000.0, places=3)

    def test_duplicate_name_fails(self):
        self.simple_part("src", "a.py", "Cube", self.cube)
        self.simple_part("src", "b.py", "Cube", self.other)
        _, failures = self.discover()
        self.assertTrue(any("'Cube' is also declared" in f for f in failures), failures)

    def test_duplicate_stl_fails(self):
        self.simple_part("src", "a.py", "A", self.cube)
        self.simple_part("other", "b.py", "B", self.cube)
        _, failures = self.discover()
        self.assertTrue(any("STL cube.stl is also declared" in f for f in failures), failures)

    def test_empty_parts_fails(self):
        self.part_file("src", "empty.py", "PARTS = []\n")
        _, failures = self.discover()
        self.assertEqual(failures, ["src/parts/empty.py: declares no part (PARTS missing or empty)"])

    def test_missing_parts_fails(self):
        self.part_file("src", "none.py", "SIDE = 1\n")
        _, failures = self.discover()
        self.assertTrue(any("none.py: declares no part" in f for f in failures), failures)

    def test_three_parts_fail(self):
        third = self.stl("ref/third.stl", Box(1, 1, 1))
        self.part_file("src", "three.py", f"""
            from build123d import Box
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": n, "stl": s, "build": build}}
                     for n, s in (("A", {self.cube!r}), ("B", {self.other!r}), ("C", {third!r}))]
            """)
        _, failures = self.discover()
        self.assertTrue(any("three.py: declares 3 parts" in f for f in failures), failures)

    def test_two_modelled_parts_fail(self):
        self.part_file("src", "two.py", f"""
            from build123d import Box
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": "A", "stl": {self.cube!r}, "build": build}},
                     {{"name": "B", "stl": {self.other!r}, "build": build}}]
            """)
        _, failures = self.discover()
        self.assertTrue(any("second side of a mirrored pair has no build" in f for f in failures), failures)

    def test_pair_second_side_with_build_and_mirror_of_fails(self):
        self.part_file("src", "pair.py", f"""
            from build123d import Box, Plane
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": "Left", "stl": {self.cube!r}, "build": build}},
                     {{"name": "Right", "stl": {self.other!r}, "build": build, "mirror_of": "Left",
                       "plane": Plane.YZ}}]
            """)
        _, failures = self.discover()
        self.assertTrue(any("pair.py: Right: the second side of a mirrored pair has no build" in f for f in failures),
                        failures)

    def test_pair_mirror_of_another_file_fails(self):
        self.simple_part("src", "a.py", "Left", self.cube)
        third = self.stl("ref/third.stl", Box(1, 1, 1))
        self.part_file("src", "pair.py", f"""
            from build123d import Box, Plane
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": "Mine", "stl": {third!r}, "build": build}},
                     {{"name": "Right", "stl": {self.other!r}, "mirror_of": "Left", "plane": Plane.YZ}}]
            """)
        _, failures = self.discover()
        self.assertTrue(any("mirror_of the first side's name ('Mine'), not 'Left'" in f for f in failures), failures)

    def test_valid_pair_is_accepted(self):
        self.part_file("src", "pair.py", f"""
            from build123d import Box, Plane
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": "Left", "stl": {self.cube!r}, "build": build}},
                     {{"name": "Right", "stl": {self.other!r}, "mirror_of": "Left", "plane": Plane.YZ}}]
            """)
        sources, failures = self.discover()
        self.assertEqual(failures, [])
        self.assertEqual([p.name for p in sources[0].parts], ["Left", "Right"])

    def test_mesh_reading_fails_in_part_and_helper(self):
        cases = {
            "imports trimesh": "import trimesh\n",
            "imports pathlib": "import pathlib\n",
            "calls open": "DATA = open('x.stl', 'rb')\n",
            "imports import_stl": "from build123d import import_stl\n",
            "calls b.import_stl": "import build123d as b\nMESH = b.import_stl('x.stl')\n",
        }
        for what, code in cases.items():
            for filename in ("cube.py", "_helper.py"):
                with self.subTest(what=what, file=filename):
                    shutil.rmtree(self.root / "src", ignore_errors=True)
                    self.simple_part("src", "cube.py", "Cube", self.cube)
                    path = self.root / "src" / "parts" / filename
                    path.write_text(code + path.read_text() if filename == "cube.py" else code)
                    sources, failures = self.discover()
                    self.assertTrue(any(f.startswith(f"src/parts/{filename}:") for f in failures), failures)
                    self.assertEqual(sources[0].parts, [], "a source failing the scan is not imported")

    def test_name_with_a_path_fails(self):
        for name in ("../step/Cube", "a/b", ".hidden", "a\\b"):
            with self.subTest(name=name):
                shutil.rmtree(self.root / "src", ignore_errors=True)
                self.simple_part("src", "cube.py", name, self.cube)
                _, failures = self.discover()
                self.assertTrue(any("a part name is one file name" in f for f in failures), failures)

    def test_stl_outside_the_repository_fails(self):
        outside = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, outside)
        write_stl(Box(10, 10, 10), outside / "ref.stl")
        rel = os.path.relpath(outside / "ref.stl", self.repo)
        for stl in (str(outside / "ref.stl"), rel):
            with self.subTest(stl=stl):
                shutil.rmtree(self.root / "src", ignore_errors=True)
                self.simple_part("src", "cube.py", "Cube", stl)
                _, failures = self.discover()
                self.assertTrue(any("relative to the repository root, inside it" in f for f in failures), failures)

    def test_contract_failure_writes_nothing(self):
        self.part_file("src", "bad.py", "import os\nPARTS = []\n")
        code, out = quiet(build.main, [], self.root, self.repo)
        self.assertEqual(code, 1)
        self.assertIn("bad.py:1: imports os", out)
        self.assertFalse((self.root / "src" / "step").exists())


# --- 1.2 gates --------------------------------------------------------------


class Gates(Temp):
    def setUp(self):
        super().setUp()
        self.ref = self.repo / self.stl("ref/bored.stl", bored())

    def check(self, shape, **kw):
        kw.setdefault("samples", FEW)
        return gates.check("p", shape, self.ref, **kw)

    def test_good_shape_passes(self):
        report = self.check(bored())
        self.assertEqual(report.failures, [])
        self.assertIsNotNone(report.solid)

    def test_two_solids_fail(self):
        report = self.check(Compound([bored(), Pos(40, 0, 0) * Box(2, 2, 2)]))
        self.assertTrue(failed(report, "solid"), report.failures)
        self.assertIn("2 solids", report.failures[0])

    def test_missing_bore_fails(self):
        report = self.check(block())
        self.assertTrue(failed(report, "deviation max"), report.failures)
        self.assertTrue(failed(report, "volume"), report.failures)
        self.assertIn("STL to part 5.0", next(f for f in report.failures if "deviation max" in f))

    def test_face_off_fails(self):
        report = self.check(bored(z1=10.3))
        self.assertTrue(failed(report, "deviation max"), report.failures)

    def test_extra_boss_fails(self):
        report = self.check(bored() + Pos(3, 3, 10.5) * Box(2, 2, 1))
        self.assertTrue(failed(report, "deviation max"), report.failures)
        msg = next(f for f in report.failures if "deviation max" in f)
        self.assertIn("part to STL 1.0", msg)

    def test_box_too_long_fails_bounding_box(self):
        report = self.check(bored(x1=20.2))
        self.assertTrue(failed(report, "bounding box"), report.failures)
        self.assertIn("x max", next(f for f in report.failures if "bounding box" in f))
        self.assertFalse(failed(self.check(bored(x1=20.1)), "bounding box"), "0.1 mm is within the 0.15 mm limit")

    def test_small_boss_missed_by_samples_fails_through_vertices(self):
        boss = Pos(14.5, 14.5, 10.15) * Box(1, 1, 0.3)
        part = bored() + boss
        part_mesh, ref_mesh = gates.solid_mesh(part.solids()[0]), gates.load_stl(self.ref)
        samples = 300
        for seed in range(200):
            a = gates.direction(gates.side_points(part_mesh, samples, seed), ref_mesh)
            b = gates.direction(gates.side_points(ref_mesh, samples, seed + 1), part_mesh)
            if a.samples.max() < 0.1 and b.samples.max() < 0.1:
                break
        else:
            self.fail("no seed keeps every area sample off the boss")
        self.assertGreater(max(a.vertices.max(), b.vertices.max()), 0.29)
        report = self.check(part, samples=samples, seed=seed)
        self.assertTrue(failed(report, "deviation max"), report.failures)

    def test_nan_volume_fails(self):
        with mock.patch.object(Solid, "volume", new=property(lambda self: float("nan"))):
            report = self.check(bored())
        self.assertTrue(failed(report, "volume"), report.failures)

    def test_bisected_vertices_bound_every_edge(self):
        import numpy as np

        tri = np.array([[[0, 0, 0], [230, 0, 0], [0, 1, 0]]], dtype=float)
        points = gates.bisected_vertices(tri, 0.5)
        # Far fewer than splitting every edge, which turns the sliver into 4^9
        # facets (about 131 000 vertices).
        self.assertLess(len(points), 20_000)
        # Every point of the facet lies within 0.5 / sqrt(3) of a vertex.
        rng = np.random.default_rng(1)
        u, v = rng.random(5000), rng.random(5000)
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        probe = u[:, None] * tri[0, 1] + v[:, None] * tri[0, 2]
        nearest = np.min(np.linalg.norm(probe[:, None, :] - points[None, :, :], axis=2), axis=1)
        self.assertLessEqual(nearest.max(), gates.SAMPLE_BOUND + 1e-9)


class Exceptions(Temp):
    def test_exception_applies_to_its_part_only(self):
        pocket = "Pos(10, 15, 9.85) * Box(2, 2, 0.3)"
        for name in ("A", "B"):
            stl = self.stl(f"ref/{name}.stl", block())
            exc = ('{"deviation_max": {"limit": 0.35, "feature": "the 0.3 mm pocket", "reason": "test"}}'
                   if name == "A" else "{}")
            self.part_file("src", f"{name.lower()}.py", f"""
                from build123d import Box, Pos
                def build():
                    return Pos(10, 10, 5) * Box(20, 20, 10) - {pocket}
                PARTS = [{{"name": {name!r}, "stl": {stl!r}, "build": build, "exceptions": {exc}}}]
                """)
        sources, failures = discover(self.root, self.repo)
        self.assertEqual(failures, [])
        (solids, failures), out = quiet(build.check_parts, sources, samples=FEW)
        self.assertIn("exception: src/A deviation_max limit 0.35 (global 0.2) for the 0.3 mm pocket", out)
        self.assertNotIn("exception: src/B", out)
        self.assertEqual([f.split(":")[0] for f in failures], ["src/B"], failures)
        self.assertEqual(list(solids["src"]), ["A"])

    def test_unknown_gate_exception_fails(self):
        stl = self.stl("ref/a.stl", block())
        self.part_file("src", "a.py", f"""
            from build123d import Box
            def build():
                return Box(1, 1, 1)
            PARTS = [{{"name": "A", "stl": {stl!r}, "build": build,
                       "exceptions": {{"solid": {{"limit": 1, "feature": "x", "reason": "y"}}}}}}]
            """)
        _, failures = discover(self.root, self.repo)
        self.assertTrue(any("unknown gate 'solid'" in f for f in failures), failures)


class MirroredPair(Temp):
    def pair(self, right_shape):
        left = self.stl("ref/left.stl", bored(x=5, y=5))
        right = self.stl("ref/right.stl", right_shape)
        self.part_file("src", "pair.py", f"""
            from build123d import Box, Cylinder, Plane, Pos
            BORE_X = 5
            def build():
                return Pos(10, 10, 5) * Box(20, 20, 10) - Pos(BORE_X, 5, 5) * Cylinder(3, 30)
            PARTS = [{{"name": "Left", "stl": {left!r}, "build": build}},
                     {{"name": "Right", "stl": {right!r}, "mirror_of": "Left", "plane": Plane.YZ}}]
            """)
        sources, failures = discover(self.root, self.repo)
        self.assertEqual(failures, [])
        return quiet(build.check_parts, sources, samples=FEW)[0]

    def test_each_side_checked_against_its_own_stl(self):
        solids, failures = self.pair(bored(x=5, y=5).mirror(Plane.YZ))
        self.assertEqual(failures, [])
        self.assertEqual(sorted(solids["src"]), ["Left", "Right"])
        self.assertLess(solids["src"]["Right"].bounding_box().max.X, 1e-6)

    def test_wrong_right_stl_fails_the_right_only(self):
        solids, failures = self.pair(bored(x=15, y=5).mirror(Plane.YZ))
        self.assertTrue(failures)
        self.assertTrue(all(f.startswith("src/Right:") for f in failures), failures)
        self.assertEqual(list(solids["src"]), ["Left"])


# --- 1.3 STEP export --------------------------------------------------------


class Export(Temp):
    def setUp(self):
        super().setUp()
        self.src = self.root / "src"
        (self.src / "parts").mkdir(parents=True)

    def solid(self, **kw):
        return bored(**kw).solids()[0]

    def files(self, d: Path) -> dict[str, bytes]:
        return {p.name: p.read_bytes() for p in sorted(d.iterdir())} if d.exists() else {}

    def test_unchanged_part_writes_identical_file(self):
        self.assertEqual(export.export_all([(self.src, {"A": self.solid()})]), [])
        first = self.files(self.src / "step")
        self.assertEqual(list(first), ["A.step"])
        self.assertEqual(export.export_all([(self.src, {"A": self.solid()})]), [])
        self.assertEqual(self.files(self.src / "step"), first)
        self.assertEqual(sorted(p.name for p in self.src.iterdir()), ["parts", "step"], "no staging left")

    def test_removed_part_is_gone(self):
        export.export_all([(self.src, {"A": self.solid(), "B": self.solid(x=5)})])
        self.assertEqual(sorted(self.files(self.src / "step")), ["A.step", "B.step"])
        export.export_all([(self.src, {"A": self.solid()})])
        self.assertEqual(sorted(self.files(self.src / "step")), ["A.step"])

    def test_failed_reread_leaves_step_unchanged(self):
        export.export_all([(self.src, {"A": self.solid()})])
        before = self.files(self.src / "step")
        two = Compound([Box(1, 1, 1), Pos(5, 0, 0) * Box(1, 1, 1)])
        with mock.patch("build123d.import_step", return_value=two):
            failures = export.export_all([(self.src, {"A": self.solid(x=5), "B": self.solid()})])
        self.assertTrue(any("re-reads as 2 solids" in f for f in failures), failures)
        self.assertEqual(self.files(self.src / "step"), before)
        self.assertEqual(sorted(p.name for p in self.src.iterdir()), ["parts", "step"])

    def test_nan_reread_fails(self):
        solid = self.solid()
        nan = float("nan")
        corner = mock.Mock(X=nan, Y=nan, Z=nan)
        for what, volume, box in (("volume", nan, solid.bounding_box()),
                                  ("bounding box", solid.volume, mock.Mock(min=corner, max=corner))):
            with self.subTest(what=what):
                shape = mock.Mock()
                shape.solids.return_value = [mock.Mock(volume=volume)]
                shape.bounding_box.return_value = box
                with mock.patch("build123d.import_step", return_value=shape):
                    failures = export.export_all([(self.src, {"A": solid})])
                self.assertTrue(any(what in f for f in failures), failures)
                self.assertFalse((self.src / "step").exists())

    def test_failed_second_rename_keeps_old_step(self):
        export.export_all([(self.src, {"A": self.solid()})])
        before = self.files(self.src / "step")
        real = os.rename

        def rename(src, dst):
            if Path(src).name.startswith(export.STAGING_PREFIX):
                raise OSError("stubbed failure of the second rename")
            return real(src, dst)

        with mock.patch.object(export.os, "rename", side_effect=rename), self.assertRaises(OSError):
            export.export_all([(self.src, {"B": self.solid(x=5)})])
        self.assertEqual(self.files(self.src / "step"), before)
        self.assertFalse((self.src / export.OLD_DIR).exists())

    def test_leftover_old_is_restored_in_check_mode(self):
        export.export_all([(self.src, {"A": self.solid()})])
        before = self.files(self.src / "step")
        os.rename(self.src / "step", self.src / export.OLD_DIR)  # killed between the renames
        code, out = quiet(build.main, ["--check"], self.root, self.repo)
        self.assertEqual(code, 0)
        self.assertIn("restored step/", out)
        self.assertEqual(self.files(self.src / "step"), before)
        self.assertFalse((self.src / export.OLD_DIR).exists())

    def test_leftover_old_beside_step_is_removed(self):
        export.export_all([(self.src, {"A": self.solid()})])
        shutil.copytree(self.src / "step", self.src / export.OLD_DIR)
        quiet(build.main, ["--check"], self.root, self.repo)
        self.assertFalse((self.src / export.OLD_DIR).exists())
        self.assertEqual(list(self.files(self.src / "step")), ["A.step"])

    def test_publishing_one_source_leaves_another_alone(self):
        other = self.root / "other"
        (other / "parts").mkdir(parents=True)
        export.export_all([(other, {"B": self.solid(x=5)})])
        before = self.files(other / "step")
        export.export_all([(self.src, {"A": self.solid()})])
        self.assertEqual(self.files(other / "step"), before)


# --- 1.4 import checks ------------------------------------------------------


class ImportChecks(Temp):
    def checked(self, name="A", volume=1000.0) -> import_check.Checked:
        return import_check.Checked(name, self.repo / f"{name}.step", volume)

    def test_tools_hidden_report_skipped_and_build_exits_zero(self):
        stl = self.stl("ref/a.stl", bored())
        self.simple_part("src", "a.py", "A", stl,
                         "Pos(10, 10, 5) * Box(20, 20, 10) - Pos(10, 10, 5) * Cylinder(3, 30)")

        def no_run(*a, **kw):
            raise AssertionError("no tool may run when none is installed")

        with mock.patch.dict(os.environ, {"PATH": ""}), mock.patch.object(import_check, "run", no_run), \
                mock.patch.object(gates, "SAMPLES", FEW):
            code, out = quiet(build.main, [], self.root, self.repo)
        self.assertEqual(code, 0, out)
        self.assertIn("SKIPPED A [FreeCAD] FreeCAD not found", out)
        self.assertIn("SKIPPED A [solidcraft] solidcraft-cli not found on PATH", out)
        self.assertIn("import checks, FreeCAD: 0 passed, 0 failed, 1 skipped", out)
        self.assertIn("import checks, solidcraft: 0 passed, 0 failed, 1 skipped", out)
        self.assertNotIn("PASS", out)
        self.assertTrue((self.root / "src" / "step" / "A.step").is_file())

    def freecad(self, files: dict) -> list[import_check.Result]:
        part = self.checked()
        payload = {"version": "1.1.4", "files": {str(part.step.resolve()): files}}
        out = "FreeCAD 1.1.4\n" + import_check.FREECAD_MARKER + json.dumps(payload) + "\n"
        proc = mock.Mock(returncode=0, stdout=out, stderr="")
        with mock.patch.object(import_check, "run", return_value=proc):
            return import_check.check_freecad([part], ["FreeCADCmd"])

    def solidcraft(self, bodies: list[dict]) -> list[import_check.Result]:
        version = mock.Mock(returncode=0, stdout="solidcraft-cli 0.30.1\n", stderr="")
        evals = mock.Mock(returncode=0, stdout=json.dumps({"bodies": bodies}), stderr="")
        with mock.patch.object(import_check, "run", side_effect=[version, evals]):
            return import_check.check_solidcraft([self.checked()], ["solidcraft-cli"])

    def test_stubbed_mismatches_fail(self):
        good_fc = {"solids": 1, "closed": True, "valid": True, "volume": 1000.0}
        good_sc = {"closed": True, "volume_mm3": 1000.0}
        cases = {
            "two solids": ({**good_fc, "solids": 2}, [good_sc, good_sc], "2 solids, not one"),
            "open shell": ({**good_fc, "closed": False}, [{**good_sc, "closed": False}], "an open shell"),
            "volume 0.2 % off": ({**good_fc, "volume": 1002.0}, [{**good_sc, "volume_mm3": 1002.0}],
                                 "volume 1002.000 mm3, built 1000.000 mm3 (0.200 %"),
        }
        for what, (fc, sc, expect) in cases.items():
            for tool, results in (("FreeCAD", self.freecad(fc)), ("solidcraft", self.solidcraft(sc))):
                with self.subTest(what=what, tool=tool):
                    (r,) = results
                    self.assertEqual((r.status, r.part, r.tool), ("FAIL", "A", tool))
                    self.assertIn(expect, r.detail)
                    self.assertTrue(import_check.failed(results))
        for tool, results in (("FreeCAD", self.freecad(good_fc)), ("solidcraft", self.solidcraft([good_sc]))):
            with self.subTest(what="good", tool=tool):
                self.assertEqual([r.status for r in results], ["PASS"])

    def test_non_finite_volume_fails(self):
        for tool, results in (("FreeCAD", self.freecad({"solids": 1, "closed": True, "valid": True,
                                                         "volume": float("nan")})),
                              ("solidcraft", self.solidcraft([{"closed": True, "volume_mm3": float("nan")}]))):
            with self.subTest(tool=tool):
                self.assertEqual([r.status for r in results], ["FAIL"])
                self.assertIn("volume nan", results[0].detail)

    def test_malformed_tool_output_fails_per_part(self):
        part = self.checked()
        bad_fc = [import_check.FREECAD_MARKER + "{", import_check.FREECAD_MARKER + '{"files": []}',
                  import_check.FREECAD_MARKER + json.dumps({"files": {str(part.step.resolve()): {"solids": "x"}}})]
        good = {"solids": 1, "closed": True, "valid": True, "volume": 1000.0}
        for key, bad in (("solids", 1.9), ("solids", True), ("closed", "false"), ("valid", "false"),
                         ("volume", "1000"), ("volume", True)):
            got = {**good, key: bad}
            bad_fc.append(import_check.FREECAD_MARKER + json.dumps({"files": {str(part.step.resolve()): got}}))
        for out in bad_fc:
            with self.subTest(freecad=out):
                proc = mock.Mock(returncode=0, stdout=out + "\n", stderr="")
                with mock.patch.object(import_check, "run", return_value=proc):
                    results = import_check.check_freecad([part], ["FreeCADCmd"])
                self.assertEqual([(r.part, r.status) for r in results], [("A", "FAIL")])
        for body in ('{"bodies": 3}', '{"bodies": [{"closed": true}]}', '{"bodies": ["x"]}', "{}",
                     '{"bodies": [{"closed": "true", "volume_mm3": 1000.0}]}',
                     '{"bodies": [{"closed": true, "volume_mm3": "1000"}]}'):
            with self.subTest(solidcraft=body):
                version = mock.Mock(returncode=0, stdout="solidcraft-cli 0.30.1\n", stderr="")
                evals = mock.Mock(returncode=0, stdout=body, stderr="")
                with mock.patch.object(import_check, "run", side_effect=[version, evals]):
                    results = import_check.check_solidcraft([part], ["solidcraft-cli"])
                self.assertEqual([(r.part, r.status) for r in results], [("A", "FAIL")])

    def test_freecad_unreadable_file_fails(self):
        (r,) = self.freecad({"error": "Cannot read STEP file"})
        self.assertEqual(r.status, "FAIL")
        self.assertIn("Cannot read STEP file", r.detail)

    @unittest.skipUnless(import_check.freecad_command() and import_check.solidcraft_command(),
                         "FreeCAD and solidcraft-cli are not both installed")
    def test_real_tools_pass_a_synthetic_part(self):
        # Inside the checkout, not /tmp: the FreeCAD flatpak sees the host's
        # files but has its own /tmp.
        with tempfile.TemporaryDirectory(prefix=".test-import-", dir=HERE) as tmp:
            solid = bored().solids()[0]
            path = Path(tmp) / "A.step"
            export_step(solid, str(path))
            results = import_check.check_all([import_check.Checked("A", path, solid.volume)])
        for r in results:
            print(r.line())
        self.assertEqual([(r.tool, r.status) for r in results], [("FreeCAD", "PASS"), ("solidcraft", "PASS")],
                         [r.line() for r in results])


# --- 2. the frame group ------------------------------------------------------

AECERT = "aecert-hexapod-v1"
FRAME_FILES = {"frame_top.py": "Frame Top", "frame_bottom.py": "Frame Bottom", "servo_mount.py": "Servo Mount",
               "bearing_axle_mount.py": "Bearing Axle Mount", "hubcap.py": "Hubcap"}


class FrameGroup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sources, failures = discover(HERE, build.REPO)
        cls.failures = failures
        cls.source = next((s for s in sources if s.name == AECERT), None)

    def part(self, name):
        return next(p for p in self.source.parts if p.name == name)

    def test_the_frame_group_is_declared(self):
        self.assertEqual(self.failures, [])
        self.assertIsNotNone(self.source, f"no source {AECERT}")
        declared = {p.file.name: p.name for p in self.source.parts if p.file.name in FRAME_FILES}
        for filename, name in FRAME_FILES.items():
            self.assertEqual(declared.get(filename), name, f"frame part {name} is not declared in {filename}")
        self.assertEqual(sorted(declared.values()), sorted(FRAME_FILES.values()))
        for name in FRAME_FILES.values():
            self.assertEqual(self.part(name).stl, build.REPO / "hardware" / AECERT / "stl" / f"{name}.stl")

    def test_a_thicker_hubcap_fails_its_gates(self):
        part = self.part("Hubcap")
        module = part.build.__globals__
        good, _ = quiet(gates.check, "Hubcap", part.build(), part.stl, samples=FEW)
        self.assertEqual(good.failures, [])
        module["THICKNESS"] += 0.5
        try:
            thicker = part.build()
        finally:
            module["THICKNESS"] -= 0.5
        self.assertAlmostEqual(thicker.bounding_box().size.Z, module["THICKNESS"] + 0.5, places=6)
        report, _ = quiet(gates.check, "Hubcap", thicker, part.stl, samples=FEW)
        self.assertTrue(failed(report, "volume") or failed(report, "deviation"), report.failures)

    def test_mount_hole_pairs_lie_where_the_readme_says(self):
        # README: each Bearing Axle Mount screws to a pair of frame holes
        # 28.5 mm apart, 87.60 mm (middle legs) or 98.13 mm (corners) from
        # the centre, at 0° and ±56° from the sides. Found as the frames'
        # r 1.6 holes, not from the constants that place them.
        for name in ("Frame Top", "Frame Bottom"):
            solid = self.part(name).build()
            centres = {(round(e.arc_center.X, 3), round(e.arc_center.Y, 3)) for e in solid.edges()
                       if e.geom_type.name == "CIRCLE" and abs(e.radius - 1.6) < 1e-6}
            pairs = []
            for a in centres:
                for b in centres:
                    if a < b and abs(math.dist(a, b) - 28.5) < 0.01:
                        pairs.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
            with self.subTest(part=name):
                self.assertEqual(len(pairs), 6, pairs)
                radii = sorted(round(math.hypot(*m), 2) for m in pairs)
                self.assertEqual(radii, [87.6, 87.6, 98.13, 98.13, 98.13, 98.13])
                angles = sorted(round(math.degrees(math.atan2(m[1], m[0])) % 180, 2) for m in pairs)
                self.assertEqual(angles, [0.0, 0.0, 56.0, 56.0, 124.0, 124.0])

    def test_readme_figures(self):
        servo = self.part("Servo Mount").build.__globals__
        bam = self.part("Bearing Axle Mount").build.__globals__
        # README: the Servo Mount's 15 mm ring and 5 mm ear pads ...
        self.assertAlmostEqual(servo["RING_HEIGHT"], 15.0)
        self.assertAlmostEqual(servo["PAD_TOP_Z"] - servo["RING_BOTTOM_Z"] - servo["RING_HEIGHT"], 5.0)
        ring = self.part("Servo Mount").build()
        self.assertAlmostEqual(ring.bounding_box().size.Z, 20.0, places=6)
        # ... the pads 57 mm long ...
        self.assertAlmostEqual(ring.bounding_box().size.X, 57.0, places=6)
        # ... and the Bearing Axle Mount's axle 9.90 mm off its screws' line,
        # on a 9.4 mm base.
        self.assertAlmostEqual(bam["AXLE_X"] - bam["SCREW_X"], 9.90)
        self.assertAlmostEqual(bam["PLATE_THICKNESS"], 9.4)


class ChordError(unittest.TestCase):
    def test_a_tessellated_cylinder_gives_its_sagitta(self):
        import trimesh

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.stl"
            trimesh.creation.cylinder(radius=10.0, height=5.0, sections=24).export(str(path))
            f = chord_error.measure(path)
        self.assertIsNotNone(f)
        self.assertAlmostEqual(f.radius, 10.0, places=3)
        self.assertAlmostEqual(f.segments_per_turn, 24.0, places=3)
        self.assertAlmostEqual(f.error, 10.0 * (1 - math.cos(math.pi / 24)), places=4)

    def test_a_level_cylinder_is_seen_too(self):
        import trimesh

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.stl"
            mesh = trimesh.creation.cylinder(radius=10.0, height=5.0, sections=24)
            mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            mesh.export(str(path))
            f = chord_error.measure(path)
        self.assertIsNotNone(f)
        self.assertEqual(f.axis, "x")
        self.assertAlmostEqual(f.error, 10.0 * (1 - math.cos(math.pi / 24)), places=4)

    def test_the_frame_parts_match_the_recorded_table(self):
        # design.md, "Tolerances": the chord error table task 2.1 records,
        # and the 0.15 mm above which a part needs an exception.
        design = (build.REPO / "openspec" / "changes" / "parametric-reference-parts" / "design.md").read_text()
        rows = {}
        for line in design.splitlines():
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if len(cells) == 3 and cells[0] in FRAME_FILES.values():
                rows[cells[0]] = (float(cells[1]), cells[2])
        self.assertEqual(sorted(rows), sorted(FRAME_FILES.values()))
        sources, _ = discover(HERE, build.REPO)
        parts = {p.name: p for s in sources if s.name == AECERT for p in s.parts}
        for name, (error, feature) in rows.items():
            with self.subTest(part=name):
                f = chord_error.measure(parts[name].stl)
                self.assertEqual(f"{f.error:.3f}", f"{error:.3f}")
                self.assertIn(f"r {f.radius:.2f}, n {f.segments_per_turn:.0f}", feature)
                self.assertLessEqual(f.error, 0.15, f"{name} needs a tessellation exception")
                self.assertNotIn("deviation_max", parts[name].exceptions)

    def test_a_box_has_no_curves(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_stl(block(), Path(tmp) / "b.stl")
            self.assertIsNone(chord_error.measure(path))


if __name__ == "__main__":
    unittest.main()
