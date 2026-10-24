"""Tests of the mount's checks, the build and the export (tasks 2.1 to 2.3).

    uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --test

Each check is shown to fail on a deliberately bad shape and to pass on a good
one. The export is tested on a small fixture set (a deck on the eight
inserts, a bumper with copies, a cover with inlays): one print file and one
STEP file per distinct part, each STEP re-read with the built part's bounding
box, and the deck's holes found again on the insert positions.
"""

from __future__ import annotations

import io
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from build123d import Location

import build
import checks as C
import measurements as M
from export import export_parts, verify_export
from model import (LM2596_OUTPUT_ROUTE, SCHOTTKY, AirChannel, Assembly, Exposure, Held, Opening, Part, Route, Screw,
                   box, teardrop)

UP = (0.0, 0.0, 1.0)
DOWN = (0.0, 0.0, -1.0)
Z0 = 26.0  # a deck's underside, just above the ring


def deck(holes=M.FRAME_INSERTS, name="deck") -> Part:
    """A plate over the eight inserts, inside the front and rear tabs."""
    shape = box(-24, 24, -78, 78, Z0, Z0 + 3)
    for x, y in holes:
        shape = shape - C.cylinder(C.M3_CLEARANCE_D / 2, (x, y, Z0 - 1), (0, 0, 1), 5)
    return Part(name, shape, "deck", UP, "black")


def block(x0, x1, y0, y1, z0, z1, name="block", role="holder", up=UP) -> Part:
    return Part(name, box(x0, x1, y0, y1, z0, z1), role, up, "white")


def cover(window=None, hole=None) -> Part:
    """An open-bottomed shell over (-20..20, -30..30, 30..50), 2 mm walls;
    `window` and `hole` are (x0, x1, y0, y1) cut through the lid."""
    shape = box(-22, 22, -32, 32, 30, 52) - box(-20, 20, -30, 30, 29, 50)
    for cut in (window, hole):
        if cut:
            shape = shape - box(*cut, 49, 53)
    return Part("cover", shape, "cover", DOWN, "black")


def fails(failures: list[str], *words: str) -> bool:
    return any(all(w in f for w in words) for f in failures)


class SingleSolid(unittest.TestCase):
    def test_one_solid_passes(self):
        self.assertEqual(C.check_single_solid(Assembly(parts=[deck()])), [])

    def test_two_solids_fail(self):
        two = Part("two", box(0, 5, 0, 5, 30, 35) + box(10, 15, 0, 5, 30, 35), "holder", UP, "white")
        self.assertTrue(fails(C.check_single_solid(Assembly(parts=[two])), "two", "2 solids"))

    def test_shared_name_fails(self):
        a, b = block(0, 5, 0, 5, 30, 35), block(10, 15, 0, 5, 30, 35)
        self.assertTrue(fails(C.check_single_solid(Assembly(parts=[a, b])), "share the name"))


class Fit(unittest.TestCase):
    def test_inside_passes(self):
        asm = Assembly(parts=[deck()], held=[Held("board", box(-10, 10, -10, 10, 30, 32))])
        self.assertEqual(C.check_fit(asm), [])

    def test_outside_the_radius_fails(self):
        asm = Assembly(parts=[block(60, 70, 40, 50, 30, 35)])  # r 81 at its far corner
        self.assertTrue(fails(C.check_fit(asm), "block", "keep-clear"))

    def test_tab_passes_and_beyond_it_fails(self):
        self.assertEqual(C.check_fit(Assembly(parts=[block(-20, 20, 70, 78.9, 30, 35)])), [])
        self.assertTrue(fails(C.check_fit(Assembly(parts=[block(-20, 20, 70, 79.5, 30, 35)])), "keep-clear"))

    def test_held_part_outside_fails(self):
        asm = Assembly(held=[Held("fan", box(70, 80, -5, 5, 30, 40))])
        self.assertTrue(fails(C.check_fit(asm), "fan", "keep-clear"))

    def test_below_the_floor_fails(self):
        self.assertTrue(fails(C.check_fit(Assembly(parts=[block(-5, 5, -5, 5, 19.5, 30)])), "below the frame floor"))

    def test_a_copy_outside_fails(self):
        part = block(-5, 5, -5, 5, 30, 35, name="bumper")
        part.copies = (Location((70, 30, 0)),)
        self.assertTrue(fails(C.check_fit(Assembly(parts=[part])), "bumper copy 2", "keep-clear"))

    def test_bed(self):
        import numpy as np

        def pts(x, y, z):
            return np.array([(a, b, c) for a in (0, x) for b in (0, y) for c in (0, z)], dtype=float)

        self.assertIsNotNone(C.bed_fit(pts(250, 250, 10), UP))
        self.assertIsNotNone(C.bed_fit(pts(300, 10, 10), UP), "turned diagonally, 300 x 10 fits")
        self.assertIsNone(C.bed_fit(pts(300, 300, 1), UP))
        self.assertIsNone(C.bed_fit(pts(10, 10, 300), UP), "300 tall as printed")
        self.assertIsNotNone(C.bed_fit(pts(10, 10, 300), (1, 0, 0)), "lying on its side")
        a = math.radians(0.5)
        turned = pts(255, 255, 10) @ np.array([[math.cos(a), math.sin(a), 0], [-math.sin(a), math.cos(a), 0], [0, 0, 1]])
        self.assertIsNotNone(C.bed_fit(turned, UP), "255 x 255 turned by 0.5° fits when turned back")
        self.assertIsNone(C.bed_fit(pts(257, 255, 10) @ np.array([[math.cos(a), math.sin(a), 0], [-math.sin(a), math.cos(a), 0], [0, 0, 1]]), UP))


class InsertHoles(unittest.TestCase):
    def test_holes_on_the_inserts_pass(self):
        self.assertEqual(C.check_insert_holes(Assembly(parts=[deck()])), [])

    def test_no_holes_fail(self):
        f = C.check_insert_holes(Assembly(parts=[deck(holes=())]))
        self.assertEqual(len(f), 8, f)

    def test_a_hole_off_its_insert_fails(self):
        holes = list(M.FRAME_INSERTS)
        holes[5] = (holes[5][0] + 1.0, holes[5][1])
        f = C.check_insert_holes(Assembly(parts=[deck(holes=holes)]))
        self.assertEqual(len(f), 1, f)
        self.assertIn(str(M.FRAME_INSERTS[5][0]), f[0])

    def test_a_hole_in_thin_air_fails(self):
        thin = Part("deck", box(-24, 24, -10, 10, Z0, Z0 + 3), "deck", UP, "black")
        f = C.check_insert_holes(Assembly(parts=[thin]))
        self.assertTrue(fails(f, "no material round"), f)

    def test_no_deck_fails(self):
        self.assertTrue(fails(C.check_insert_holes(Assembly(parts=[block(0, 5, 0, 5, 30, 35)])), "no deck"))


class Overlaps(unittest.TestCase):
    def test_apart_and_touching_pass(self):
        asm = Assembly(parts=[block(0, 5, 0, 5, 30, 35, name="a"), block(5, 10, 0, 5, 30, 35, name="b")],
                       held=[Held("board", box(0, 5, 0, 5, 35, 37))])
        self.assertEqual(C.check_overlaps(asm), [])

    def test_printed_parts_overlapping_fail(self):
        asm = Assembly(parts=[block(0, 5, 0, 5, 30, 35, name="a"), block(4, 10, 0, 5, 30, 35, name="b")])
        self.assertTrue(fails(C.check_overlaps(asm), "a overlaps b"))

    def test_held_parts_overlapping_fail(self):
        asm = Assembly(parts=[block(0, 5, 0, 5, 30, 35, name="a")],
                       held=[Held("board", box(4, 9, 0, 5, 34, 37)), Held("fan", box(8, 12, 0, 5, 36, 40))])
        f = C.check_overlaps(asm)
        self.assertTrue(fails(f, "a overlaps board"), f)
        self.assertTrue(fails(f, "board overlaps fan"), f)

    def test_copies_overlapping_fail(self):
        part = block(0, 5, 0, 5, 30, 35, name="bumper")
        part.copies = (Location((3, 0, 0)),)
        self.assertTrue(fails(C.check_overlaps(Assembly(parts=[part])), "bumper overlaps bumper copy 2"))


class Enclosure(unittest.TestCase):
    def test_enclosed_passes(self):
        asm = Assembly(parts=[cover()], held=[Held("board", box(-10, 10, -10, 10, 31, 35))])
        self.assertEqual(C.check_enclosure(asm), [])

    def test_sticking_out_fails(self):
        asm = Assembly(parts=[cover()], held=[Held("board", box(-10, 30, -10, 10, 31, 35))])
        self.assertTrue(fails(C.check_enclosure(asm), "board", "not enclosed"))

    def test_a_hole_in_the_lid_fails(self):
        asm = Assembly(parts=[cover(hole=(-5, 5, -5, 5))], held=[Held("board", box(-10, 10, -10, 10, 31, 35))])
        self.assertTrue(fails(C.check_enclosure(asm), "board", "towards +z"))

    def test_a_screen_under_its_window_passes(self):
        screen = Exposure(UP, (-6, 6, -6, 6, 35, 35))
        asm = Assembly(parts=[cover(window=(-6, 6, -6, 6))],
                       held=[Held("t-display", box(-10, 10, -10, 10, 31, 34) + box(-6, 6, -6, 6, 34, 35),
                                  exposed=(screen,))])
        self.assertEqual(C.check_enclosure(asm), [])

    def test_a_screen_without_its_window_fails(self):
        screen = Exposure(UP, (-6, 6, -6, 6, 35, 35))
        asm = Assembly(parts=[cover()],
                       held=[Held("t-display", box(-10, 10, -10, 10, 31, 34) + box(-6, 6, -6, 6, 34, 35),
                                  exposed=(screen,))])
        self.assertTrue(fails(C.check_enclosure(asm), "t-display", "must show"))

    def test_an_exposure_with_nothing_in_it_fails(self):
        nowhere = Exposure((1, 0, 0), (-30, -25, -1, 1, 30, 40))
        asm = Assembly(parts=[cover()], held=[Held("xt60", box(-10, 10, -10, 10, 31, 35), exposed=(nowhere,))])
        self.assertTrue(fails(C.check_enclosure(asm), "xt60", "nothing of it faces"))

    def test_held_parts_without_a_cover_are_not_checked(self):
        self.assertEqual(C.check_enclosure(Assembly(held=[Held("board", box(0, 1, 0, 1, 30, 31))])), [])


class Routes(unittest.TestCase):
    ROUTE = Route(LM2596_OUTPUT_ROUTE, box(0, 40, -4, 4, 30, 38))
    OTHER = Route("link", box(30, 34, -10, 10, 30, 38))
    SENSOR = Route("sensor", box(0, 40, 10, 20, 30, 38))

    def test_clear_passes(self):
        asm = Assembly(parts=[block(0, 40, 5, 10, 30, 38)], held=[Held("board", box(0, 40, -10, -5, 30, 38))],
                       routes=[self.ROUTE])
        self.assertEqual(C.check_routes(asm), [])

    def test_a_part_in_a_route_fails(self):
        asm = Assembly(parts=[block(0, 40, 3, 10, 30, 38)], routes=[self.ROUTE])
        self.assertTrue(fails(C.check_routes(asm), f"block enters the route '{LM2596_OUTPUT_ROUTE}'"))

    def test_a_held_part_in_a_route_fails(self):
        asm = Assembly(held=[Held("board", box(0, 40, -10, -3, 30, 38))], routes=[self.ROUTE])
        self.assertTrue(fails(C.check_routes(asm), "board enters"))

    def test_the_diode_in_its_route_passes(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        self.assertEqual(C.check_routes(Assembly(held=[diode], routes=[self.ROUTE, self.OTHER])), [])

    def test_the_diode_partly_outside_its_route_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 6, 31, 37), route=LM2596_OUTPUT_ROUTE)
        self.assertTrue(fails(C.check_routes(Assembly(held=[diode], routes=[self.ROUTE])), "outside its route"))

    def test_the_diode_in_another_route_fails(self):
        diode = Held(SCHOTTKY, box(15, 35, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        self.assertTrue(fails(C.check_routes(Assembly(held=[diode], routes=[self.ROUTE, self.OTHER])),
                              f"{SCHOTTKY} enters the route 'link'"))

    def test_the_diode_with_its_route_not_reserved_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        self.assertTrue(fails(C.check_routes(Assembly(held=[diode])), "is not reserved"))

    def test_the_diode_claiming_another_route_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, 12, 18, 31, 37), route="sensor")
        f = C.check_routes(Assembly(held=[diode], routes=[self.ROUTE, self.SENSOR]))
        self.assertTrue(fails(f, "only the", "not 'sensor'"), f)
        self.assertTrue(fails(f, f"{SCHOTTKY} enters the route 'sensor'"), f)

    def test_another_held_part_claiming_a_route_fails(self):
        board = Held("board", box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        f = C.check_routes(Assembly(held=[board], routes=[self.ROUTE]))
        self.assertTrue(fails(f, "board: only the"), f)
        self.assertTrue(fails(f, f"board enters the route '{LM2596_OUTPUT_ROUTE}'"), f)


class FrameClearance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frame = C.FrameMap()

    def asm(self, footprint, cut=True) -> Assembly:
        part = deck()
        if cut:
            part.shape = part.shape - box(*footprint, Z0 - 1, Z0 + 4)
        return Assembly(parts=[part], openings=[Opening("tie", "deck", footprint, Z0, Z0 + 3)])

    def test_over_open_frame_passes(self):
        self.assertEqual(C.check_frame_clearance(self.asm((-2.0, 2.0, -1.0, 1.0)), self.frame), [])

    def test_over_the_floor_passes(self):
        # the floor at z 20 round the floor inserts leaves 6 mm under a deck at z 26
        x, y = M.FRAME_FLOOR_INSERTS[0]
        top = self.frame.highest(x + 3, x + 4, y - 0.5, y + 0.5)
        self.assertEqual(top, M.FRAME_FLOOR_Z)
        self.assertEqual(C.check_frame_clearance(self.asm((x + 3, x + 4, y - 0.5, y + 0.5)), self.frame), [])

    def test_over_the_ring_too_close_fails(self):
        f = C.check_frame_clearance(self.asm((-1.0, 1.0, 70.0, 72.0)), self.frame)
        self.assertTrue(fails(f, "tie", "frame rises to z 25.75"), f)

    def test_not_cut_through_fails(self):
        f = C.check_frame_clearance(self.asm((-2.0, 2.0, -1.0, 1.0), cut=False), self.frame)
        self.assertTrue(fails(f, "does not run clear"), f)

    def test_a_missing_part_fails(self):
        asm = Assembly(openings=[Opening("slot", "deck", (-2.0, 2.0, -1.0, 1.0), Z0, Z0 + 3)])
        self.assertTrue(fails(C.check_frame_clearance(asm, self.frame), "no part 'deck'"))


class Overhangs(unittest.TestCase):
    def check(self, shape, up=UP) -> list[str]:
        return C.overhang_failures("part", [shape], up)

    def test_a_block_on_the_bed_passes(self):
        self.assertEqual(self.check(box(0, 20, 0, 20, 0, 10)), [])

    def test_a_ledge_fails(self):
        t = box(8, 12, 0, 20, 0, 10) + box(0, 20, 0, 20, 10, 13)
        self.assertTrue(fails(self.check(t), "downward face"))

    def test_a_narrow_bridge_passes_and_a_wide_one_fails(self):
        narrow = box(0, 20, 0, 10, 0, 10) - box(8.5, 11.5, -1, 11, 2, 6)  # a 3 mm tunnel across
        wide = box(0, 20, 0, 10, 0, 10) - box(6, 14, -1, 11, 2, 6)
        self.assertEqual(self.check(narrow), [])
        self.assertTrue(fails(self.check(wide), "downward face"))

    def test_a_45_degree_chamfer_passes_and_a_steeper_one_fails(self):
        from build123d import Polygon, extrude, Plane
        def wedge(run):  # a profile in xz rising from the bed, overhanging by `run` over 10 mm of height
            return extrude(Plane.XZ * Polygon((0, 0), (10, 0), (10 + run, 10), (0, 10), align=None), amount=-10).solid()
        self.assertEqual(self.check(wedge(10)), [])
        self.assertTrue(fails(self.check(wedge(12)), "downward face"))

    def test_a_round_horizontal_hole_fails_and_a_teardrop_passes(self):
        body = box(0, 20, 0, 10, 0, 10)
        self.assertTrue(fails(self.check(body - C.cylinder(1.7, (10, -1, 5), (0, 1, 0), 12)), "teardrop"))
        self.assertEqual(self.check(body - teardrop(1.7, 12, 10, 5, 5, "y", UP)), [])

    def test_a_teardrop_follows_the_print_orientation(self):
        lid_down = box(0, 20, 0, 10, 0, 10)
        self.assertEqual(self.check(lid_down - teardrop(1.7, 12, 10, 5, 5, "y", DOWN), DOWN), [])
        self.assertTrue(fails(self.check(lid_down - teardrop(1.7, 12, 10, 5, 5, "y", UP), DOWN), "teardrop"))

    def test_vertical_round_holes_pass(self):
        self.assertEqual(self.check(box(0, 20, 0, 10, 0, 10) - C.cylinder(1.7, (10, 5, -1), (0, 0, 1), 12)), [])

    def test_check_overhangs_uses_each_parts_orientation(self):
        ledge = box(8, 12, 0, 20, 30, 40) + box(0, 20, 0, 20, 40, 43)  # fine lid-down, not upright
        self.assertEqual(C.check_overhangs(Assembly(parts=[Part("cover", ledge, "cover", DOWN, "black")])), [])
        self.assertTrue(fails(C.check_overhangs(Assembly(parts=[Part("cover", ledge, "cover", UP, "black")])),
                              "cover", "downward face"))


class ScrewdriverPaths(unittest.TestCase):
    SCREW = Screw("deck screw", (20.0, 73.0, Z0 + 3), (0.0, 0.0, 1.0))

    def test_clear_path_passes(self):
        self.assertEqual(C.check_screwdriver_paths(Assembly(parts=[deck()], screws=[self.SCREW])), [])

    def test_a_part_in_the_path_fails(self):
        asm = Assembly(parts=[deck(), block(15, 25, 70, 76, 40, 45, name="lid")], screws=[self.SCREW])
        self.assertTrue(fails(C.check_screwdriver_paths(asm), "deck screw", "lid blocks"))

    def test_a_held_part_in_the_path_fails(self):
        asm = Assembly(parts=[deck()], held=[Held("fan", box(18, 22, 72, 74, 50, 52))], screws=[self.SCREW])
        self.assertTrue(fails(C.check_screwdriver_paths(asm), "fan blocks"))

    def test_a_part_fitted_after_is_not_in_the_way(self):
        screw = Screw("deck screw", (20.0, 73.0, Z0 + 3), (0.0, 0.0, 1.0), after=("lid",))
        asm = Assembly(parts=[deck(), block(15, 25, 70, 76, 40, 45, name="lid")], screws=[screw])
        self.assertEqual(C.check_screwdriver_paths(asm), [])


class AirChannelCheck(unittest.TestCase):
    SZBK07 = Held("szbk07", box(-30, 30, -27, 27, 30, 58))

    def channel(self, **kw) -> AirChannel:
        args = dict(
            segments=(((-40, -30, 27, 42, 30, 58), "x"),   # in from the inlet
                      ((-30, 30, 27, 42, 30, 58), "x"),    # along the SZBK07's side
                      ((30, 40, 27, 42, 30, 60), "x")),    # out to the outlet
            inlet=(-42, -40, 27, 42, 30, 58),
            outlet=(40, 42, 27, 42, 32, 60),
            inlet_open_area=400.0,
            past="szbk07")
        args.update(kw)
        return AirChannel(**args)

    def test_a_clear_channel_passes(self):
        self.assertEqual(C.check_air_channel(Assembly(held=[self.SZBK07], channel=self.channel())), [])

    def test_a_part_in_the_channel_fails(self):
        asm = Assembly(parts=[block(-5, 5, 28, 34, 40, 45, name="fan holder")], held=[self.SZBK07],
                       channel=self.channel())
        self.assertTrue(fails(C.check_air_channel(asm), "segment 2", "fan holder enters"))

    def test_a_narrow_segment_fails(self):
        f = C.check_air_channel(Assembly(held=[self.SZBK07], channel=self.channel(inlet_open_area=430.0)))
        self.assertTrue(fails(f, "segment 2", "smaller than the inlet"), f)

    def test_a_gap_between_segments_fails(self):
        segs = self.channel().segments
        moved = (segs[0], segs[1], ((31, 40, 27, 42, 30, 60), "x"))
        f = C.check_air_channel(Assembly(held=[self.SZBK07], channel=self.channel(segments=moved)))
        self.assertTrue(fails(f, "segment 2 to segment 3"), f)

    def test_an_outlet_off_the_channel_fails(self):
        f = C.check_air_channel(Assembly(held=[self.SZBK07], channel=self.channel(outlet=(40, 42, 50, 60, 45, 60))))
        self.assertTrue(fails(f, "to outlet"), f)

    def test_a_blocked_inlet_fails(self):
        asm = Assembly(held=[self.SZBK07, Held("plug", box(-42, -40, 27, 42, 30, 58))], channel=self.channel())
        self.assertTrue(fails(C.check_air_channel(asm), "air channel inlet", "plug enters"))

    def test_a_blocked_outlet_fails(self):
        asm = Assembly(parts=[block(40, 42, 27, 42, 32, 60, name="grille")], held=[self.SZBK07],
                       channel=self.channel())
        self.assertTrue(fails(C.check_air_channel(asm), "air channel outlet", "grille enters"))

    def test_a_fan_is_left_out_of_the_passive_check(self):
        fan = Held("intake fan", box(-40, -30, 28, 41, 32, 56))
        self.assertEqual(C.check_air_channel(Assembly(held=[self.SZBK07, fan], channel=self.channel())), [])

    def test_a_fan_holder_or_another_held_part_is_not_left_out(self):
        holder = Part("intake fan", box(-40, -30, 28, 41, 32, 34), "holder", UP, "white")
        f = C.check_air_channel(Assembly(parts=[holder], held=[self.SZBK07], channel=self.channel()))
        self.assertTrue(fails(f, "segment 1", "intake fan enters"), f)
        fan = Held("intake fan", box(-40, -30, 28, 41, 34, 56))
        f = C.check_air_channel(Assembly(parts=[holder], held=[self.SZBK07, fan], channel=self.channel()))
        self.assertTrue(fails(f, "segment 1", "intake fan enters"), f)
        board = Held("fan board", box(-40, -30, 28, 41, 32, 56))
        f = C.check_air_channel(Assembly(held=[self.SZBK07, board], channel=self.channel()))
        self.assertTrue(fails(f, "segment 1", "fan board enters"), f)

    def test_a_channel_away_from_the_szbk07_fails(self):
        far = Held("szbk07", box(-30, 30, -27, 20, 30, 58))
        f = C.check_air_channel(Assembly(held=[far], channel=self.channel()))
        self.assertTrue(fails(f, "passes szbk07 no nearer"), f)


class Build(unittest.TestCase):
    def test_the_empty_part_set_passes_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            self.assertEqual(build.main([], outdir=Path(tmp)), 0)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_a_failing_check_writes_nothing(self):
        bad = Assembly(parts=[deck(), block(60, 70, 40, 50, 30, 35)])
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()) as out:
            self.assertEqual(build.main([], outdir=Path(tmp), make=lambda: bad), 1)
            self.assertEqual(list(Path(tmp).rglob("*")), [])
        self.assertIn("keep-clear", out.getvalue())

    def test_check_only_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            self.assertEqual(build.main(["--check"], outdir=Path(tmp), make=lambda: Assembly(parts=[deck()])), 0)
            self.assertEqual(list(Path(tmp).rglob("*")), [])


def fixture_parts() -> list[Part]:
    bumper = Part("bumper", box(-3, 3, 60, 66, 30, 40), "bumper", UP, "yellow",
                  copies=(Location((0, -126, 0)),))
    body = box(-22, 22, -32, 32, 30, 52) - box(-20, 20, -30, 30, 29, 50) - box(-10, 10, -2, 2, 51.5, 53)
    inlay = box(-10, 10, -2, 2, 51.5, 52)
    cov = Part("cover", body, "cover", DOWN, "black", inlays=inlay)
    return [deck(), bumper, cov]


class Export(unittest.TestCase):
    def test_each_distinct_part_once_and_reread(self):
        parts = fixture_parts()
        self.assertEqual(C.check_single_solid(Assembly(parts=parts)), [])
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            export_parts(parts, out)
            files = sorted(str(p.relative_to(out)) for p in out.rglob("*") if p.is_file())
            self.assertEqual(files, ["print/bumper.stl", "print/cover.3mf", "print/deck.stl",
                                     "step/bumper.step", "step/cover.step", "step/deck.step"])
            for part in parts:
                n = sum(1 for f in files if Path(f).stem == part.name)
                self.assertEqual(n, 2, f"{part.name}: one print file and one STEP file")
            self.assertEqual(verify_export(parts, out), [])

    def test_stale_files_are_removed_and_missing_ones_reported(self):
        parts = fixture_parts()
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / "print").mkdir()
            (out / "print" / "old.stl").write_text("solid old\nendsolid old\n")
            export_parts(parts, out)
            self.assertFalse((out / "print" / "old.stl").exists())
            (out / "step" / "bumper.step").unlink()
            f = verify_export(parts, out)
            self.assertTrue(fails(f, "step/bumper.step was not written"), f)

    def test_a_step_that_differs_from_the_built_part_fails(self):
        parts = fixture_parts()
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            export_parts(parts, out)
            parts[1].shape = box(-3, 3, 60, 66, 30, 41)
            f = verify_export(parts, out)
            self.assertTrue(fails(f, "bumper.step's bounding box"), f)

    def test_a_deck_without_its_holes_fails_on_reread(self):
        parts = [deck(holes=M.FRAME_INSERTS[:7])]
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            export_parts(parts, out)
            f = verify_export(parts, out)
            self.assertTrue(fails(f, "deck.step", "no clear M3 hole"), f)


if __name__ == "__main__":
    unittest.main()
