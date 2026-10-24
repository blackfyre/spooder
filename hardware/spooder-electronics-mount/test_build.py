"""Tests of the mount's checks, the build, the export, the layout and the
README's parts table (tasks 2.1 to 6.1).

    uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --test

Each check is shown to fail on a deliberately bad shape and to pass on a good
one. The export is tested on a small fixture set (a deck on the eight
inserts, a bumper with copies, a cover with inlays): one print file and one
STEP file per distinct part, each STEP re-read with the built part's bounding
box, and the deck's holes found again on the insert positions. The layout
(layout.py) is tested as built: every check passes on it, and what the checks
leave to the layout holds (the cover's interior, the battery plug inside the
legs' region, the spacers on the floor, the XT60's back stops, the bumpers,
the T-Display's holder and the loose holders in their pockets), and the
README lists every printed file with its copies, colour and orientation.
"""

from __future__ import annotations

import io
import math
import tempfile
import unittest
from unittest import mock
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path

from build123d import Location, Plane, Solid

import build
import checks as C
import measurements as M
from export import export_parts, model_xml, print_name, verify_export
import layout as L
from model import (FANS, IMU, LM2596_OUTPUT_ROUTE, SCHOTTKY, SERVO_2040, SZBK07, AirChannel, Assembly, Exposure, Fan, Held,
                   ImuPad, Opening, Part, Route, Screw, Support, Terminal, Vent, box, teardrop, turned)

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

    def test_a_grille_counts_as_closed(self):
        board = Held("board", box(-10, 10, -10, 10, 31, 35))
        grille = box(-5, 5, -5, 5, 49, 53)
        asm = Assembly(parts=[cover(hole=(-5, 5, -5, 5))], held=[board])
        self.assertTrue(fails(C.check_enclosure(asm), "board", "towards +z"))
        asm.vents = [Vent("lid grille", grille, 50.0)]
        self.assertEqual(C.check_enclosure(asm), [])

    def test_another_held_part_shields_it_but_a_fan_does_not(self):
        board = Held("board", box(-4, 4, -4, 4, 31, 35))
        asm = Assembly(parts=[cover(hole=(-5, 5, -5, 5))], held=[board, Held("t-display", box(-6, 6, -6, 6, 40, 42))])
        self.assertFalse(fails(C.check_enclosure(asm), "board"))
        asm.held[1] = Held(FANS[0], box(-6, 6, -6, 6, 40, 42))
        self.assertTrue(fails(C.check_enclosure(asm), "board", "towards +z"))

    def test_held_parts_without_a_cover_are_not_checked(self):
        self.assertEqual(C.check_enclosure(Assembly(held=[Held("board", box(0, 1, 0, 1, 30, 31))])), [])


class Routes(unittest.TestCase):
    ROUTE = Route.of(LM2596_OUTPUT_ROUTE, [(0, 40, -4, 4, 30, 38)])
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

    def test_a_narrow_cantilever_fails_and_a_short_ledge_passes(self):
        # An arm 2.4 mm wide from a column: narrow enough to bridge, but held
        # at one end only (the exhaust holders' first lips).
        column = box(0, 3, 0, 2.4, 0, 12)
        f = self.check(column + box(-4, 3, 0, 2.4, 8, 10))
        self.assertTrue(fails(f, "downward face", "a cantilever"), f)
        # (Its sides are flush with the column's, one face each with it: the
        # column below them does not hold the arm up.)
        self.assertEqual(self.check(column + box(-2.5, 3, 0, 2.4, 8, 10)), [])

    def test_a_cantilever_with_a_45_degree_underside_passes(self):
        from build123d import Polygon, extrude, Plane
        column = box(0, 3, 0, 2.4, 0, 12)
        arm = extrude(Plane.XZ * Polygon((0, 8), (-4, 12), (0, 12), align=None), amount=-2.4).solid()
        self.assertEqual(self.check(column + arm), [])

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

    def grille(self, filled: int = 0) -> tuple[Part, Vent]:
        """A wall beyond the outlet (x 42..44) with five 2 x 30 slots, 300 mm²
        open, the first `filled` of them filled in."""
        wall = box(42, 44, 26, 43, 30, 62)
        for i in range(5):
            if i >= filled:
                wall = wall - box(41, 45, 28 + 3 * i, 30 + 3 * i, 31, 61)
        return (Part("cover", wall, "cover", DOWN, "black"),
                Vent("outlet", box(42, 44, 26, 43, 30, 62), 300.0, (1.0, 0.0, 0.0)))

    def test_vents_are_measured_through_the_cover(self):
        cov, vent = self.grille()
        asm = Assembly(parts=[cov], held=[self.SZBK07], channel=self.channel(inlet_open_area=300.0), vents=[vent])
        self.assertEqual(C.check_air_channel(asm), [])
        cov, vent = self.grille(filled=1)
        asm = Assembly(parts=[cov], held=[self.SZBK07], channel=self.channel(inlet_open_area=300.0), vents=[vent])
        f = C.check_air_channel(asm)
        self.assertTrue(fails(f, "vent outlet", "240 mm² open", "not the 300"), f)
        self.assertTrue(fails(f, "vent outlet leaves 240", "less than the inlet's 300"), f)
        vent.across = None
        self.assertTrue(fails(C.check_air_channel(asm), "vent outlet", "no direction"))

    def test_a_slot_shut_at_any_depth_is_shut(self):
        # A skin across all five slots, however thin, at either face of the
        # wall or anywhere between, shuts them.
        cov, vent = self.grille()
        self.assertAlmostEqual(C.open_area(vent.shape, [cov.shape], vent.across), 300.0, delta=0.5)
        for x0, x1 in ((42.0, 42.09), (43.91, 44.0), (42.93, 42.97), (43.3, 43.31)):
            with self.subTest(skin=(x0, x1)):
                skin = cov.shape + box(x0, x1, 28, 42, 31, 61)
                self.assertAlmostEqual(C.open_area(vent.shape, [skin], vent.across), 0.0, delta=0.5)
        one = cov.shape + box(43, 43.4, 28, 30, 31, 61)
        self.assertAlmostEqual(C.open_area(vent.shape, [one], vent.across), 240.0, delta=0.5)

    def test_a_throat_between_vertex_depths_is_found(self):
        # A plug with planar faces whose section turns from 10 x 30 to 30 x 2
        # through a 2 mm wall: (10 + 20 s)(30 - 28 s) is largest at s = 2/7,
        # between its vertex depths, leaving 1600 - 345.7 mm² open there.
        from build123d import Wire
        ends = [Wire.make_rect(w, h, Plane(origin=(x, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)))
                for x, w, h in ((42, 10, 30), (44, 30, 2))]
        plug = Solid.make_loft(ends)
        self.assertAlmostEqual(C.open_area(box(42, 44, -20, 20, -20, 20), [plug], (1.0, 0.0, 0.0)),
                               1600 - (10 + 40 / 7) * (30 - 8), delta=0.5)

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

    def test_a_turned_segment_passes_into_a_corner(self):
        # A run along x, then one turned 45 degrees out to a corner outlet.
        sz = Held("szbk07", box(-30, 0, -30, -10.5, 30, 58))
        ch = AirChannel(segments=(((-20, 20, -10, 30, 30, 50), "x"), ((14, 40, -10, 10, 30, 50), "x", 45.0)),
                        inlet=(-22, -20, -10, 30, 30, 50), outlet=(40, 42, -10, 10, 30, 50), inlet_open_area=380.0,
                        past="szbk07", outlet_turn=45.0)
        self.assertEqual(C.check_air_channel(Assembly(held=[sz], channel=ch)), [])
        # Turned off to one side, it barely meets the run along x.
        ch.segments = (ch.segments[0], ((14, 40, 20, 40, 30, 50), "x", 45.0))
        ch.outlet = (40, 42, 20, 40, 30, 50)
        f = C.check_air_channel(Assembly(held=[sz], channel=ch))
        self.assertTrue(fails(f, "segment 1 to segment 2"), f)

    def test_the_widest_slice_of_a_box_is_its_cross_section(self):
        import numpy as np

        b = (0, 10, 0, 20, 0, 30)
        self.assertAlmostEqual(C.widest_slice(box(*b), np.array([1.0, 0, 0])), C.cross_section(b, "x"), delta=1.0)
        self.assertAlmostEqual(C.widest_slice(box(*b), np.array([0, 1.0, 0])), C.cross_section(b, "y"), delta=1.0)

    def test_a_channel_away_from_the_szbk07_fails(self):
        far = Held("szbk07", box(-30, 30, -27, 20, 30, 58))
        f = C.check_air_channel(Assembly(held=[far], channel=self.channel()))
        self.assertTrue(fails(f, "passes szbk07 no nearer"), f)


def fan_at(x: float, exhaust: bool = True) -> Fan:
    """A fan facing +x with its outer face at x, its underside at z 30."""
    return Fan(FANS[0] if exhaust else FANS[1], (x, 0.0, 30.0), (1.0, 0.0, 0.0), exhaust)


def fan_holder(x: float, name: str = "holder", lip: bool = True) -> Part:
    """A plate under the fan at x, with a cheek beyond each end and a lip
    inside its foot."""
    shape = box(x - 10, x, -16, 16, 28, 30) + box(x - 10, x, 15.3, 17.3, 28, 35) + box(x - 10, x, -17.3, -15.3, 28, 35)
    if lip:
        shape = shape + box(x - 12, x - 10.3, -16, 16, 28, 33)
    return Part(name, shape, "holder", UP, "white")


def grilled_cover(x: float, slots: bool = True) -> Part:
    """A box wall at x, beyond the fan, with slots in front of it."""
    shape = box(-30, x + 2.4, -30, 30, 28, 64) - box(-28, x, -28, 28, 27, 62)
    if slots:
        for i in range(7):
            shape = shape - box(x - 1, x + 3, -13 + 4 * i, -11 + 4 * i, 32, 58)
    return Part("cover", shape, "cover", DOWN, "black")


class Fans(unittest.TestCase):
    def asm(self, holder=True, cover_=True, slots=True, fan=None, x=20.0) -> Assembly:
        fan = fan or fan_at(x)
        parts = ([fan_holder(x)] if holder else []) + ([grilled_cover(x + 0.3, slots)] if cover_ else [])
        return Assembly(parts=parts, held=[Held(fan.name, fan.shape())], fans=[fan])

    def test_a_seated_fan_behind_its_grille_passes(self):
        self.assertEqual(C.check_fans(self.asm()), [])

    def test_a_fan_without_its_holder_fails(self):
        f = C.check_fans(self.asm(holder=False))
        self.assertTrue(fails(f, "nothing supports one end") and fails(f, "nothing locates the other end"), f)

    def test_a_fan_free_to_slide_fails(self):
        asm = self.asm()
        asm.parts[0] = fan_holder(20.0, lip=False)
        f = C.check_fans(asm)
        self.assertTrue(fails(f, "slides 1.0 mm inwards"), f)
        self.assertFalse(fails(f, "outwards"), f)
        f = C.check_fans(self.asm(cover_=False))
        self.assertTrue(fails(f, "slides 1.0 mm outwards"), f)

    def test_a_fan_without_a_vent_fails(self):
        self.assertTrue(fails(C.check_fans(self.asm(slots=False)), "no vent in front of it"))

    def test_a_fan_carried_by_the_cover_fails(self):
        asm = self.asm(holder=False)
        asm.parts[0].shape = asm.parts[0].shape + fan_holder(20.0).shape  # the holder made part of the cover
        f = C.check_fans(asm)
        self.assertTrue(fails(f, "the cover carries no fan"), f)
        self.assertTrue(fails(f, "nothing supports"), f)

    def test_an_exhaust_facing_into_the_box_fails(self):
        fan = Fan(FANS[0], (-20.0, 0.0, 30.0), (1.0, 0.0, 0.0), True)
        self.assertTrue(fails(C.check_fans(Assembly(held=[Held(fan.name, fan.shape())], fans=[fan])), "faces into the box"))

    def test_a_held_fan_out_of_its_position_fails(self):
        asm = self.asm()
        asm.held[0] = Held(FANS[0], fan_at(18.0).shape())
        self.assertTrue(fails(C.check_fans(asm), "not in its position"))

    def test_a_channel_needs_an_exhaust_position(self):
        ch = AirChannel(segments=(((0, 10, 0, 10, 30, 40), "x"),), inlet=(-1, 0, 0, 10, 30, 40),
                        outlet=(10, 11, 0, 10, 30, 40), inlet_open_area=10.0, past=SZBK07)
        self.assertTrue(fails(C.check_fans(Assembly(channel=ch)), "no exhaust fan position"))
        intake = fan_at(20.0, exhaust=False)
        self.assertTrue(fails(C.check_fans(Assembly(channel=ch, fans=[intake])), "no exhaust fan position"))


class CoverChecks(unittest.TestCase):
    SZ = Held(SZBK07, box(-15, 15, -20, 0, 31, 48), held_down=3.0)  # 2 mm under the lid

    def test_a_closed_cover_passes(self):
        self.assertEqual(C.check_cover(Assembly(parts=[cover()], held=[self.SZ])), [])

    def test_a_screen_above_the_lid_s_top_fails(self):
        screen = Exposure(UP, (-6, 6, -6, 6, 53, 53))
        td = Held("t-display", box(-6, 6, -6, 6, 50, 53), exposed=(screen,))
        self.assertTrue(fails(C.check_cover(Assembly(parts=[cover(window=(-6, 6, -6, 6))], held=[td])),
                              "not below the cover's top"))
        low = Held("t-display", box(-6, 6, -6, 6, 45, 49), exposed=(Exposure(UP, (-6, 6, -6, 6, 49, 49)),))
        self.assertEqual(C.check_cover(Assembly(parts=[cover(window=(-6, 6, -6, 6))], held=[low])), [])

    def test_a_held_down_part_with_room_to_lift_fails(self):
        high = Held(SZBK07, box(-15, 15, -20, 0, 31, 40), held_down=3.0)  # 10 mm under the lid
        self.assertTrue(fails(C.check_cover(Assembly(parts=[cover()], held=[high])), "lift out over its stops"))
        ribbed = cover()
        ribbed.shape = ribbed.shape + box(-15, -10, -20, -15, 41, 50.5)
        self.assertEqual(C.check_cover(Assembly(parts=[ribbed], held=[high])), [])

    def test_an_opening_over_the_szbk07_fails(self):
        asm = Assembly(parts=[cover(hole=(-5, -3, -10, -8))], held=[self.SZ])
        self.assertTrue(fails(C.check_cover(asm), "an opening in it above the szbk07"))

    def test_a_cover_that_hooks_under_a_part_fails_to_lift_off(self):
        # A hook on the cover under a ledge on the deck: clear in place, caught when lifted.
        hooked = cover()
        hooked.shape = hooked.shape + box(10, 20, 5, 9, 33, 35) + box(19, 20, 5, 9, 35, 50.5)
        deck_ = Part("deck", box(-22, 22, -32, 32, 27, 30) + box(5, 8, 5, 9, 30, 40) + box(5, 15, 5, 9, 38, 40),
                     "deck", UP, "black")
        self.assertLess(C.common(hooked.shape, deck_.shape), C.VOLUME_TOL)
        f = C.check_cover(Assembly(parts=[deck_, hooked], held=[self.SZ]))
        self.assertTrue(fails(f, "does not lift straight off", "deck"), f)
        self.assertEqual(C.check_cover(Assembly(parts=[deck_, cover()], held=[self.SZ])), [])

    def test_a_catch_between_the_lift_steps_fails(self):
        # The ledge 2.6 mm over the hook: lifting 2 mm clears it, 5 mm passes it.
        hooked = cover()
        hooked.shape = hooked.shape + box(10, 20, 5, 9, 33, 35) + box(19, 20, 5, 9, 35, 50.5)
        deck_ = Part("deck", box(-22, 22, -32, 32, 27, 30) + box(5, 8, 5, 9, 30, 37.9) + box(5, 15, 5, 9, 37.6, 37.9),
                     "deck", UP, "black")
        f = C.check_cover(Assembly(parts=[deck_, hooked], held=[self.SZ]))
        self.assertTrue(fails(f, "something lies above it", "does not lift straight off"), f)


def railed_cover() -> Part:
    """The test cover with a T rail on its +x wall, a 2 mm neck and a 5 mm
    head, from z 36 to 49."""
    c = cover()
    c.shape = c.shape + box(21.5, 23, -1, 1, 36, 49) + box(23, 24, -2.5, 2.5, 36, 49)
    return c


def rail_bumper(lift: float = 0.0, slide: float = 0.2, top: float = 55.0, role: str = "bumper") -> Part:
    """A block round the rail, `slide` clear of it and of the wall, its slot
    closed `slide` over the rail's top."""
    slot = box(21, 23 + slide, -1 - slide, 1 + slide, 37, 49 + slide) \
        + box(23 - slide, 24 + slide, -2.5 - slide, 2.5 + slide, 37, 49 + slide)
    shape = box(22 + slide, 26.4, -4.4, 4.4, 38, top) - slot
    return Part("bumper", shape.moved(Location((0, 0, lift))), role, DOWN, "yellow", outward=(1.0, 0.0, 0.0))


RETAINER = Screw("bumper screw", (26.4, 0.0, 45.0), (1.0, 0.0, 0.0), shank=(2.0, 4.0))  # through it into the rail


class Bumpers(unittest.TestCase):
    def asm(self, *parts: Part, screws=(RETAINER,)) -> Assembly:
        return Assembly(parts=[railed_cover(), *parts], screws=list(screws))

    def test_a_bumper_on_its_rail_rests_slides_and_lifts_off(self):
        f = C.check_bumpers(self.asm(rail_bumper()))
        self.assertFalse(fails(f, "bumper:") or fails(f, "bumper copy"), f)

    def test_a_bumper_without_its_screw_or_with_a_short_one_fails(self):
        self.assertTrue(fails(C.check_bumpers(self.asm(rail_bumper(), screws=())), "bumper: no screw holds it"))
        short = Screw("bumper screw", (26.4, 0.0, 45.0), (1.0, 0.0, 0.0), shank=(2.0, 2.0))  # ends in the slot
        self.assertTrue(fails(C.check_bumpers(self.asm(rail_bumper(), screws=(short,))), "bumper: no screw holds it"))

    def test_a_bumper_off_its_stop_fails(self):
        self.assertTrue(fails(C.check_bumpers(self.asm(rail_bumper(lift=2.0))), "nothing stops it"))

    def test_a_bumper_under_something_cannot_be_replaced(self):
        f = C.check_bumpers(self.asm(rail_bumper(), block(22, 27, -5, 5, 60, 62)))
        self.assertTrue(fails(f, "bumper: lifted", "meets block", "does not come off"), f)

    def test_a_bumper_without_a_sliding_fit_fails(self):
        self.assertTrue(fails(C.check_bumpers(self.asm(rail_bumper(slide=0.0))), "no sliding fit"))

    def test_bumpers_must_stand_proud_all_round_and_above(self):
        f = C.check_bumpers(self.asm(rail_bumper()))
        self.assertTrue(fails(f, "towards 180°", "past the cover"), f)  # one bumper, on the +x side only
        self.assertFalse(fails(f, "towards 0°"), f)
        self.assertFalse(fails(f, "upwards"), f)
        f = C.check_bumpers(self.asm(rail_bumper(top=52.5)))
        self.assertTrue(fails(f, "upwards (their tops)", "past the cover"), f)
        head = Screw("cover screw", (0.0, 0.0, 54.5), UP)
        f = C.check_bumpers(Assembly(parts=[railed_cover(), rail_bumper()], screws=[head, RETAINER]))
        self.assertTrue(fails(f, "upwards (their tops)", "past the cover screw"), f)
        self.assertFalse(fails(f, "past the bumper screw"), f)
        screen = Held("t-display", box(-6, 6, -6, 6, 45, 54.5), exposed=(Exposure(UP, (-6, 6, -6, 6, 54.5, 54.5)),))
        f = C.check_bumpers(Assembly(parts=[railed_cover(), rail_bumper()], held=[screen]))
        self.assertTrue(fails(f, "upwards (their tops)", "past the t-display"), f)

    def test_the_cover_lifts_off_with_its_bumpers(self):
        held = [CoverChecks.SZ]
        self.assertEqual(C.check_cover(Assembly(parts=[railed_cover(), rail_bumper()], held=held)), [])
        f = C.check_cover(Assembly(parts=[railed_cover(), rail_bumper(role="holder")], held=held))
        self.assertTrue(fails(f, "does not lift straight off"), f)

    def test_a_bumper_over_a_vent_fails(self):
        asm = Fans().asm()
        asm.parts.append(Part("bumper", box(22.8, 24, -16, 16, 28, 64), "bumper", DOWN, "yellow"))
        self.assertTrue(fails(C.check_fans(asm), "or a bumper over it"))


class Captive(unittest.TestCase):
    """A holder in a 1.5 mm pocket in a deck, a board on it, a lid over it."""

    deck_ = Part("deck", box(-20, 20, -20, 20, 26, 29) - box(-5.2, 17.2, -5.2, 5.2, 27.5, 30), "deck", UP, "black")
    board = Held("board", box(-6, 6, -6, 6, 32.05, 33.65))
    lid = Part("lid", box(-20, 20, -20, 20, 34.5, 36), "cover", DOWN, "black")

    def holder(self, carries=("board",), copies=()) -> Part:
        return Part("holder", box(-5, 5, -5, 5, 27.55, 32), "holder", UP, "white", copies=copies, held_down=1.5,
                    carries=carries)

    def test_held_down_through_what_it_carries_passes(self):
        self.assertEqual(C.check_captive(Assembly(parts=[self.deck_, self.holder(), self.lid], held=[self.board])), [])

    def test_nothing_over_it_fails(self):
        f = C.check_captive(Assembly(parts=[self.deck_, self.holder()], held=[self.board]))
        self.assertTrue(fails(f, "holder: nothing stops it", "it can lift out"), f)

    def test_a_stack_rises_by_its_gaps_added_up(self):
        # The lower holder meets the upper one after 0.6 mm, which meets the
        # lid after 1.0 mm more: 1.6 mm, more than the 1.5 mm pocket.
        upper = Part("upper", box(-5, 5, -5, 5, 32.6, 34), "holder", UP, "white", held_down=1.5)
        for lid_z, lifts in ((35.0, True), (34.8, False)):
            lid = Part("lid", box(-20, 20, -20, 20, lid_z, 36), "cover", DOWN, "black")
            f = C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=()), upper, lid]))
            self.assertEqual(fails(f, "holder: it rises 1.60 mm") if lifts else f, True if lifts else [], f)
        self.assertFalse(fails(f, "upper"), f)

    def test_a_copy_with_nothing_over_it_fails(self):
        asm = Assembly(parts=[self.deck_, self.holder(copies=(Location((12, 0, 0)),)), self.lid], held=[self.board])
        asm.parts[2] = Part("lid", box(-20, 6, -20, 20, 34.5, 36), "cover", DOWN, "black")
        f = C.check_captive(asm)
        self.assertTrue(fails(f, "holder copy 2"), f)
        self.assertFalse(fails(f, "holder:"), f)

    def test_parts_already_touching_have_no_room_to_rise(self):
        # Contact is a clearance of 0, alone and in a stack.
        touching = Part("lid", box(-20, 20, -20, 20, 32, 36), "cover", DOWN, "black")
        self.assertEqual(C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=()), touching])), [])
        upper = Part("upper", box(-5, 5, -5, 5, 32, 34), "holder", UP, "white", held_down=1.5)
        lid = Part("lid", box(-20, 20, -20, 20, 34, 36), "cover", DOWN, "black")
        self.assertEqual(C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=()), upper, lid])), [])

    def test_a_carried_part_left_out_is_ignored(self):
        # An optional fan not fitted is no failure; a held part it does not
        # carry (the board here) counts as fixed and stops it.
        f = C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=("fan",)), self.lid], held=[self.board]))
        self.assertEqual(f, [])
        f = C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=("fan",)), self.lid]))
        self.assertTrue(fails(f, "holder: it rises 2.50 mm", "can lift out"), f)  # up to the lid
        f = C.check_captive(Assembly(parts=[self.deck_, self.holder(carries=("fan",))]))
        self.assertTrue(fails(f, "holder: nothing stops it"), f)


class Supports(unittest.TestCase):
    """A board with a bare spot in its underside parts, on a holder with a
    pad in the bare spot and lips beyond its ends."""

    REGION = (10.0, 20.0, 5.0, 15.0)

    def board(self, under: bool = True) -> Held:
        shape = box(0, 40, 0, 20, 50, 51)
        if under:
            shape = shape + (box(0, 40, 0, 20, 47, 50) - box(*self.REGION, 46, 50.5))
        return Held("board", shape)

    def holder(self, pad=(10.3, 19.7, 5.3, 14.7), pad_top=49.95, lips=True) -> Part:
        shape = box(-2, 42, -2, 22, 44, 45) + box(*pad, 44.5, pad_top)
        if lips:
            shape = shape + box(0, 40, 20.3, 21.9, 44.5, 52) + box(0, 40, -1.9, -0.3, 44.5, 52)
        return Part("holder", shape, "holder", UP, "white")

    def asm(self, held=None, part=None) -> Assembly:
        return Assembly(parts=[part or self.holder()], held=[held or self.board()],
                        supports=[Support("holder", "board", 50.0, (self.REGION,))])

    def test_a_pad_in_the_bare_spot_with_lips_passes(self):
        self.assertEqual(C.check_supports(self.asm()), [])

    def test_a_pad_short_of_the_board_fails(self):
        self.assertTrue(fails(C.check_supports(self.asm(part=self.holder(pad_top=49.5))), "bears on the board over 0.0"))

    def test_a_board_above_its_declared_face_fails(self):
        raised = Held("board", self.board().shape.moved(Location((0, 0, 0.5))))
        self.assertTrue(fails(C.check_supports(self.asm(held=raised)), "bears on the board over 0.0"))

    def test_bearing_outside_the_bare_spot_fails(self):
        f = C.check_supports(self.asm(part=self.holder(pad=(5.0, 19.7, 5.3, 14.7))))
        self.assertTrue(fails(f, "outside its regions"), f)

    def test_a_board_free_to_slide_fails(self):
        f = C.check_supports(self.asm(held=self.board(under=False), part=self.holder(lips=False)))
        self.assertTrue(fails(f, "moved (1.0, 0.0)", "not located") and fails(f, "moved (0.0, -1.0)", "not located"), f)
        f = C.check_supports(self.asm(part=self.holder(lips=False)))
        self.assertEqual(f, [], "the pad in the bare spot locates it")

    def test_a_missing_part_fails(self):
        asm = self.asm()
        asm.supports = [Support("cradle", "board", 50.0, (self.REGION,))]
        self.assertTrue(fails(C.check_supports(asm), "no part 'cradle'"))


class Build(unittest.TestCase):
    def test_the_empty_part_set_passes_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            self.assertEqual(build.main([], outdir=Path(tmp), make=Assembly), 0)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_a_failing_check_writes_nothing(self):
        bad = Assembly(parts=[deck(), block(60, 70, 40, 50, 30, 35)])
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()) as out:
            self.assertEqual(build.main([], outdir=Path(tmp), make=lambda: bad), 1)
            self.assertEqual(list(Path(tmp).rglob("*")), [])
        self.assertIn("keep-clear", out.getvalue())

    def test_a_failing_reread_publishes_nothing(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()) as out:
            root = Path(tmp)
            (root / "print").mkdir()
            (root / "print" / "old.stl").write_text("solid old\nendsolid old\n")
            with mock.patch.object(build, "verify_export", return_value=["export: deck.stl is wrong"]):
                self.assertEqual(build.main([], outdir=root, make=lambda: Assembly(parts=[deck()])), 1)
            files = sorted(str(p.relative_to(root)) for p in root.rglob("*"))
            self.assertEqual(files, ["print", "print/old.stl"])
        self.assertIn("deck.stl is wrong", out.getvalue())

    def test_a_passing_build_publishes_and_removes_stale_files(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            root = Path(tmp)
            (root / "print").mkdir()
            (root / "print" / "old.stl").write_text("solid old\nendsolid old\n")
            self.assertEqual(build.main([], outdir=root, make=lambda: Assembly(parts=[deck()])), 0)
            files = sorted(str(p.relative_to(root)) for p in root.rglob("*") if p.is_file())
            self.assertEqual(files, ["print/deck.stl", "step/deck.step"])
            self.assertEqual(sorted(p.name for p in root.iterdir()), ["print", "step"])

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

    def test_the_cover_is_one_object_of_two_parts(self):
        from build123d import Mesher

        parts = fixture_parts()
        cov = parts[2]
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            export_parts(parts, out)
            xml = model_xml(out / "print" / "cover.3mf")
            self.assertEqual(xml.count("<item "), 1, "one build item")
            self.assertEqual(xml.count("<component "), 2, "the body and the inlays as its parts")
            # Written as two separate objects (build123d's default), it fails.
            mesher = Mesher()
            for shape in (cov.shape, cov.inlays):
                mesher.add_shape(shape)
            mesher.write(str(out / "print" / "cover.3mf"))
            f = verify_export(parts, out)
            self.assertTrue(fails(f, "cover.3mf", "not one object of two parts"), f)

    def test_a_deck_without_its_holes_fails_on_reread(self):
        parts = [deck(holes=M.FRAME_INSERTS[:7])]
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            export_parts(parts, out)
            f = verify_export(parts, out)
            self.assertTrue(fails(f, "deck.step", "no clear M3 hole"), f)


class RouteEnds(unittest.TestCase):
    BOARD = Held("board", box(0, 20, 0, 20, 30, 32))
    EXT = Terminal("ext", "board", (17, 20, 8, 12, 30, 32), (1.0, 0.0, 0.0))

    def asm(self, route_box, end=EXT) -> Assembly:
        return Assembly(held=[self.BOARD], routes=[Route.of("ext run", [route_box], ends=(end,))])

    def test_a_route_from_the_wire_side_passes(self):
        self.assertEqual(C.check_route_ends(self.asm((20.05, 30, 6, 14, 29, 34))), [])

    def test_a_route_that_stops_short_fails(self):
        f = C.check_route_ends(self.asm((22, 30, 6, 14, 29, 34)))
        self.assertTrue(fails(f, "ext run", "does not reach ext from +x"), f)

    def test_a_route_from_another_side_fails(self):
        f = C.check_route_ends(self.asm((0, 20, 20.05, 30, 29, 34)))
        self.assertTrue(fails(f, "does not reach ext from +x"), f)

    def test_a_route_beside_the_terminal_fails(self):
        f = C.check_route_ends(self.asm((20.05, 30, 13, 20, 29, 34)))
        self.assertTrue(fails(f, "does not reach ext"), f)

    def test_a_terminal_off_its_part_or_on_no_part_fails(self):
        off = Terminal("ext", "board", (21, 24, 8, 12, 30, 32), (1.0, 0.0, 0.0))
        self.assertTrue(fails(C.check_route_ends(self.asm((20.05, 30, 6, 14, 29, 34), off)), "is not on board"))
        missing = Terminal("ext", "nothing", (17, 20, 8, 12, 30, 32), (1.0, 0.0, 0.0))
        self.assertTrue(fails(C.check_route_ends(self.asm((20.05, 30, 6, 14, 29, 34), missing)), "no held part"))

    def test_a_side_off_the_axes_fails(self):
        bad = Terminal("ext", "board", (17, 20, 8, 12, 30, 32), (0.7, 0.7, 0.0))
        self.assertTrue(fails(C.check_route_ends(self.asm((20.05, 30, 6, 14, 29, 34), bad)), "not an axis"))

    def test_a_route_in_separate_pieces_fails(self):
        entry = Terminal("entry", None, (25, 27, 8, 12, 29, 30), (0.0, 0.0, -1.0))
        apart = Route.of("ext run", [(20.05, 23, 6, 14, 29, 34), (24, 30, 6, 14, 29, 34)], ends=(self.EXT, entry))
        f = C.check_route_ends(Assembly(held=[self.BOARD], routes=[apart]))
        self.assertTrue(fails(f, "2 separate pieces"), f)

    def test_a_free_space_end_must_lie_in_the_route(self):
        entry = Terminal("entry", None, (25, 27, 8, 12, 29, 30), (0.0, 0.0, -1.0))
        self.assertEqual(C.check_route_ends(self.asm((20.05, 30, 6, 14, 29, 34), entry)), [])
        self.assertTrue(fails(C.check_route_ends(self.asm((20.05, 26, 6, 14, 29, 34), entry)), "does not reach entry"))


class DiodeRouteWidth(unittest.TestCase):
    def test_a_box_narrower_than_the_sleeve_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        wide = Route.of(LM2596_OUTPUT_ROUTE, [(0, 40, -4, 4, 30, 38)])
        self.assertEqual(C.check_routes(Assembly(held=[diode], routes=[wide])), [])
        narrow = Route.of(LM2596_OUTPUT_ROUTE, [(0, 40, -4, 4, 30, 38), (40, 60, -2, 2, 30, 38)])
        f = C.check_routes(Assembly(held=[diode], routes=[narrow]))
        self.assertTrue(fails(f, "too narrow for the diode"), f)

    def test_a_junction_narrower_than_the_sleeve_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        turn = Route.of(LM2596_OUTPUT_ROUTE, [(0, 30, -3, 3, 30, 37), (24, 30, -3, 40, 30, 37)])
        self.assertEqual(C.check_routes(Assembly(held=[diode], routes=[turn])), [])
        pinched = Route.of(LM2596_OUTPUT_ROUTE, [(0, 30, -3, 3, 30, 37), (29, 35, 2.5, 40, 36, 43)])
        f = C.check_routes(Assembly(held=[diode], routes=[pinched]))
        self.assertTrue(fails(f, "do not join into one run"), f)

    def test_a_route_not_made_of_boxes_fails(self):
        diode = Held(SCHOTTKY, box(5, 25, -3, 3, 31, 37), route=LM2596_OUTPUT_ROUTE)
        shape_only = Route(LM2596_OUTPUT_ROUTE, box(0, 40, -4, 4, 30, 38))
        f = C.check_routes(Assembly(held=[diode], routes=[shape_only]))
        self.assertTrue(fails(f, "not made of boxes"), f)


class CoverOffRoutes(unittest.TestCase):
    def test_the_cover_may_cross_a_cover_off_route_but_nothing_else(self):
        usb = Route.of("usb-c cable", [(-30, -15, -2, 2, 35, 40)], cover_off=True)
        asm = Assembly(parts=[cover()], routes=[usb])
        self.assertEqual(C.check_routes(asm), [])
        asm.parts.append(block(-17, -16, -1, 1, 36, 38))
        self.assertTrue(fails(C.check_routes(asm), "block enters the route 'usb-c cable'"))
        asm.routes[0] = Route.of("usb-c cable", [(-30, -15, -2, 2, 35, 40)])
        self.assertTrue(fails(C.check_routes(asm), "cover enters"))


class Sensors(unittest.TestCase):
    """The IMU pad, the pass-through and the end-stop entry on a small set:
    a deck with two bosses, the IMU on them, an SZBK07 and a board."""

    @classmethod
    def setUpClass(cls):
        cls.frame = C.FrameMap()

    def asm(self, imu_at=(-10.0, 10.0, 9.0, 24.0), szbk07_y1=-1.5, bosses=((-7.5, 16.5), (7.5, 16.5)),
            pilot=True, route=True, openings=()) -> Assembly:
        x0, x1, y0, y1 = imu_at
        shape = box(-24, 24, -60, 60, Z0, Z0 + 3)
        for bx, by in bosses:
            shape = shape + C.cylinder(2.5, (bx, by, Z0 + 2.5), (0, 0, 1), 3.5)
            if pilot:
                shape = shape - C.cylinder(0.8, (bx, by, Z0 + 1), (0, 0, 1), 6)
        top = Z0 + 6
        board = Held(SERVO_2040, box(-20, 20, 6, 40, 40, 42))
        held = [Held(IMU, box(x0, x1, y0, y1, top + 0.05, top + 3)), Held(SZBK07, box(-20, 20, -40, szbk07_y1, 30, 50)),
                board]
        j6 = Terminal("j6", SERVO_2040, (-10, 10, 6, 8, 40, 42), (0.0, -1.0, 0.0))
        imu_end = Terminal("imu wires", IMU, (x0, x1, y0, y0 + 2, top + 0.05, top + 3), (0.0, -1.0, 0.0))
        routes = [Route.of("imu to j6", [(x0, x1, 0, y0, top, top + 4), (-10, 10, 0, 5.9, top, 44)],
                           ends=(imu_end, j6))] if route else []
        return Assembly(parts=[Part("deck", shape, "deck", UP, "black")], held=held, routes=routes,
                        openings=list(openings), imu_pad=ImuPad("deck", bosses, top, 2.5, 0.8))

    def test_the_pad_passes(self):
        self.assertEqual(C.check_sensors(self.asm(), self.frame), [])

    def test_too_far_from_the_centre_fails(self):
        f = C.check_sensors(self.asm(imu_at=(-10.0, 10.0, 16.0, 31.0), bosses=((-7.5, 23.5), (7.5, 23.5))), self.frame)
        self.assertTrue(fails(f, "from the centre"), f)

    def test_too_near_the_szbk07_fails(self):
        f = C.check_sensors(self.asm(szbk07_y1=0.0), self.frame)
        self.assertTrue(fails(f, "from the szbk07's outline"), f)

    def test_overlapping_a_held_part_fails(self):
        asm = self.asm()
        asm.held.append(Held("fuse holder", box(5, 15, 20, 30, 30, 40)))
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "lsm6dso: overlaps fuse holder"))

    def test_a_missing_boss_or_pilot_fails(self):
        asm = self.asm(bosses=((-7.5, 16.5),))
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "1 bosses, not two"))
        asm.imu_pad.bosses = ((-7.5, 16.5), (7.5, 16.5))
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "no boss at (7.5, 16.5)"))
        self.assertTrue(fails(C.check_sensors(self.asm(pilot=False), self.frame), "no pilot hole"))

    def test_a_boss_top_that_does_not_carry_it_fails(self):
        asm = self.asm()
        asm.imu_pad.top -= 1.0
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "does not carry"))

    def test_no_route_to_j6_fails(self):
        self.assertTrue(fails(C.check_sensors(self.asm(route=False), self.frame), "no route to the servo 2040's J6"))

    def test_an_imu_without_a_pad_or_a_pad_without_an_imu_fails(self):
        asm = self.asm()
        asm.imu_pad = None
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "no pad"))
        asm = self.asm()
        asm.held = [h for h in asm.held if h.name != IMU]
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "no held part 'lsm6dso'"))

    def test_a_pass_through_over_open_frame_passes_and_over_the_floor_fails(self):
        open_ = Opening("pass-through", "deck", (-5.0, -1.0, 1.0, 5.0), Z0, Z0 + 3, kind="pass-through", wires=4)
        self.assertEqual(C.check_sensors(self.asm(openings=[open_]), self.frame), [])
        x, y = M.FRAME_FLOOR_INSERTS[0]
        floor = Opening("pass-through", "deck", (x + 3, x + 7, y - 2, y + 2), Z0, Z0 + 3, kind="pass-through", wires=4)
        self.assertTrue(fails(C.check_sensors(self.asm(openings=[floor]), self.frame), "frame is not open"))

    def test_an_opening_too_small_for_its_wires_fails(self):
        small = Opening("pass-through", "deck", (-5.0, -3.5, 1.0, 5.0), Z0, Z0 + 3, kind="pass-through", wires=4)
        self.assertTrue(fails(C.check_sensors(self.asm(openings=[small]), self.frame), "does not take 4 wires"))

    def test_an_entry_needs_a_route_to_the_board(self):
        entry = Opening("end-stop entry", "deck", (22.0, 26.0, 10.0, 20.0), Z0, Z0 + 3, kind="entry", wires=7)
        asm = self.asm(openings=[entry])
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "end-stop entry: no route"))
        header = Terminal("sensor header", SERVO_2040, (17, 20, 10, 20, 40, 42), (1.0, 0.0, 0.0))
        asm.routes.append(Route.of("end-stop wires", [(20.05, 26, 9, 21, Z0 + 3, 44)],
                                   ends=(Terminal("entry", None, (22, 26, 10, 20, Z0 + 3, Z0 + 4), (0, 0, -1)), header)))
        self.assertEqual(C.check_sensors(asm, self.frame), [])
        asm.routes[-1].ends = (asm.routes[-1].ends[0], Terminal("ext", SERVO_2040, header.region, header.side))
        self.assertTrue(fails(C.check_sensors(asm, self.frame), "no route from it to the servo 2040's sensor header"))
        self.assertTrue(fails(C.check_sensors(self.asm(openings=[Opening(
            "end-stop entry", "deck", (22.0, 23.5, 10.0, 20.0), Z0, Z0 + 3, kind="entry", wires=7)]), self.frame),
            "does not take 7 wires"))


README = (Path(__file__).resolve().parent / "README.md").read_text(encoding="utf-8")

# What the README's "Print orientation" cell must say for each print_up: as in
# the file, turned over, or which face lies on the bed.
ORIENTATION_WORDS = {(0.0, 0.0, 1.0): "as in the file", (0.0, 0.0, -1.0): "turn it over",
                     (0.0, 1.0, 0.0): "(−y) on the bed", (0.0, -1.0, 0.0): "(+y) on the bed",
                     (1.0, 0.0, 0.0): "(−x) on the bed", (-1.0, 0.0, 0.0): "(+x) on the bed"}


def readme_parts_mismatches(parts: list[Part], text: str) -> list[str]:
    """Where the README's "Parts and colours" table (file, part, copies,
    colour, print orientation) disagrees with the built parts: each distinct
    part's print file listed once, with its copies, its colour first in the
    colour cell, and its print orientation; and nothing else listed."""
    section = text.split("## Parts and colours", 1)
    body = section[1].split("\n## ", 1)[0] if len(section) == 2 else ""
    rows = [[c.strip() for c in r.strip().strip("|").split("|")] for r in body.splitlines() if r.startswith("| `")]
    listed = {r[0].strip("`"): r for r in rows}
    found = [f"README: {name} is listed twice" for name in listed if [r[0] for r in rows].count(f"`{name}`") > 1]
    for part in parts:
        name = f"print/{print_name(part)}"
        row = listed.pop(name, None)
        if row is None:
            found.append(f"README: {name} is not listed")
            continue
        if len(row) != 5:
            found.append(f"README: {name} has {len(row)} columns, not 5")
            continue
        if row[2] != str(1 + len(part.copies)):
            found.append(f"README: {name} copies {row[2]}, built {1 + len(part.copies)}")
        if not row[3].startswith(part.colour):
            found.append(f"README: {name} colour {row[3]!r}, built {part.colour}")
        if part.inlays is not None and "inlays yellow" not in row[3]:
            found.append(f"README: {name} colour {row[3]!r} names no yellow inlays")
        words = ORIENTATION_WORDS.get(tuple(float(c) for c in part.print_up))
        if words is None or words not in row[4]:
            found.append(f"README: {name} orientation {row[4]!r}, built up {part.print_up}")
    found += [f"README: {name} lists no built part" for name in listed]
    return found


class Layout(unittest.TestCase):
    """The layout as built."""

    @classmethod
    def setUpClass(cls):
        cls.asm = L.assembly()

    def test_every_check_passes(self):
        self.assertEqual(C.run_checks(self.asm), [])

    def test_the_part_set(self):
        self.assertEqual(sorted(p.name for p in self.asm.parts),
                         ["bumper", "cover", "deck", "exhaust-fan-holder-left", "exhaust-fan-holder-right", "imu-pad",
                          "intake-fan-holder", "lm2596-standoff", "spacer", "t-display-holder"])
        self.assertEqual(sorted(h.name for h in self.asm.held),
                         sorted(["szbk07", "servo 2040", "lm2596", "t-display", "lsm6dso", "xt60", "fuse holder",
                                 SCHOTTKY, *FANS]))
        self.assertEqual(len(self.asm.parts[1].instances()), 4, "a spacer at each floor insert")

    def test_held_parts_and_routes_lie_inside_the_cover(self):
        for h in self.asm.held:
            for x, y, _ in C.vertices(h.shape):
                self.assertTrue(L.interior_ok((x, x, y, y, 0, 0)), f"{h.name} at ({x:.2f}, {y:.2f})")
        for r in self.asm.routes:
            if r.cover_off or r.name == "battery plug":
                continue
            for b in r.boxes:
                self.assertTrue(L.interior_ok(b), f"{r.name} {b}")

    def test_the_battery_plug_stays_inside_the_legs_region(self):
        plug = next(r for r in self.asm.routes if r.name == "battery plug")
        far = max(math.hypot(x, y) for b in plug.boxes for x in b[:2] for y in b[2:4])
        self.assertLessEqual(far, C.KEEP_CLEAR_R)

    def test_the_spacers_stand_on_the_floor(self):
        frame = C.FrameMap()
        for x, y in M.FRAME_FLOOR_INSERTS:
            r = L.SPACER_OD / 2
            for dx, dy in ((r - 0.5, 0), (-r + 0.5, 0), (0, r - 0.5), (0, -r + 0.5)):
                top = frame.highest(x + dx - 0.2, x + dx + 0.2, y + dy - 0.2, y + dy + 0.2)
                self.assertIn(top, (None, M.FRAME_FLOOR_Z), f"spacer at ({x}, {y}) over z {top}")

    def test_every_route_reaches_its_ends_and_the_sensors_are_provided(self):
        self.assertEqual(C.check_route_ends(self.asm), [])
        self.assertEqual(C.check_sensors(self.asm), [])
        kinds = sorted((o.kind, o.wires) for o in self.asm.openings if o.kind in ("entry", "pass-through"))
        self.assertEqual(kinds, [("entry", 7), ("pass-through", 4)])
        self.assertTrue(any(e.name == "sensor header" for r in self.asm.routes for e in r.ends))

    def test_the_board_s_wire_sides_follow_the_measurements(self):
        # A wire side measured differently on the board turns its terminal,
        # and the route laid for the old side no longer reaches it.
        held = [L.servo2040(), L.szbk07()]
        for name, side in (("S2040_EXT_WIRE_SIDE", "-y"), ("S2040_SENSOR_WIRE_SIDE", "+y")):
            old = getattr(M, name)
            try:
                setattr(M, name, side)
                t = L.s2_terminals()
                runs = [r for r in L.routes(t) if r.name in ("szbk07 output to ext", "end-stop wires")]
                f = C.check_route_ends(Assembly(held=held, routes=runs))
                self.assertTrue(fails(f, "does not reach", f"from {side}"), f"{name}: {f}")
            finally:
                setattr(M, name, old)

    def test_the_lm2596_is_screwed_down_and_the_diode_tied_in_its_route(self):
        screws = [s for s in self.asm.screws if s.name.startswith("lm2596 screw")]
        self.assertEqual(len(screws), len(M.LM2596_HOLES))
        diode = next(h for h in self.asm.held if h.name == SCHOTTKY)
        bb = C.bbox6(diode.shape)
        ties = [o for o in self.asm.openings if o.name.startswith("diode tie")]
        self.assertEqual(len(ties), 2)
        for o in ties:
            self.assertTrue(bb[0] <= o.footprint[0] and o.footprint[1] <= bb[1], "under the diode's length")

    def part(self, name: str) -> Part:
        return next(p for p in self.asm.parts if p.name == name)

    def test_the_readme_lists_every_printed_file(self):
        self.assertEqual(readme_parts_mismatches(self.asm.parts, README), [])

    def test_a_readme_missing_a_part_or_with_a_wrong_colour_fails(self):
        dropped = "\n".join(r for r in README.splitlines() if not r.startswith("| `print/spacer.stl`"))
        self.assertTrue(fails(readme_parts_mismatches(self.asm.parts, dropped), "print/spacer.stl", "not listed"))
        recoloured = README.replace("| `print/bumper.stl` | a corner guard (bumper) | 4 | yellow |",
                                    "| `print/bumper.stl` | a corner guard (bumper) | 4 | black |")
        self.assertNotEqual(recoloured, README)
        self.assertTrue(fails(readme_parts_mismatches(self.asm.parts, recoloured), "print/bumper.stl", "colour"))
        turned_over = README.replace("lid-down: turn it over", "lid-down, as in the file")
        self.assertTrue(fails(readme_parts_mismatches(self.asm.parts, turned_over), "print/cover.3mf", "orientation"))

    def test_the_cover_prints_lid_down_and_its_screen_sits_below_its_top(self):
        cov = self.part("cover")
        self.assertEqual(cov.print_up, (0.0, 0.0, -1.0))
        self.assertEqual(C.check_overhangs(Assembly(parts=[cov])), [])
        top = cov.shape.bounding_box().max.Z
        screen = next(e for h in self.asm.held if h.name == "t-display" for e in h.exposed if e.must_show)
        self.assertLess(screen.region[5], top - L.LID_T, "the screen sits below the lid")
        self.assertEqual(C.check_cover(self.asm), [])

    def test_the_lid_is_closed_over_the_szbk07_and_holds_it_down(self):
        sz = next(h for h in self.asm.held if h.name == SZBK07)
        b = C.bbox6(sz.shape)
        lid = box(b[0], b[1], b[2], b[3], L.LID_Z0, L.LID_Z1)
        cov = self.part("cover")
        # Closed but for the panel lines engraved in its top.
        lines = C.common(L.panel_lines(), lid)
        self.assertAlmostEqual(C.common(cov.shape, lid) + C.common(cov.inlays, lid) + lines, C.volume(lid), delta=0.5)
        self.assertGreaterEqual(sz.held_down, 3.0)
        gap = min(f.bounding_box().min.Z for f in (cov.shape & box(*b[:4], b[5], L.LID_Z0)).solids()) - b[5]
        self.assertLess(gap, sz.held_down, "the hold-down blocks come nearer than its stops are tall")

    def test_the_cover_is_screwed_into_inserts_in_the_deck_s_posts(self):
        deck_, cov = self.part("deck").shape, self.part("cover").shape
        screws = [s for s in self.asm.screws if s.name.startswith("cover screw")]
        self.assertGreaterEqual(len(screws), 3)
        for s in screws:
            x, y, _ = s.head
            self.assertEqual(s.axis, UP)
            hole = C.cylinder(L.M3_D / 2 - 0.05, (x, y, L.LID_Z0 - 0.5), (0, 0, 1), L.LID_T + 1)
            self.assertLess(C.common(cov, hole), C.VOLUME_TOL, "a clear hole through the lid")
            insert = C.cylinder(L.M3_INSERT_D / 2 - 0.05, (x, y, L.LID_Z0 - L.M3_INSERT_L), (0, 0, 1), L.M3_INSERT_L - 0.1)
            self.assertLess(C.common(deck_, insert), C.VOLUME_TOL, "an insert hole in the post")
            wall = C.cylinder(L.POST_D / 2 - 0.3, (x, y, L.LID_Z0 - 2), (0, 0, 1), 1) - C.cylinder(
                L.M3_INSERT_D / 2 + 0.05, (x, y, L.LID_Z0 - 3), (0, 0, 1), 3)
            self.assertGreater(C.common(deck_, wall), 0.9 * C.volume(wall), "a post round the insert")

    def test_the_fans_and_the_vents(self):
        exhaust, intake = (next(f for f in self.asm.fans if f.exhaust == e) for e in (True, False))
        self.assertEqual(C.check_fans(self.asm), [])
        self.assertLess(intake.origin[2] + intake.size[1] / 2, exhaust.origin[2] + exhaust.size[1] / 2,
                        "the inlet is low and the outlet high")
        self.assertLess(intake.outward[0] * exhaust.outward[0], 0, "on opposite sides")
        inlet = next(v for v in self.asm.vents if v.name == "inlet")
        self.assertEqual(self.asm.channel.inlet_open_area, inlet.open_area)
        sz = C.bbox6(next(h for h in self.asm.held if h.name == SZBK07).shape)
        self.assertTrue(intake.origin[0] < sz[0] and sz[1] < exhaust.origin[0], "the SZBK07 between them")
        self.assertEqual(C.check_air_channel(self.asm), [])

    def test_without_its_lips_the_exhaust_fan_slides_in(self):
        # The lips' first version (cheeks only) let the fan slide in off its
        # columns; the slide check catches it.
        old = L.EXH_LIP_OVER
        try:
            L.EXH_LIP_OVER = -(L.FIT + 0.4)  # the lips end beyond the fan's ends
            parts = [L.deck(), L.intake_holder()[0], *(p for p, _ in L.exhaust_holders())]
            asm = Assembly(parts=parts, held=[h for h in self.asm.held if h.name in FANS], fans=self.asm.fans)
            self.assertTrue(fails(C.check_fans(asm), "exhaust fan", "slides 1.0 mm inwards"))
        finally:
            L.EXH_LIP_OVER = old

    def test_the_lid_keeps_the_intake_fan_within_its_lips(self):
        # Without the cover's block over it, the fan could lift out over its
        # holder's lips upside down and slide in (the first version).
        intake = next(h for h in self.asm.held if h.name == FANS[1])
        self.assertEqual(intake.held_down, L.FAN_HOLD_H)
        cov = self.part("cover")
        old = L.INTAKE_HOLD_GAP
        try:
            L.INTAKE_HOLD_GAP = L.FAN_HOLD_H + 1.0
            asm = Assembly(parts=[L.cover()], held=[intake])
            self.assertTrue(fails(C.check_cover(asm), "intake fan", "lift out over its stops"))
        finally:
            L.INTAKE_HOLD_GAP = old
        self.assertEqual(C.check_cover(Assembly(parts=[cov], held=[intake])), [])

    def test_a_filled_outlet_slot_fails(self):
        cov = self.part("cover")
        outlet = next(v for v in self.asm.vents if v.name == "outlet")
        ov0, _, oz0, oz1 = L.outlet_span()
        u0, u1 = L.EXH_U1, L.BOX_DIAG / L.SQRT2 + 1
        plug = turned((u0, u1, ov0, ov0 + L.VENT_SLOT_W, oz0, oz1), L.DIAG)
        filled = replace(cov, shape=cov.shape + plug)
        asm = replace(self.asm, parts=[filled if p.name == "cover" else p for p in self.asm.parts])
        f = C.check_air_channel(asm)
        self.assertTrue(fails(f, "vent outlet leaves 312", "less than the inlet's 336"), f)

    def test_measurements_the_layout_cannot_take_are_named(self):
        self.assertEqual(L.measurement_failures(), [])
        for name, value, words in (("SZBK07_TERMINALS_ON_SHORT_ENDS", False, "lay it out again"),
                                   ("SZBK07_HEATSINK_TOP", M.SZBK07_SIZE[2] + 1, "must include the heatsinks"),
                                   ("LM2596_HOLE_D", 2.0, "too small"),
                                   ("LSM6DSO_HOLE_D", 1.5, "too small"),
                                   ("SCHOTTKY_BODY", (5.2, M.SCHOTTKY_SLEEVE_D + 0.5), "does not fit")):
            with self.subTest(name=name):
                old = getattr(M, name)
                try:
                    setattr(M, name, value)
                    self.assertTrue(fails(L.measurement_failures(), name, words))
                finally:
                    setattr(M, name, old)

    def test_the_inlays_are_flush_with_the_lid(self):
        # Their top is the lid's, they are INLAY_DEPTH deep, and with the body
        # they fill the lid's top layer over them without gap or overlap.
        cov = self.part("cover")
        bi, bb = cov.inlays.bounding_box(), cov.shape.bounding_box()
        self.assertAlmostEqual(bi.max.Z, bb.max.Z, places=6)
        self.assertAlmostEqual(bi.max.Z - bi.min.Z, L.INLAY_DEPTH, places=6)
        self.assertGreater(len(cov.inlays.solids()), len(L.NAME), "the letters and the chevrons")
        self.assertLess(C.common(cov.shape, cov.inlays), C.VOLUME_TOL)
        layer = box(bi.min.X, bi.max.X, bi.min.Y, bi.max.Y, bi.min.Z, bi.max.Z)
        # Engraved panel lines and the cover's screw holes are not filled.
        lines = C.common(L.panel_lines(), layer)
        lines += sum(C.common(L.cyl(L.M3_D / 2, x, y, L.LID_Z0 - 1, L.LID_Z1 + 1), layer) for x, y in L.COVER_SCREWS)
        self.assertAlmostEqual(C.common(cov.shape, layer) + C.common(cov.inlays, layer) + lines, C.volume(layer),
                               delta=0.05)

    def test_the_bumpers(self):
        # One part, copied to the four corners, printed on its top in yellow;
        # their tops above the lid, the screen and the cover's screw heads;
        # each on its corner face's rails, which end in wedges flush with the
        # top chamfer.
        b = self.part("bumper")
        self.assertEqual((b.role, b.colour, b.print_up, len(b.instances())), ("bumper", "yellow", DOWN, 4))
        self.assertEqual(C.check_bumpers(self.asm), [])
        self.assertEqual(C.check_overhangs(Assembly(parts=[b])), [])
        top = min(i.bounding_box().max.Z for i in b.instances())
        screen = next(e for h in self.asm.held if h.name == "t-display" for e in h.exposed if e.must_show)
        heads = [s.head[2] for s in self.asm.screws if s.name.startswith("cover screw")]
        self.assertGreater(top, max([self.part("cover").shape.bounding_box().max.Z, screen.region[5], *heads]))
        for k, inst in enumerate(b.instances()):
            centre = inst.bounding_box().center()
            self.assertGreater(abs(centre.X), L.BOX_X - L.CORNER_HALF, f"bumper {k + 1} at a corner")
            self.assertGreater(abs(centre.Y), L.BOX_Y - L.CORNER_HALF, f"bumper {k + 1} at a corner")
        self.assertEqual(C.volume(L.rails()) > 0, True)
        self.assertLess(C.common(self.part("cover").shape, b.shape), C.VOLUME_TOL)

    def test_a_bumper_too_low_fails(self):
        # Its top under the cover's screw heads: the screws would meet the floor first.
        old = L.BUMPER_Z1
        try:
            L.BUMPER_Z1 = L.LID_Z1 + 1.0
            asm = Assembly(parts=[self.part("cover"), L.bumper()], held=self.asm.held, screws=self.asm.screws)
            self.assertTrue(fails(C.check_bumpers(asm), "upwards (their tops)", "cover screw"))
        finally:
            L.BUMPER_Z1 = old

    def test_the_t_display_holder(self):
        # Printed on its back; it bears on the board's bare spots only and
        # locates it, its screws' paths and the LM2596's are clear, and the lid
        # over the board's pins keeps the board within its lips.
        h = self.part("t-display-holder")
        self.assertEqual((h.role, h.colour, h.print_up), ("holder", "white", (0.0, 1.0, 0.0)))
        self.assertEqual(C.check_overhangs(Assembly(parts=[h])), [])
        self.assertEqual(C.check_supports(self.asm), [])
        self.assertEqual(C.check_screwdriver_paths(self.asm), [])
        for s in self.asm.screws:
            if s.name.startswith("lm2596 screw"):
                self.assertIn("t-display-holder", s.after, "the LM2596 is screwed down before the holder goes on")
        td = next(x for x in self.asm.held if x.name == "t-display")
        self.assertEqual(td.held_down, L.TDH_LIP_H)
        self.assertLess(L.LID_Z0 - C.bbox6(td.shape)[5], L.TDH_LIP_H, "the lid keeps the board within its lips")
        self.assertEqual([s.regions for s in self.asm.supports],
                         [tuple(L.td_box(*b, 0, 0)[:4] for b in M.TD_BARE)])

    def test_the_holders_are_white_and_kept_in_their_pockets(self):
        holders = [p for p in self.asm.parts if p.role == "holder" and p.name != "spacer"]
        self.assertEqual(len(holders), 6)
        for p in holders:
            self.assertEqual(p.colour, "white", p.name)
            self.assertEqual(p.held_down, L.POCKET_DEPTH, p.name)
            self.assertEqual(C.check_overhangs(Assembly(parts=[p])), [], p.name)
            bottom = min(i.bounding_box().min.Z for i in p.instances())
            self.assertAlmostEqual(bottom, L.POCKET_Z + L.TOUCH, places=6, msg=p.name)
        self.assertEqual(C.check_captive(self.asm), [])
        self.assertEqual(self.asm.imu_pad.part, "imu-pad")

    def test_the_holders_stay_in_their_pockets_without_the_fans(self):
        asm = Assembly(parts=self.asm.parts, held=[h for h in self.asm.held if h.name not in FANS])
        self.assertEqual(C.check_captive(asm), [])

    def test_the_bumpers_are_screwed_to_their_rails(self):
        screws = [s for s in self.asm.screws if s.name.startswith("bumper")]
        self.assertEqual(len(screws), 4)
        b = self.part("bumper")
        covers = [self.part("cover")]
        for inst in b.instances():
            self.assertEqual(sum(C.retains(s, inst, covers) for s in screws), 1)

    def test_a_bumper_on_straight_rails_comes_off(self):
        # Its rails' dovetails keep it on: on straight rails it pulls off.
        old = L.RAIL_HEAD
        try:
            L.RAIL_HEAD = L.RAIL_NECK
            asm = Assembly(parts=[L.cover(), L.bumper()], screws=L.bumper_screws())
            self.assertTrue(fails(C.check_bumpers(asm), "bumper: one end, pulled up to", "comes off its rail"))
        finally:
            L.RAIL_HEAD = old

    def test_a_bumper_held_by_one_rail_fails(self):
        # The unscrewed post's jaws cut away: the screwed post's dovetail
        # must not hide that the other end peels off its rail.
        b = self.part("bumper")
        cut = box(L.RAIL_S - 8, L.RAIL_S + 9, -2, L.RAIL_D + L.SLIDE + 0.01, 39, 72).moved(L.corner_location(0))
        asm = Assembly(parts=[self.part("cover"), replace(b, shape=b.shape - cut)], screws=L.bumper_screws())
        f = C.check_bumpers(asm)
        self.assertTrue(fails(f, "bumper: the other end", "comes off its rail"), f)
        self.assertFalse(fails(f, "bumper: one end"), f)

    def test_a_bumper_with_no_way_off_fails(self):
        asm = Assembly(parts=[self.part("cover"), replace(self.part("bumper"), outward=None)])
        self.assertTrue(fails(C.check_bumpers(asm), "bumper copy 3: it declares no way off the cover"))

    def test_the_cover_s_armour(self):
        # Plates stand PLATE_T proud of the side walls; the panel lines and
        # bolt heads are engraved INLAY_DEPTH into the lid's flat top.
        cov = self.part("cover")
        across_x = C.bbox6(cov.shape & box(0, 100, -1, 1, *L.PLATE_Z))  # the right wall's middle
        across_y = C.bbox6(cov.shape & box(-1, 1, -100, 0, *L.PLATE_Z))  # the rear wall's middle
        self.assertAlmostEqual(across_x[1], L.BOX_X + L.PLATE_T, places=3)
        self.assertAlmostEqual(across_y[2], -L.BOX_Y - L.PLATE_T, places=3)
        cut = L.panel_lines()
        self.assertLess(C.common(cov.shape, cut), C.VOLUME_TOL)
        self.assertLess(C.common(cov.inlays, cut), C.VOLUME_TOL)
        self.assertAlmostEqual(cut.bounding_box().min.Z, L.LID_Z1 - L.INLAY_DEPTH, places=6)

    def test_without_the_t_display_holder_the_lm2596_lifts_out(self):
        # The holder's plate keeps the LM2596 and its standoffs down.
        asm = Assembly(parts=[p for p in self.asm.parts if p.name != "t-display-holder"], held=self.asm.held)
        f = C.check_captive(asm)
        self.assertTrue(fails(f, "lm2596-standoff: ", "can lift out") and fails(f, "lm2596-standoff copy 2: "), f)

    def test_an_intake_holder_short_of_the_lid_lifts_out(self):
        old = L.INTAKE_LID_GAP
        try:
            L.INTAKE_LID_GAP = 10.0
            asm = Assembly(parts=[self.part("deck"), self.part("cover"), L.intake_holder()[0]], held=self.asm.held)
            self.assertTrue(fails(C.check_captive(asm), "intake-fan-holder: ", "can lift out"))
        finally:
            L.INTAKE_LID_GAP = old

    def test_the_xt60_is_held_against_plugging_and_unplugging(self):
        # Pushed in (+x), its body meets the cradle's back stops; pulled out,
        # a tie over it through the deck holds it.
        xt = next(h for h in self.asm.held if h.name == "xt60")
        deck = self.asm.parts[0].shape
        push = box(L.XT_BODY[1], L.XT_BODY[1] + 1.0, *L.XT_BODY[2:])
        self.assertGreater(C.common(deck, push), 1.0, "a back stop behind the body")
        self.assertLess(C.common(deck, xt.shape), C.VOLUME_TOL)
        self.assertEqual(len([o for o in self.asm.openings if o.name.startswith("xt60 tie")]), 2)


if __name__ == "__main__":
    unittest.main()
