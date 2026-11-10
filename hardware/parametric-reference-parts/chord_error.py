"""The reference STLs' own tessellation chord error (design: "Tolerances").

An STL's vertices lie on the true surface, so a facet spanning an angle
theta of a curve of radius r stands off it by the sagitta r(1 - cos(theta/2)),
written r(1 - cos(pi/n)) with n = 2 pi / theta the segments a full turn of
that size would have. The curved features are found in sections of each
STL every SECTION_STEP mm along each axis: horizontal ones see the vertical
cylinders and cones, vertical ones across x and y the rounds whose axes lie
level and square to the section (such as Frame Bottom's rear floor step).
No section sees a round revolved about a vertical axis in its meridian (the
Bearing Axle Mount's boss foot): a vertical section rarely passes through
the axis, and where it does it cuts the facets slantwise. design.md gives
that round's chord error from a direct fit.
A vertical section counts a run only where the facets it crosses stand
square to the section (their normals within IN_PLANE of it): one that cuts
a vertical cylinder or cone slantwise shows its facets stretched, not their
sagitta. Every curved face of the frame group is a vertical cylinder or
cone, seen square in horizontal sections, a level round, or the boss foot.
In each section loop, runs of at least
MIN_SEGMENTS segments, each spanning at most MAX_SEGMENT_ANGLE, whose
consecutive vertices lie on one circle (to FIT_TOLERANCE) are a curved
feature, provided that the boundary turns at each inner vertex by the
angle the circle gives its two segments (to TURN_TOLERANCE), so that
neither a polygon's corners, which can lie on a circle, nor a straight run's
noise counts; its circle fit gives r, and its longest
segment theta. A part's chord error is the largest over its features.

    uv run --locked --quiet --script build.py --chord-error
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SECTION_STEP = 0.5  # mm between sections, along each axis
MIN_SEGMENTS = 4  # segments in a run before it counts as a curve
FIT_TOLERANCE = 0.002  # mm: a run's vertices' largest distance from its circle (the STL is float32)
MAX_RADIUS = 100.0  # mm: a longer radius is a straight line (the parts' largest arc is r 44.22)
TURN_TOLERANCE = 0.1  # a curve's vertex turns by its two segments' half angles, to this fraction
COLLINEAR = math.radians(0.02)  # turns below this are a section's split points, not vertices
IN_PLANE = 0.05  # largest |facet normal . section normal| for a vertical section's run
MAX_SEGMENT_ANGLE = math.radians(40)  # a segment spanning more is a corner, not a curve's facet


@dataclass
class Feature:
    radius: float
    segments_per_turn: float
    error: float
    axis: str  # the section's normal
    at: float  # and its place along it


def fit_circle(p: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Least-squares circle through 2D points: centre, radius, largest
    distance of a point from it."""
    a = np.c_[2 * p, np.ones(len(p))]
    b = (p ** 2).sum(1)
    s = np.linalg.lstsq(a, b, rcond=None)[0]
    c = s[:2]
    r = math.sqrt(max(s[2] + c @ c, 0.0))
    return c, r, float(np.abs(np.linalg.norm(p - c, axis=1) - r).max())


def loop_vertices(loop: np.ndarray) -> np.ndarray:
    """A closed section loop's vertices, less repeats and the collinear
    points where the section crosses a facet's inner edge."""
    p = loop[:-1] if np.allclose(loop[0], loop[-1]) else loop
    p = p[np.linalg.norm(np.roll(p, -1, 0) - p, axis=1) > 1e-6]
    for _ in range(3):
        if len(p) < 3:
            return p
        d = np.roll(p, -1, 0) - p
        ang = np.arctan2(d[:, 1], d[:, 0])
        turn = (ang - np.roll(ang, 1) + np.pi) % (2 * np.pi) - np.pi
        p = p[np.abs(turn) > COLLINEAR]
    return p


def loop_features(p: np.ndarray) -> list[tuple[float, float, np.ndarray]]:
    """(radius, largest segment angle, the run's segment midpoints) of each
    curved run in a loop."""
    n = len(p)
    if n < MIN_SEGMENTS + 1:
        return []
    out = []
    used = np.zeros(n, bool)
    def angle(a, b, r):
        return 2 * math.asin(min(1.0, float(np.linalg.norm(p[b] - p[a])) / (2 * r)))

    for start in range(n):
        if used[start]:
            continue
        run = [start, (start + 1) % n, (start + 2) % n]
        _, r, err = fit_circle(p[run])
        if r > MAX_RADIUS or max(angle(run[0], run[1], r), angle(run[1], run[2], r)) > MAX_SEGMENT_ANGLE:
            continue
        while len(run) < n:
            nxt = (run[-1] + 1) % n
            _, r2, err2 = fit_circle(p[run + [nxt]])
            if err2 > FIT_TOLERANCE or r2 > MAX_RADIUS or angle(run[-1], nxt, r2) > MAX_SEGMENT_ANGLE:
                break
            run.append(nxt)
            r, err = r2, err2
        if len(run) - 1 < MIN_SEGMENTS or err > FIT_TOLERANCE:
            continue
        q = p[run]
        d = np.diff(q, axis=0)
        heading = np.arctan2(d[:, 1], d[:, 0])
        turns = np.abs((np.diff(heading) + np.pi) % (2 * np.pi) - np.pi)
        halves = np.arcsin(np.clip(np.linalg.norm(d, axis=1) / (2 * r), 0, 1))
        expected = halves[:-1] + halves[1:]
        if not np.all(np.abs(turns - expected) <= TURN_TOLERANCE * expected):
            continue
        used[run] = True
        chords = np.linalg.norm(np.diff(q, axis=0), axis=1)
        theta = float((2 * np.arcsin(np.clip(chords / (2 * r), 0, 1))).max())
        out.append((r, theta, (q[1:] + q[:-1]) / 2))
    return out


def section_loops(mesh, normal: np.ndarray, value: float) -> list[np.ndarray]:
    """The closed loops of a plane section of a mesh, as 3D point arrays
    (trimesh's own section loops need networkx, which the build does not
    pin): the section's segments chained end to end."""
    import trimesh

    segments = trimesh.intersections.mesh_plane(mesh, normal, normal * value)
    if len(segments) == 0:
        return []
    key = np.round(segments.reshape(-1, 3), 6)
    _, ids = np.unique(key, axis=0, return_inverse=True)
    ids = ids.reshape(-1, 2)
    points = segments.reshape(-1, 3)
    neighbours: dict[int, list[int]] = {}
    first: dict[int, int] = {}
    for k, (a, b) in enumerate(ids):
        if a == b:
            continue
        neighbours.setdefault(a, []).append(b)
        neighbours.setdefault(b, []).append(a)
        first.setdefault(a, 2 * k)
        first.setdefault(b, 2 * k + 1)
    loops, seen = [], set()
    for start in neighbours:
        if start in seen or len(neighbours[start]) != 2:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [v for v in neighbours[cur] if v != prev]
            if not nxt or len(neighbours[cur]) != 2:
                loop = None
                break
            prev, cur = cur, nxt[0]
            if cur == start:
                break
            if cur in seen:
                loop = None
                break
            seen.add(cur)
            loop.append(cur)
        if loop and len(loop) >= 3:
            loops.append(np.array([points[first[v]] for v in loop + [start]]))
    return loops


def measure(stl: Path) -> Feature | None:
    """The largest chord error of an STL's curved features."""
    import trimesh

    mesh = trimesh.load(str(stl), force="mesh")
    lo, hi = mesh.bounds
    best: Feature | None = None
    for axis in range(3):
        normal = np.eye(3)[axis]
        keep = [i for i in range(3) if i != axis]
        for v in np.arange(lo[axis] + SECTION_STEP / 2, hi[axis], SECTION_STEP):
            for loop in section_loops(mesh, normal, float(v)):
                p = loop_vertices(loop[:, keep])
                for r, theta, mids in loop_features(p):
                    if axis != 2:
                        full = np.insert(mids, axis, v, axis=1)
                        _, _, faces = trimesh.proximity.closest_point(mesh, full)
                        if np.abs(mesh.face_normals[faces] @ normal).max() > IN_PLANE:
                            continue
                    error = r * (1 - math.cos(theta / 2))
                    if best is None or error > best.error:
                        best = Feature(r, 2 * math.pi / theta, error, "xyz"[axis], float(v))
    return best


def main(sources) -> int:
    """Print each part's STL chord error, for the source parts given."""
    for source in sources:
        for part in source.parts:
            f = measure(part.stl)
            if f is None:
                print(f"{source.name}/{part.name}: no curved features found")
                continue
            print(f"{source.name}/{part.name}: chord error {f.error:.3f} mm (r {f.radius:.2f} mm, "
                  f"n {f.segments_per_turn:.0f}, section {f.axis} = {f.at:.2f})")
    return 0
