"""The gates every part passes against its reference STL (design: "Gates").

In order, per part:
1. solid: the build result is exactly one Solid, valid, its shell closed;
2. volume: |V - V_stl| <= 0.5 % of V_stl;
3. surface deviation, measured both ways: the points on each side are every
   vertex of that side's mesh bisected until no edge exceeds 0.5 mm, plus
   100 000 points sampled by area with a fixed seed; each point's distance to
   the other mesh is taken. The maximum over all points of both directions
   must be <= 0.20 mm and the 95th percentile over both directions' area
   samples <= 0.05 mm. Every point of a facet whose edges are at most e lies
   within e/sqrt(3) of one of its vertices, so the true maximum is at most the
   reported one + 0.29 mm;
4. bounding box: each of the six faces within 0.15 mm of the STL's.

A part's `exceptions` replace a gate's limit for that part only, each naming
the feature responsible; the report prints every exception it applies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Limits. Source: design.md, "Tolerances, against the STL's chord error".
VOLUME_PCT = 0.5  # % of the STL's volume
DEVIATION_MAX = 0.20  # mm, over vertices and samples, both ways
DEVIATION_P95 = 0.05  # mm, over the area samples, both ways
BBOX_TOL = 0.15  # mm, per bounding-box face
GLOBAL_LIMITS = {"volume": VOLUME_PCT, "deviation_max": DEVIATION_MAX, "deviation_p95": DEVIATION_P95,
                 "bbox": BBOX_TOL}
EXCEPTION_GATES = frozenset(GLOBAL_LIMITS)

# Measurement. Source: design.md, "Gates".
TESSELLATION_TOL = 0.005  # mm, chordal: the solid's own chord error is negligible
TESSELLATION_ANGULAR = 0.1  # rad, build123d's default
SUBDIVIDE_EDGE = 0.5  # mm, the longest edge after bisection
SAMPLE_BOUND = SUBDIVIDE_EDGE / math.sqrt(3)  # mm, the unsampled deviation's bound
SAMPLES = 100_000  # area samples per side
SEED = 20261110  # the part side's; the STL side's is SEED + 1
CHUNK = 50_000  # points per closest-point query


@dataclass
class Direction:
    """Distances from one side's points to the other side's mesh."""
    vertices: np.ndarray
    samples: np.ndarray

    @property
    def maximum(self) -> float:
        return float(max(self.vertices.max(initial=0.0), self.samples.max(initial=0.0)))

    @property
    def p95(self) -> float:
        return float(np.percentile(self.samples, 95)) if len(self.samples) else 0.0


@dataclass
class Report:
    name: str
    failures: list[str] = field(default_factory=list)
    lines: list[str] = field(default_factory=list)
    solid: object | None = None  # the checked solid, when the solid gate passes
    volume: float | None = None


def bisected_vertices(triangles: np.ndarray, max_edge: float) -> np.ndarray:
    """Every corner of the triangles after splitting each at its longest
    edge's midpoint until no edge exceeds `max_edge`. Each facet is split on
    its own (the result need not be a conforming mesh: only the points are
    used), which avoids the slivers that splitting every edge makes of long
    thin facets."""
    done = []
    tris = np.asarray(triangles, dtype=float)
    while len(tris):
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        lengths = np.stack([np.linalg.norm(b - c, axis=1), np.linalg.norm(c - a, axis=1),
                            np.linalg.norm(a - b, axis=1)], axis=1)
        long = lengths.max(axis=1) > max_edge
        done.append(tris[~long].reshape(-1, 3))
        k = lengths.argmax(axis=1)[long]
        t = tris[long]
        # Rotate each triangle so that its longest edge runs from corner 1 to 2.
        order = np.stack([k, (k + 1) % 3, (k + 2) % 3], axis=1)
        t = np.take_along_axis(t, order[:, :, None], axis=1)
        mid = (t[:, 1] + t[:, 2]) / 2
        tris = np.concatenate([np.stack([t[:, 0], t[:, 1], mid], axis=1), np.stack([t[:, 0], mid, t[:, 2]], axis=1)])
    return np.unique(np.concatenate(done), axis=0)


def distances(points: np.ndarray, mesh) -> np.ndarray:
    import trimesh

    out = [trimesh.proximity.closest_point(mesh, points[i:i + CHUNK])[1] for i in range(0, len(points), CHUNK)]
    return np.concatenate(out) if out else np.zeros(0)


def side_points(mesh, count: int, seed: int) -> tuple[np.ndarray, np.ndarray]:
    import trimesh

    samples, _ = trimesh.sample.sample_surface(mesh, count, seed=seed)
    return bisected_vertices(mesh.triangles, SUBDIVIDE_EDGE), np.asarray(samples)


def direction(points: tuple[np.ndarray, np.ndarray], other) -> Direction:
    vertices, samples = points
    return Direction(distances(vertices, other), distances(samples, other))


def solid_mesh(solid):
    import trimesh

    verts, tris = solid.tessellate(TESSELLATION_TOL, TESSELLATION_ANGULAR)
    return trimesh.Trimesh(np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(tris), process=True)


def load_stl(path: Path):
    import trimesh

    return trimesh.load(str(path), force="mesh")


def single_solid(result) -> tuple[object | None, list[str]]:
    """The one valid, closed solid in a build result, or the reasons why not."""
    from OCP.BRep import BRep_Tool

    solids = result.solids() if hasattr(result, "solids") else []
    if len(solids) != 1:
        return None, [f"is {len(solids)} solids, not one"]
    solid = solids[0]
    reasons = []
    if not solid.is_valid:
        reasons.append("is not a valid solid")
    shells = solid.shells()
    if len(shells) < 1 or not all(BRep_Tool.IsClosed_s(s.wrapped) for s in shells):
        reasons.append("has an open shell")
    return (None, reasons) if reasons else (solid, [])


def limits_for(name: str, exceptions: dict[str, dict], report: Report) -> dict[str, float]:
    limits = dict(GLOBAL_LIMITS)
    for gate, spec in sorted(exceptions.items()):
        limits[gate] = float(spec["limit"])
        report.lines.append(f"  exception: {name} {gate} limit {spec['limit']} (global {GLOBAL_LIMITS[gate]}) "
                            f"for {spec['feature']}: {spec['reason']}")
    return limits


def check(name: str, result, stl: Path, exceptions: dict[str, dict] | None = None,
          samples: int | None = None, seed: int | None = None) -> Report:
    """Run the four gates on one build result against its STL."""
    samples = SAMPLES if samples is None else samples
    seed = SEED if seed is None else seed
    report = Report(name)
    limits = limits_for(name, exceptions or {}, report)
    solid, reasons = single_solid(result)
    if reasons:
        report.failures += [f"{name}: solid: the build result {r}" for r in reasons]
        report.failures.append(f"{name}: the volume, deviation and bounding-box gates need one valid closed solid")
        return report
    report.solid = solid
    ref = load_stl(stl)
    part = solid_mesh(solid)
    report.volume = solid.volume

    v_stl = float(ref.volume)
    pct = abs(solid.volume - v_stl) / v_stl * 100 if v_stl else math.inf
    report.lines.append(f"  {name}: volume {solid.volume:.1f} mm3, STL {v_stl:.1f} mm3, {pct:.3f} % "
                        f"(limit {limits['volume']} %)")
    if not pct <= limits["volume"]:  # NaN fails too
        report.failures.append(f"{name}: volume: {solid.volume:.1f} mm3 is {pct:.3f} % off the STL's {v_stl:.1f} mm3 "
                               f"(limit {limits['volume']} %)")

    to_stl = direction(side_points(part, samples, seed), ref)  # an extra feature shows here
    to_part = direction(side_points(ref, samples, seed + 1), part)  # a missing feature shows here
    worst = max(to_stl.maximum, to_part.maximum)
    p95 = float(np.percentile(np.concatenate([to_stl.samples, to_part.samples]), 95))
    for label, d in (("part to STL", to_stl), ("STL to part", to_part)):
        report.lines.append(f"  {name}: deviation {label}: max {d.maximum:.3f} mm, 95th percentile {d.p95:.4f} mm "
                            f"({len(d.vertices)} vertices, {len(d.samples)} samples)")
    report.lines.append(f"  {name}: deviation both ways: max {worst:.3f} mm (limit {limits['deviation_max']}; "
                        f"unsampled points within {SAMPLE_BOUND:.2f} mm more), 95th percentile {p95:.4f} mm "
                        f"(limit {limits['deviation_p95']})")
    if not worst <= limits["deviation_max"]:
        report.failures.append(f"{name}: deviation max: {worst:.3f} mm (part to STL {to_stl.maximum:.3f}, "
                               f"STL to part {to_part.maximum:.3f}; limit {limits['deviation_max']} mm)")
    if not p95 <= limits["deviation_p95"]:
        report.failures.append(f"{name}: deviation 95th percentile: {p95:.4f} mm (part to STL {to_stl.p95:.4f}, "
                               f"STL to part {to_part.p95:.4f}; limit {limits['deviation_p95']} mm)")

    bb = solid.bounding_box()
    got = (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)
    want = (*ref.bounds[0], *ref.bounds[1])
    faces = ("x min", "y min", "z min", "x max", "y max", "z max")
    off = [(f, g, w) for f, g, w in zip(faces, got, want) if not abs(g - w) <= limits["bbox"]]
    report.lines.append(f"  {name}: bounding box faces within {max(abs(g - w) for g, w in zip(got, want)):.3f} mm "
                        f"(limit {limits['bbox']})")
    for f, g, w in off:
        report.failures.append(f"{name}: bounding box: {f} {g:.3f} mm, STL {w:.3f} mm "
                               f"({abs(g - w):.3f} off; limit {limits['bbox']} mm)")
    return report
