# /// script
# requires-python = "==3.12.*"
# dependencies = [
#     "build123d==0.13.0",
#     "matplotlib==3.11.2",
#     "numpy==2.1.3",
#     "trimesh==4.5.3",
# ]
# [tool.uv]
# exclude-newer = "2026-10-01T00:00:00Z"
# ///
"""Render the electronics mount on the frame top to images/.

    uv run --locked --quiet --script hardware/spooder-electronics-mount/preview.py

`mise run cad-preview` runs it. Its dependencies are locked in
preview.py.lock (`uv lock --script`). It builds the part set as build.py
does, without checking or exporting it, and draws:
- images/assembly.png, the open assembly from above and in front;
- images/top.png, from straight above, with the legs' keep-clear region.

Printed parts are drawn in their colours, held parts in blue, the reserved
routes and the air channel in translucent yellow, and the frame top in grey.
Every triangle goes into one Poly3DCollection: separate collections are
drawn in the wrong depth order (the spooder-printed-parts skill).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import measurements as M  # noqa: E402
from build import build  # noqa: E402
from checks import KEEP_CLEAR_R, KEEP_CLEAR_TAB, triangles  # noqa: E402
from model import Assembly, box6  # noqa: E402
from reference_check import FRAME_STL  # noqa: E402

COLOURS = {"black": (0.17, 0.17, 0.17, 1.0), "yellow": (0.95, 0.76, 0.0, 1.0), "white": (0.91, 0.91, 0.91, 1.0)}
HELD = (0.23, 0.49, 0.85, 1.0)
ZONE = (0.95, 0.83, 0.23, 0.30)
FRAME = (0.69, 0.69, 0.69, 1.0)
VIEWS = {"assembly": (35, -60), "top": (90, -90)}
LIGHT = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])


def shaded(tris: np.ndarray, colour) -> np.ndarray:
    """Per-triangle colours, lit from above and in front."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    k = 0.55 + 0.45 * np.abs(n @ LIGHT)
    c = np.tile(np.array(colour, dtype=float), (len(tris), 1))
    c[:, :3] *= k[:, None]
    return c


def scene(asm: Assembly):
    frame = trimesh.load(FRAME_STL)
    groups = [(frame.triangles, FRAME)]
    for part in asm.parts:
        colour = COLOURS.get(part.colour, COLOURS["white"])
        groups.append((triangles(part.instances()), colour))
        if part.inlays is not None:
            groups.append((triangles([part.inlays]), COLOURS["yellow"]))
    for held in asm.held:
        groups.append((triangles([held.shape]), HELD))
    zones = [r.shape for r in asm.routes]
    if asm.channel is not None:
        zones += [box6(b) for b, _ in asm.channel.segments]
    if zones:
        groups.append((triangles(zones), ZONE))
    tris = np.concatenate([t for t, _ in groups if len(t)])
    colours = np.concatenate([shaded(t, c) for t, c in groups if len(t)])
    return tris, colours


def keep_clear_outline() -> np.ndarray:
    """The keep-clear region's outline at the ring's height: the circle and
    the front and rear tabs."""
    tx, ty = KEEP_CLEAR_TAB
    xy = []
    for a in np.linspace(0, 2 * math.pi, 721):
        x, y = KEEP_CLEAR_R * math.sin(a), KEEP_CLEAR_R * math.cos(a)
        if abs(x) <= tx:
            y = math.copysign(ty, y)
        xy.append((x, y))
    xy = np.array(xy)
    return np.column_stack([xy, np.full(len(xy), M.FRAME_RING_Z + 0.1)])


def render(asm: Assembly, outdir: Path) -> list[Path]:
    tris, colours = scene(asm)
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    centre, half = (lo + hi) / 2, (hi - lo).max() / 2
    outline = keep_clear_outline()
    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    for name, (elev, azim) in VIEWS.items():
        fig = plt.figure(figsize=(8, 8), dpi=100)
        ax = fig.add_subplot(projection="3d", proj_type="ortho", computed_zorder=False)
        ax.add_collection3d(Poly3DCollection(tris, facecolors=colours, edgecolors="none", linewidths=0))
        if name == "top":
            ax.plot(outline[:, 0], outline[:, 1], outline[:, 2], color=(0.85, 0.2, 0.2), lw=1.2, zorder=10)
        ax.set_xlim(centre[0] - half, centre[0] + half)
        ax.set_ylim(centre[1] - half, centre[1] + half)
        ax.set_zlim(centre[2] - half, centre[2] + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        fig.subplots_adjust(0, 0, 1, 1)
        path = outdir / f"{name}.png"
        fig.savefig(path, metadata={"Software": None})
        plt.close(fig)
        written.append(path)
    return written


def main() -> int:
    asm = build()
    for path in render(asm, HERE / "images"):
        print(f"wrote {path.relative_to(HERE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
