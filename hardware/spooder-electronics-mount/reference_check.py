# /// script
# requires-python = "==3.12.*"
# dependencies = [
#     "build123d==0.13.0",
#     "numpy==2.1.3",
#     "pillow==12.2.0",
#     "trimesh==4.5.3",
# ]
# [tool.uv]
# exclude-newer = "2026-10-01T00:00:00Z"
# ///
"""Re-derive the reference data in measurements.py from the reference files.

    uv run --locked --quiet hardware/spooder-electronics-mount/reference_check.py

Its dependencies are locked in reference_check.py.lock (`uv lock --script`).
Checks each constant taken from a machine-readable reference file against
that file, and exits non-zero naming every one that no longer matches:

- the frame top STL: its surface levels, its insert holes (found
  independently, so a missing or extra one fails) and the open rectangles
  (a point-in-triangle height map of its upward faces);
- the T-Display STEP model: the PCB, the screen, the buttons, the header rows
  and the bare spots on its +z face (its solids' bounding boxes);
- the Servo 2040 drawing: every pad of J6, the EXT terminal, the sensor
  header and the servo header (the magenta pad rings), the mounting holes and
  the USB-C connector, against the drawing's pixels.

Not checkable from pixels, and read from the drawing's labels instead: which
servo row is "-", "+" or signal, and which sensor column is IN, 3V3 or GND.
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import measurements as M  # noqa: E402

ROOT = HERE.parent.parent
FRAME_STL = ROOT / "hardware/aecert-hexapod-v1/stl/Frame Top.stl"
TD_STEP = ROOT / "hardware/lilygo-t-display/3d_file/T-Display.step"
S2040_PNG = ROOT / "hardware/pimoroni-servo2040/servo-2040-drawing.png"

failures: list[str] = []


def expect(ok: bool, what: str) -> None:
    print(("ok    " if ok else "FAIL  ") + what)
    if not ok:
        failures.append(what)


def close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


# --------------------------------------------------------------------------- frame top

GRID = 0.25


def height_map(mesh, step: float):
    """Highest upward surface at each grid point (NaN where open)."""
    xs = np.arange(-120 + step / 2, 120, step)
    ys = np.arange(-120 + step / 2, 120, step)
    h = np.full((len(ys), len(xs)), np.nan)
    for a, b, c in mesh.triangles[mesh.face_normals[:, 2] > 1e-6]:
        lo = np.minimum(np.minimum(a, b), c)
        hi = np.maximum(np.maximum(a, b), c)
        i0, i1 = np.searchsorted(xs, lo[0]), np.searchsorted(xs, hi[0], side="right")
        j0, j1 = np.searchsorted(ys, lo[1]), np.searchsorted(ys, hi[1], side="right")
        if i0 >= i1 or j0 >= j1:
            continue
        gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
        v0, v1 = c[:2] - a[:2], b[:2] - a[:2]
        px, py = gx - a[0], gy - a[1]
        d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-12:
            continue
        d20, d21 = px * v0[0] + py * v0[1], px * v1[0] + py * v1[1]
        u = (d11 * d20 - d01 * d21) / den
        v = (d00 * d21 - d01 * d20) / den
        inside = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        z = a[2] + u * (c[2] - a[2]) + v * (b[2] - a[2])
        sub = h[j0:j1, i0:i1]
        better = inside & (np.isnan(sub) | (z > sub))
        sub[better] = z[better]
    gx, gy = np.meshgrid(xs, ys)
    return gx, gy, h


def round_holes(gx, gy, h):
    """Centres of the round holes in a flat surface: clusters of points that
    are open or more than 0.5 mm low, inside a full ring of 16 samples 3 mm
    out at one level. A cluster of fewer than 10 points is an edge, not a hole."""
    k = round(3.0 / GRID)
    offsets = {(round(k * np.cos(a)), round(k * np.sin(a))) for a in np.linspace(0, 2 * np.pi, 16, endpoint=False)}
    padded = np.pad(h, k, constant_values=np.nan)
    ring = np.stack([padded[k + dy:k + dy + h.shape[0], k + dx:k + dx + h.shape[1]] for dx, dy in offsets])
    flat = np.all(np.abs(ring - ring[0]) < 0.01, axis=0)
    low = np.isnan(h) | (h < ring[0] - 0.5)
    clusters: list[list[tuple[float, float]]] = []
    for x, y in zip(gx[flat & low], gy[flat & low]):
        for c in clusters:
            if (c[0][0] - x) ** 2 + (c[0][1] - y) ** 2 < 9:
                c.append((x, y))
                break
        else:
            clusters.append([(x, y)])
    return [tuple(np.mean(c, axis=0)) for c in clusters if len(c) >= 10]


def check_frame() -> None:
    import trimesh

    mesh = trimesh.load(FRAME_STL)
    n, centre, area = mesh.face_normals, mesh.triangles_center, mesh.area_faces
    levels: dict[float, float] = {}
    for z, a in zip(np.round(centre[n[:, 2] > 0.99, 2], 2), area[n[:, 2] > 0.99]):
        levels[float(z)] = levels.get(float(z), 0.0) + a
    by_area = sorted(levels, key=lambda z: -levels[z])
    expect(by_area[0] == M.FRAME_RING_Z and max(levels) == M.FRAME_RING_Z,
           f"FRAME_RING_Z {M.FRAME_RING_Z} is the highest and largest upward level ({by_area[0]})")
    expect(by_area[1] == M.FRAME_FLOOR_Z, f"FRAME_FLOOR_Z {M.FRAME_FLOOR_Z} is the second level ({by_area[1]})")
    expect(close(mesh.bounds[0, 2], M.FRAME_BOTTOM_Z, 1e-3),
           f"FRAME_BOTTOM_Z {M.FRAME_BOTTOM_Z} is the STL's lowest z ({mesh.bounds[0, 2]:.3f})")

    gx, gy, h = height_map(mesh, GRID)
    found = sorted((round(float(x), 2), round(float(y), 2)) for x, y in round_holes(gx, gy, h))
    want = sorted(M.FRAME_INSERTS)
    expect(len(found) == len(want) == 8 and all(close(a, b, 0.1) for f, w in zip(found, want) for a, b in zip(f, w)),
           f"FRAME_INSERTS are the frame's {len(want)} insert holes (found {found})")
    expect(set(M.FRAME_INSERTS) == set(M.FRAME_RING_INSERTS) | set(M.FRAME_FLOOR_INSERTS)
           and len(M.FRAME_RING_INSERTS) == len(M.FRAME_FLOOR_INSERTS) == 4,
           "FRAME_INSERTS is the four ring and four floor inserts")
    for x, y in M.FRAME_INSERTS:
        near = (gx - x) ** 2 + (gy - y) ** 2 < 3.5**2
        top = np.nanmax(h[near])
        hole = near & (np.isnan(h) | (h < top - 0.5))
        cx, cy = gx[hole].mean(), gy[hole].mean()
        dia = 2 * np.sqrt(hole.sum() * GRID * GRID / np.pi)
        expect(close(cx, x, 0.1) and close(cy, y, 0.1),
               f"insert ({x}, {y}): hole centre ({cx:.2f}, {cy:.2f})")
        expect(close(dia, M.FRAME_INSERT_HOLE_D, 0.15),
               f"insert ({x}, {y}): FRAME_INSERT_HOLE_D {M.FRAME_INSERT_HOLE_D} (found {dia:.2f})")
        floor = h[hole]
        if (x, y) in M.FRAME_RING_INSERTS:
            expect(top == M.FRAME_RING_Z and np.all(floor == M.FRAME_RING_INSERT_FLOOR_Z),
                   f"ring insert ({x}, {y}): on the ring, blind to FRAME_RING_INSERT_FLOOR_Z")
        else:
            expect(top == M.FRAME_FLOOR_Z and np.all(np.isnan(floor)),
                   f"floor insert ({x}, {y}): on the floor, through")

    expect(len(M.FRAME_OPEN) > 0, "FRAME_OPEN names open space")
    for x0, x1, y0, y1 in M.FRAME_OPEN:
        cells = (gx > x0) & (gx < x1) & (gy > y0) & (gy < y1)
        solid = int(np.count_nonzero(~np.isnan(h[cells])))
        expect(cells.any() and solid == 0,
               f"FRAME_OPEN ({x0}, {x1}, {y0}, {y1}) is open ({solid} solid points)")
    for group in (M.FRAME_OPEN_NECK, M.FRAME_OPEN_FRONT_REAR, M.FRAME_OPEN_SIDES):
        mirrored = {(-r[1], -r[0], r[2], r[3]) for r in group} | {(r[0], r[1], -r[3], -r[2]) for r in group}
        expect(mirrored <= set(group), f"FRAME_OPEN group {group[0]} is mirrored about both axes")


# --------------------------------------------------------------------------- T-Display


def overlaps(a, b) -> bool:
    """Open rectangles (x0, x1, y0, y1) share area."""
    return a[0] < b[1] and a[1] > b[0] and a[2] < b[3] and a[3] > b[2]


def check_t_display() -> None:
    from build123d import import_step

    boxes = [(s.bounding_box(), s.volume) for s in import_step(TD_STEP).solids()]
    rect = lambda b: (b.min.X, b.max.X, b.min.Y, b.max.Y)  # noqa: E731

    pcb = [b for b, v in boxes if v > 100 and b.min.Z > -1.05 and b.max.Z < 0.05]
    outline = (min(b.min.X for b in pcb), max(b.max.X for b in pcb), min(b.min.Y for b in pcb), max(b.max.Y for b in pcb))
    expect(all(close(p, q, 0.05) for p, q in zip(outline, M.TD_PCB)), f"TD_PCB {M.TD_PCB} (found {tuple(round(v, 2) for v in outline)})")
    thick = -min(b.min.Z for b in pcb)
    expect(close(thick, M.TD_PCB_THICKNESS, 0.02), f"TD_PCB_THICKNESS {M.TD_PCB_THICKNESS} (found {thick:.2f})")

    screen = max(boxes, key=lambda bv: bv[1])[0]
    expect(all(close(p, q, 0.02) for p, q in zip(rect(screen), M.TD_SCREEN)) and screen.max.Z < -1.0,
           f"TD_SCREEN {M.TD_SCREEN} is the largest solid, on the -z face")
    expect(close(screen.max.Z - screen.min.Z, M.TD_SCREEN_DEPTH, 0.02), f"TD_SCREEN_DEPTH {M.TD_SCREEN_DEPTH}")

    deep = [b for b, v in boxes if b.max.Z <= -1.0 and b.min.Z < -2.5 and b is not screen]
    found = sorted(rect(b) for b in deep)
    want = sorted(M.TD_BUTTONS)
    expect(len(found) == len(want) and all(close(p, q, 0.02) for f, w in zip(found, want) for p, q in zip(f, w)),
           f"TD_BUTTONS are the {len(want)} deep parts on the -z face other than the screen (found {len(found)})")
    expect(all(close(b.max.Z - b.min.Z, M.TD_BUTTON_DEPTH, 0.02) for b in deep), f"TD_BUTTON_DEPTH {M.TD_BUTTON_DEPTH}")

    back = [b for b, _ in boxes if b.min.Z > -0.05 and b.max.Z > 0.05]
    tallest = max(b.max.Z for b in back)
    expect(close(tallest, M.TD_UNDERSIDE_TALLEST, 0.02), f"TD_UNDERSIDE_TALLEST {M.TD_UNDERSIDE_TALLEST} (found {tallest:.2f})")

    pins = [b for b, _ in boxes if b.min.Z < -2 and b.max.Z > -0.1]
    for row in M.TD_HEADER_ROWS:
        inrow = [b for b in pins if overlaps(rect(b), row)]
        span = (min(b.min.X for b in inrow), max(b.max.X for b in inrow), min(b.min.Y for b in inrow), max(b.max.Y for b in inrow))
        expect(all(close(p, q, 0.02) for p, q in zip(span, row)), f"TD_HEADER_ROWS {row} (found {tuple(round(v, 2) for v in span)})")
    expect(all(any(overlaps(rect(b), r) for r in M.TD_HEADER_ROWS) for b in pins), "every pin lies in a TD_HEADER_ROWS strip")
    depth = -min(b.min.Z for b in pins)
    expect(close(depth, M.TD_HEADER_PIN_DEPTH, 0.05), f"TD_HEADER_PIN_DEPTH {M.TD_HEADER_PIN_DEPTH} (found {depth:.2f})")

    expect(len(M.TD_BARE) > 0, "TD_BARE names bare spots")
    for bare in M.TD_BARE:
        inside = M.TD_PCB[0] - 1e-6 <= bare[0] < bare[1] <= M.TD_PCB[1] + 1e-6 and M.TD_PCB[2] - 1e-6 <= bare[2] < bare[3] <= M.TD_PCB[3] + 1e-6
        hits = [rect(b) for b in back if overlaps(rect(b), bare)] + [r for r in M.TD_HEADER_ROWS if overlaps(r, bare)]
        expect(inside and not hits, f"TD_BARE {bare} lies on the PCB and is bare ({len(hits)} parts in it)")


# --------------------------------------------------------------------------- Servo 2040 drawing


def check_servo2040() -> None:
    from PIL import Image

    img = np.asarray(Image.open(S2040_PNG).convert("RGB")).astype(int)
    dark = img.sum(axis=2) < 60
    # The board is the one large dark area: its edges are the columns and rows
    # that are mostly dark.
    cols = np.nonzero(dark.sum(axis=0) > 0.5 * dark.sum(axis=0).max())[0]
    rows = np.nonzero(dark.sum(axis=1) > 0.5 * dark.sum(axis=1).max())[0]
    left, right, top, bottom = cols.min(), cols.max() + 1, rows.min(), rows.max() + 1
    sx = (right - left) / M.S2040_BOARD[0]
    sy = (bottom - top) / M.S2040_BOARD[1]
    print(f"      drawing scale {sx:.2f} x {sy:.2f} px/mm")
    ring = (img[:, :, 0] > 150) & (img[:, :, 2] > 150) & (img[:, :, 1] < 100)

    def find_pad(x: float, y: float, tol: float = 0.3) -> tuple[bool, str]:
        """Whether one whole pad ring is centred at (x, y): its centroid within
        tol, and its pixels at one distance from that centroid, so that a
        window between two pads (half of each ring) does not count."""
        px, py = left + x * sx, bottom - y * sy
        r = 1.4 * sx  # under the 1.54 mm from a pad's centre to its neighbour's ring
        x0, y0 = int(px - r), int(py - r)
        ys, xs = np.nonzero(ring[y0:int(py + r) + 1, x0:int(px + r) + 1])
        if len(xs) < 20:
            return False, "no pad"
        xs, ys = xs + x0, ys + y0
        cx, cy = xs.mean(), ys.mean()
        dist = np.hypot(xs - cx, ys - cy) / sx
        whole = dist.std() < 0.15  # 0.12 where a dimension arrow crosses a ring
        cx, cy = (cx - left) / sx, (bottom - cy) / sy
        ok = whole and close(cx, x, tol) and close(cy, y, tol)
        return ok, f"found {cx:.2f}, {cy:.2f}, ring radius {dist.mean():.2f} +- {dist.std():.2f}"

    def pad(x: float, y: float, what: str) -> None:
        ok, found = find_pad(x, y)
        expect(ok, f"{what}: pad at ({x:.2f}, {y:.2f}) ({found})")

    grey = (np.abs(img[:, :, 0] - img[:, :, 1]) < 15) & (np.abs(img[:, :, 1] - img[:, :, 2]) < 15)
    grey &= (img.sum(axis=2) > 450) & (img.sum(axis=2) < 650)
    expect(len(set(M.S2040_HOLES)) == 4, f"S2040_HOLES holds four distinct holes ({len(set(M.S2040_HOLES))})")
    for x, y in M.S2040_HOLES:
        px, py, r = int(left + x * sx), int(bottom - y * sy), int(1.4 * sx)
        ys, xs = np.nonzero(grey[py - r:py + r + 1, px - r:px + r + 1])
        cx = (xs.mean() + px - r - left) / sx if len(xs) else float("nan")
        cy = (bottom - (ys.mean() + py - r)) / sy if len(xs) else float("nan")
        expect(len(xs) > 500 and close(cx, x, 0.2) and close(cy, y, 0.2),
               f"S2040_HOLES ({x}, {y}): a mounting hole (found {cx:.2f}, {cy:.2f})")

    # USB-C: its two filled oval shell pads in the strip along the left edge;
    # the connector's centre is midway between them.
    strip = ring[:, left:int(left + 8 * sx)].sum(axis=1)
    rows = np.nonzero(strip >= 20)[0]
    runs = np.split(rows, np.nonzero(np.diff(rows) > 3)[0] + 1)
    runs = sorted(runs, key=lambda run: -strip[run].sum())[:2]
    usb = np.mean([(bottom - np.average(run, weights=strip[run])) / sy for run in runs])
    expect(len(runs) == 2 and close(usb, M.S2040_USB_Y, 0.3), f"S2040_USB_Y {M.S2040_USB_Y} (found {usb:.2f})")

    p = M.S2040_PITCH
    expect(len(M.S2040_SERVO_GROUP_CENTRES_X) == 3, "S2040_SERVO_GROUP_CENTRES_X holds the three groups")
    expect(all(close(M.S2040_SERVO_COL1_X + 2.5 * p + g * M.S2040_SERVO_GROUP_STEP, c, 1e-6)
               for g, c in enumerate(M.S2040_SERVO_GROUP_CENTRES_X)),
           "S2040_SERVO_COL1_X and S2040_SERVO_GROUP_STEP put each group's centre on S2040_SERVO_GROUP_CENTRES_X")
    expect(M.S2040_J6_PINS >= 9, f"S2040_J6_PINS {M.S2040_J6_PINS} reaches pin 9, the dimensioned A1")
    for n in range(1, M.S2040_J6_PINS + 1):
        pad(M.S2040_J6_PIN9_X + (9 - n) * p, M.S2040_J6_ROW_Y, f"J6 pin {n}")
    for n in (0, M.S2040_J6_PINS + 1):  # the row ends at pins 1 and S2040_J6_PINS
        x = M.S2040_J6_PIN9_X + (9 - n) * p
        for y in (M.S2040_J6_ROW_Y - 0.14, M.S2040_J6_ROW_Y + 0.14):  # either stagger
            found, what = find_pad(x, y)
            expect(not found, f"no J6 pad beyond the row, at ({x:.2f}, {y:.2f}) ({what})")
    ex, ey = M.S2040_EXT_CENTRE
    pad(ex, ey + M.S2040_EXT_POLE_PITCH / 2, "EXT +")
    pad(ex, ey - M.S2040_EXT_POLE_PITCH / 2, "EXT -")
    sx0, sy0 = M.S2040_SENSOR_CENTRE
    for c in range(M.S2040_SENSOR_COLS):
        for r in range(M.S2040_SENSOR_ROWS):
            pad(sx0 + (c - 1) * p, sy0 + (r - 2.5) * p, f"sensor {r + 1} column {c + 1}")
    for col in range(18):
        x = M.S2040_SERVO_COL1_X + (col // 6) * M.S2040_SERVO_GROUP_STEP + (col % 6) * p
        for name, y in (("-", M.S2040_SERVO_ROW_GND_Y), ("+", M.S2040_SERVO_ROW_POWER_Y), ("S", M.S2040_SERVO_ROW_SIGNAL_Y)):
            pad(x, y, f"servo {col + 1} {name}")


if __name__ == "__main__":
    check_frame()
    check_t_display()
    check_servo2040()
    if failures:
        print(f"\n{len(failures)} reference value(s) do not match their files:")
        for f in failures:
            print("  " + f)
        sys.exit(1)
    print("\nAll reference values match their files.")
