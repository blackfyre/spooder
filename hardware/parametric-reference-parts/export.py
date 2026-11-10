"""STEP export, re-read and publish, per source (design: "STEP export and re-read").

Each part's solid is written as `<name>.step` with a fixed header time (so an
unchanged part writes an unchanged file) into a staging folder beside the
source's `step/`. Each staged file is re-read as one solid whose volume and
bounding box match the built part's. Only then is `step/` replaced as a whole:
`step/` -> `step.old/`, staging -> `step/`, then `step.old/` removed; if the
second rename fails, `step.old/` is renamed back. A leftover `step.old/` (a
run killed between the renames) is restored when `step/` is missing, and
otherwise removed, before anything else (`recover`). Each source is published
on its own, so one source's build never touches another's `step/`.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path

STEP_DIR = "step"
OLD_DIR = "step.old"
STAGING_PREFIX = ".step-staging-"
STEP_SUFFIX = ".step"
# A fixed STEP header time. Source: chosen, as the electronics mount's.
STEP_TIMESTAMP = "2026-01-01T00:00:00"
# Re-read tolerances. Source: design.md, "STEP export and re-read".
REREAD_VOLUME_PCT = 0.1  # % of the built part's volume
REREAD_BBOX_TOL = 0.05  # mm, per bounding-box face


def step_name(part_name: str) -> str:
    return part_name + STEP_SUFFIX


def recover(source_dir: Path) -> list[str]:
    """Resolve what an interrupted run left: restore `step.old/` when `step/`
    is missing, else remove it; remove stale staging folders. Returns what it
    did, for the log."""
    done = []
    old, step = source_dir / OLD_DIR, source_dir / STEP_DIR
    if old.is_dir():
        if step.exists():
            shutil.rmtree(old)
            done.append(f"{source_dir.name}: removed a leftover {OLD_DIR}/")
        else:
            os.rename(old, step)
            done.append(f"{source_dir.name}: restored {STEP_DIR}/ from a leftover {OLD_DIR}/")
    for stale in sorted(source_dir.glob(STAGING_PREFIX + "*")):
        if stale.is_dir():
            shutil.rmtree(stale)
            done.append(f"{source_dir.name}: removed a stale staging folder {stale.name}")
    return done


def stage(source_dir: Path) -> Path:
    """A staging folder beside `step/` (same file system, so the swap is a rename)."""
    return Path(tempfile.mkdtemp(prefix=STAGING_PREFIX, dir=source_dir))


def write(solids: dict[str, object], staging: Path) -> list[Path]:
    from build123d import export_step

    written = []
    for name, solid in sorted(solids.items()):
        path = staging / step_name(name)
        if not export_step(solid, path, timestamp=STEP_TIMESTAMP):
            raise RuntimeError(f"export_step failed for {name}")
        written.append(path)
    return written


def bbox(shape) -> tuple[float, ...]:
    bb = shape.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)


def reread(solids: dict[str, object], staging: Path) -> list[str]:
    """Re-read every staged file: one solid, the built volume and bounding box."""
    from build123d import import_step

    failures = []
    expected = {step_name(n) for n in solids}
    found = {p.name for p in staging.iterdir()}
    failures += [f"export: {f} was not written" for f in sorted(expected - found)]
    failures += [f"export: {f} is not a current part" for f in sorted(found - expected)]
    for name, built in sorted(solids.items()):
        path = staging / step_name(name)
        if not path.exists():
            continue
        try:
            shape = import_step(path)
        except Exception as e:  # noqa: BLE001 - an unreadable file is the failure being checked
            failures.append(f"export: {path.name} does not re-read: {e}")
            continue
        count = len(shape.solids())
        if count != 1:
            failures.append(f"export: {path.name} re-reads as {count} solids, not one")
            continue
        v, vb = shape.solids()[0].volume, built.volume
        if not abs(v - vb) <= vb * REREAD_VOLUME_PCT / 100:  # NaN fails too
            failures.append(f"export: {path.name} re-reads with volume {v:.3f} mm3, built {vb:.3f} mm3 "
                            f"(limit {REREAD_VOLUME_PCT} %)")
        got, want = bbox(shape), bbox(built)
        if not all(abs(g - w) <= REREAD_BBOX_TOL for g, w in zip(got, want)):
            failures.append(f"export: {path.name}'s bounding box {tuple(round(g, 3) for g in got)} is not the "
                            f"built part's {tuple(round(w, 3) for w in want)} (limit {REREAD_BBOX_TOL} mm)")
    return failures


def publish(staging: Path, source_dir: Path) -> None:
    """Replace `source_dir/step/` with `staging` as a whole."""
    step, old = source_dir / STEP_DIR, source_dir / OLD_DIR
    if step.exists():
        os.rename(step, old)
        try:
            os.rename(staging, step)
        except BaseException:
            os.rename(old, step)
            raise
        shutil.rmtree(old)
    else:
        os.rename(staging, step)
    if not any(step.iterdir()):
        step.rmdir()  # a source with no parts keeps no empty step/


def export_all(sources: list[tuple[Path, dict[str, object]]]) -> list[str]:
    """Stage and re-read every source's parts, and only when every file
    re-reads correctly publish each source's `step/`; a failure anywhere
    leaves every `step/` as it was."""
    staged, failures = [], []
    try:
        for source_dir, solids in sources:
            staging = stage(source_dir)
            staged.append((staging, source_dir))
            write(solids, staging)
            failures += [f"{source_dir.name}: {f}" for f in reread(solids, staging)]
        if not failures:
            for staging, source_dir in staged:
                publish(staging, source_dir)
        return failures
    finally:
        for staging, _ in staged:
            shutil.rmtree(staging, ignore_errors=True)
