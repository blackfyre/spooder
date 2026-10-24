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
"""Generate the electronics mount, check it, and only then write its files.

    uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py          # check, then write
    uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --check  # check only
    uv run --locked --quiet --script hardware/spooder-electronics-mount/build.py --test   # the unit tests

`mise run cad` runs all of them. Its dependencies are locked in build.py.lock
(`uv lock --script`). The sizes come from measurements.py, the checks from
checks.py and the export from export.py. When any check fails, the build
names every failure and writes nothing; when all pass, it writes each
distinct part once (export.py) and re-reads the files to confirm them.
"""

from __future__ import annotations

import argparse
import sys
import unittest
from collections.abc import Callable
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from checks import run_checks  # noqa: E402
from export import export_parts, verify_export  # noqa: E402
from model import Assembly  # noqa: E402


def build() -> Assembly:
    """The part set, in the frame top's coordinates. Empty until the deck
    and the held parts are laid out (task group 3 of add-electronics-mount)."""
    return Assembly()


def run_tests() -> int:
    suite = unittest.defaultTestLoader.discover(str(HERE), pattern="test_*.py", top_level_dir=str(HERE))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


def main(argv: list[str] | None = None, outdir: Path = HERE, make: Callable[[], Assembly] = build) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="check the parts, write nothing")
    parser.add_argument("--test", action="store_true", help="run the unit tests")
    args = parser.parse_args(argv)
    if args.test:
        return run_tests()

    asm = make()
    print(f"{len(asm.parts)} parts, {len(asm.held)} held parts, {len(asm.routes)} routes, "
          f"{len(asm.openings)} openings, {len(asm.screws)} screws, "
          f"{'an' if asm.channel else 'no'} air channel")
    failures = run_checks(asm)
    if failures:
        print(f"\n{len(failures)} check(s) failed; nothing written:")
        for f in failures:
            print("  " + f)
        return 1
    print("All checks pass.")
    if args.check:
        return 0

    written = export_parts(asm.parts, outdir)
    failures = verify_export(asm.parts, outdir)
    if failures:
        print(f"\n{len(failures)} export problem(s):")
        for f in failures:
            print("  " + f)
        return 1
    for path in written:
        print(f"wrote {path.relative_to(outdir)}")
    print(f"Exported {len(asm.parts)} distinct part(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
