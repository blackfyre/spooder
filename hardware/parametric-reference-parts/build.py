# /// script
# requires-python = "==3.12.*"
# dependencies = [
#     "build123d==0.13.0",
#     "numpy==2.1.3",
#     "rtree==1.4.1",
#     "trimesh==4.5.3",
# ]
# [tool.uv]
# exclude-newer = "2026-10-01T00:00:00Z"
# ///
"""Build the parametric reference parts, check them against their STLs, and
only then write their STEP files.

    uv run --locked --quiet --script build.py          # gates, export, re-read, publish, import checks
    uv run --locked --quiet --script build.py --check  # gates only, nothing written
    uv run --locked --quiet --script build.py --test   # the unit tests

`mise run cad-reference` runs the tests and the build. Its dependencies are
locked in build.py.lock (`uv lock --script`). A source is a folder here with
a `parts/` folder (discovery.py); the gates are in gates.py, the STEP export
in export.py and the FreeCAD and solidcraft import checks in import_check.py.
Every failure is reported before the build exits non-zero, and nothing is
written unless every part passes every gate and every staged STEP re-reads.
"""

from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))

import export  # noqa: E402
import gates  # noqa: E402
import import_check  # noqa: E402
from discovery import Source, discover, find_sources  # noqa: E402


def run_tests() -> int:
    suite = unittest.defaultTestLoader.discover(str(HERE), pattern="test_*.py", top_level_dir=str(HERE))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


def check_parts(sources: list[Source], samples: int | None = None,
                seed: int | None = None) -> tuple[dict[str, dict[str, object]], list[str]]:
    """Build every part and run its gates; the checked solids per source, and
    every failure."""
    solids: dict[str, dict[str, object]] = {}
    failures: list[str] = []
    for source in sources:
        raw: dict[str, object] = {}
        solids[source.name] = {}
        for part in source.parts:
            label = f"{source.name}/{part.name}"
            try:
                if part.mirror_of is None:
                    result = part.build()
                elif raw.get(part.mirror_of) is not None:
                    result = raw[part.mirror_of].mirror(part.plane)
                else:
                    failures.append(f"{label}: not built, as its mirror {part.mirror_of} did not build")
                    continue
            except Exception as e:  # noqa: BLE001 - a failing build is that part's failure
                failures.append(f"{label}: build raised {type(e).__name__}: {e}")
                continue
            raw[part.name] = result
            kind = f" (mirror of {part.mirror_of})" if part.mirror_of else ""
            print(f"{label}{kind} against {part.stl.relative_to(REPO) if part.stl.is_relative_to(REPO) else part.stl}")
            report = gates.check(label, result, part.stl, part.exceptions, samples=samples, seed=seed)
            for line in report.lines:
                print(line)
            failures += report.failures
            if not report.failures:
                solids[source.name][part.name] = report.solid
    return solids, failures


def report_failures(what: str, failures: list[str]) -> int:
    print(f"\n{len(failures)} {what} failed; nothing written:")
    for f in failures:
        print("  " + f)
    return 1


def main(argv: list[str] | None = None, root: Path = HERE, repo_root: Path = REPO) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="run the gates, write nothing")
    parser.add_argument("--test", action="store_true", help="run the unit tests")
    args = parser.parse_args(argv)

    # An interrupted publish is resolved before anything else, in every mode.
    for d in find_sources(root):
        for line in export.recover(d):
            print(line)
    if args.test:
        return run_tests()

    sources, failures = discover(root, repo_root)
    count = sum(len(s.parts) for s in sources)
    print(f"{len(sources)} source(s), {count} part(s)" + "".join(f"; {s.name}: {len(s.parts)}" for s in sources))
    if failures:
        return report_failures("part contract check(s)", failures)
    if count == 0:
        print("No parts.")
    solids, failures = check_parts(sources)
    if failures:
        return report_failures("gate(s)", failures)
    print("All gates pass.")
    if args.check:
        return 0

    failures = export.export_all([(s.path, solids[s.name]) for s in sources])
    if failures:
        return report_failures("export check(s)", failures)
    for s in sources:
        for name in sorted(solids[s.name]):
            print(f"wrote {(s.path / export.STEP_DIR / export.step_name(name)).relative_to(root)}")

    checked = [import_check.Checked(name, s.path / export.STEP_DIR / export.step_name(name), solid.volume)
               for s in sources for name, solid in sorted(solids[s.name].items())]
    results = import_check.check_all(checked)
    for r in results:
        print(r.line())
    for line in import_check.summary(results):
        print(line)
    if import_check.failed(results):
        print("\nAn import check failed.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
