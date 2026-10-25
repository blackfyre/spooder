"""Check that the native test env compiles exactly the core/ sources.

`[env:native]` `build_src_filter` in platformio.ini must be `-<*>` followed by
one plain `+<path.cpp>` entry per `.cpp` file under core/ (path relative to
core/, no wildcard, no `.` or `..`). The check prints, sorted, each source
missing from the filter, each entry with no file under core/ and each entry of
any other form, and exits 1 when there is any. Run by `mise run test` before
`pio test`; tested by test_check_test_sources.py.

Usage: python check_test_sources.py [project_dir]  (default: the repository)
"""

import configparser
import pathlib
import re
import sys

ENV = "env:native"
OPTION = "build_src_filter"
SRC_DIR = "core"
EXCLUDE_ALL = "-<*>"

# PlatformIO's own split of a filter: `+<…>` or `-<…>`; anything else that is
# not whitespace is kept as a token so it is reported, not skipped.
_TOKEN = re.compile(r"[+-]<[^>]*>|\S+")
_INCLUDE = re.compile(r"\+<([^>]*)>")
_WILDCARD = set("*?[]{}")


class CheckError(Exception):
    """The filter could not be read at all."""


def read_filter(ini_path):
    """Return the native env's build_src_filter entries, in order."""
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=(";", "#"))
    try:
        with open(ini_path, encoding="utf-8") as fp:
            parser.read_file(fp)
    except (OSError, configparser.Error) as exc:
        raise CheckError("cannot read %s: %s" % (ini_path, exc)) from exc
    if not parser.has_option(ENV, OPTION):
        raise CheckError("%s has no [%s] %s" % (ini_path, ENV, OPTION))
    return _TOKEN.findall(parser.get(ENV, OPTION))


def plain_path(path):
    """True when path is a plain relative path to a .cpp file."""
    if not path.endswith(".cpp") or path.startswith("/") or "\\" in path:
        return False
    if any(ch in _WILDCARD for ch in path):
        return False
    return all(part not in ("", ".", "..") for part in path.split("/"))


def core_sources(src_dir):
    """The .cpp files under src_dir, as paths relative to it."""
    return {p.relative_to(src_dir).as_posix() for p in pathlib.Path(src_dir).rglob("*.cpp") if p.is_file()}


def check(project_dir):
    """Return (missing, extra, invalid), each sorted."""
    project = pathlib.Path(project_dir)
    entries = read_filter(project / "platformio.ini")
    invalid = []
    listed = set()
    if not entries or entries[0] != EXCLUDE_ALL:
        invalid.append("(no leading %s)" % EXCLUDE_ALL)
    else:
        entries = entries[1:]
    for entry in entries:
        match = _INCLUDE.fullmatch(entry)
        if match is None or not plain_path(match.group(1)) or match.group(1) in listed:
            invalid.append(entry)
        else:
            listed.add(match.group(1))
    sources = core_sources(project / SRC_DIR)
    return sorted(sources - listed), sorted(listed - sources), sorted(invalid)


def main(argv):
    project = argv[1] if len(argv) > 1 else pathlib.Path(__file__).resolve().parent.parent
    try:
        missing, extra, invalid = check(project)
    except CheckError as exc:
        sys.stderr.write("Test source check FAILED: %s\n" % exc)
        return 1
    if not (missing or extra or invalid):
        print("Test source check: build_src_filter lists every core/ source")
        return 0
    sys.stderr.write("Test source check FAILED ([%s] %s in platformio.ini):\n" % (ENV, OPTION))
    for label, names in (
        ("missing from build_src_filter", missing),
        ("not in core/", extra),
        ("invalid entry", invalid),
    ):
        for name in names:
            sys.stderr.write("  %s: %s\n" % (label, name))
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
