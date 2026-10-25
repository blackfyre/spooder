"""Check that the native test env compiles exactly the core/ sources.

`[env:native]` `build_src_filter` in platformio.ini must be `-<*>` followed by
one plain `+<path.cpp>` entry per `.cpp` file under core/ (path relative to
core/, no wildcard, no `.` or `..`). The check prints, sorted, each source
missing from the filter, each entry with no file under core/ and each entry of
any other form, and exits 1 when there is any. The filter checked is the one
PlatformIO uses: the file's, with PLATFORMIO_BUILD_SRC_FILTER appended. Run by `mise run test` before
`pio test`; tested by test_check_test_sources.py.

Usage: python check_test_sources.py [project_dir]  (default: the repository)
"""

import configparser
import os
import pathlib
import re
import sys

ENV = "env:native"
OPTION = "build_src_filter"
SRC_DIR = "core"
EXCLUDE_ALL = "-<*>"
ENV_VARIABLE = "PLATFORMIO_BUILD_SRC_FILTER"
OLD_ENV_VARIABLE = "PLATFORMIO_SRC_FILTER"

# PlatformIO's own split of a filter: `+<…>` or `-<…>`; anything else that is
# not whitespace is kept as a token so it is reported, not skipped.
_TOKEN = re.compile(r"[+-]<[^>]*>|\S+")
_INCLUDE = re.compile(r"\+<([^>]*)>")
_WILDCARD = set("*?[]{}")
_INLINE_COMMENT = re.compile(r"\s+;.*$")  # PlatformIO's INLINE_COMMENT_RE


class CheckError(Exception):
    """The filter could not be read at all."""


def env_filter(environ):
    """The filter PlatformIO appends from the environment, as it reads it.

    PlatformIO 6.2 (project/config.py) appends PLATFORMIO_BUILD_SRC_FILTER, or
    when that is empty the old name PLATFORMIO_SRC_FILTER, to the configured
    build_src_filter, so the check sees the same effective filter.
    """
    return environ.get(ENV_VARIABLE) or environ.get(OLD_ENV_VARIABLE) or ""


def read_filter(ini_path, environ=None):
    """Return the native env's effective build_src_filter entries, in order."""
    parser = configparser.ConfigParser(interpolation=None, inline_comment_prefixes=(";", "#"))
    try:
        with open(ini_path, encoding="utf-8") as fp:
            parser.read_file(fp)
    except (OSError, configparser.Error) as exc:
        raise CheckError("cannot read %s: %s" % (ini_path, exc)) from exc
    if not parser.has_option(ENV, OPTION):
        raise CheckError("%s has no [%s] %s" % (ini_path, ENV, OPTION))
    value = parser.get(ENV, OPTION)
    appended = env_filter(os.environ if environ is None else environ)
    if appended:
        value += "\n" + appended
    return [token for item in multi_values(value) for token in _TOKEN.findall(item)]


def multi_values(value):
    """Split a multi-value option as PlatformIO 6.2 does (parse_multi_values).

    Lines (or `, `-separated items of a single line) are stripped; blank lines
    and lines starting with `;` or `#` are dropped, and a ` ;` inline comment is
    cut off.
    """
    items = []
    for item in value.split("\n" if "\n" in value else ", "):
        item = item.strip()
        if not item or item.startswith((";", "#")):
            continue
        items.append(_INLINE_COMMENT.sub("", item).strip())
    return items


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
    source = "platformio.ini" + (" with %s appended" % ENV_VARIABLE if env_filter(os.environ) else "")
    sys.stderr.write("Test source check FAILED ([%s] %s in %s):\n" % (ENV, OPTION, source))
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
