"""Finding the parts: every `<source>/parts/*.py` not starting with `_`.

A source is a folder beside this file that holds a `parts/` folder. Each
discovered part file defines `PARTS`, a list of declarations, each a dict:
- `name`: upstream's file name without `.stl`;
- `stl`: the reference STL, relative to the repository root;
- `build`: a function returning the solid, in the STL's coordinates; or, for
  the second side of a mirrored pair only, `mirror_of` (the first side's
  name) and `plane` (a build123d `Plane`) instead;
- `exceptions` (optional): per-gate limits, `{gate: {"limit", "feature",
  "reason"}}`, the gates being those in `gates.EXCEPTION_GATES`.

A file declares one part, or two for a mirrored pair. Before any file in
`parts/` is imported (the `_` helpers too), it is parsed and must import only
the allowlist and call no file-reading or code-loading function (design:
"Part contract and discovery"); part code therefore cannot read the STL it
is checked against.
"""

from __future__ import annotations

import ast
import functools
import importlib
import importlib.util
import re
import sys
import types
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

PARTS_DIR = "parts"
# Modules a file in parts/ may import, besides its siblings in parts/.
# __future__ is a compiler directive, not a module that reads anything.
ALLOWED_IMPORTS = frozenset({"__future__", "math", "dataclasses", "typing", "build123d"})
# Built-ins that read files, load code or reach other modules' globals,
# refused as any reference.
BANNED_NAMES = frozenset({"open", "exec", "eval", "compile", "__import__", "getattr", "__builtins__",
                          "globals", "locals", "vars", "breakpoint"})
# build123d's mesh and file importers, refused as a name, an imported name or
# an attribute (so `b.import_stl` fails as well as `import_stl`): these, and
# every name starting with IMPORTER_PREFIX (`import_dxf`, `import_svg`, ...).
BANNED_BUILD123D = frozenset({"import_stl", "import_3mf", "import_step", "import_brep", "Mesher"})
IMPORTER_PREFIX = "import_"
# A helper declares no part; this name in one fails the scan.
PARTS_NAME = "PARTS"


def banned_build123d(name: str) -> bool:
    return name in BANNED_BUILD123D or name.startswith(IMPORTER_PREFIX)


@functools.cache
def module_attributes() -> frozenset[str]:
    """Names under which an allowed module holds another module (such as
    `dataclasses.sys` or `build123d.importers`): refused as attributes, so an
    allowed module is no route to `sys`, `os` or the importers."""
    names = set()
    for mod in sorted(ALLOWED_IMPORTS - {"__future__"}):
        module = importlib.import_module(mod)
        names |= {n for n in dir(module) if isinstance(getattr(module, n, None), types.ModuleType)}
    return frozenset(names)


def dunder(name: str) -> bool:
    return name.startswith("__") and name.endswith("__")
DECLARATION_KEYS = frozenset({"name", "stl", "build", "mirror_of", "plane", "exceptions"})
EXCEPTION_KEYS = frozenset({"limit", "feature", "reason"})


@dataclass
class Declaration:
    name: str
    stl: Path  # absolute
    file: Path
    build: Callable[[], object] | None = None
    mirror_of: str | None = None
    plane: object | None = None
    exceptions: dict[str, dict] = field(default_factory=dict)


@dataclass
class Source:
    name: str
    path: Path
    parts: list[Declaration]


def find_sources(root: Path) -> list[Path]:
    """The source folders under `root`; a symbolic link is not one (it could
    lead recovery and publishing outside the checkout), see `linked_sources`."""
    return sorted(p for p in root.iterdir() if p.is_dir() and not p.is_symlink() and (p / PARTS_DIR).is_dir())


def linked_sources(root: Path) -> list[Path]:
    """Symbolic links under `root` that would otherwise be sources."""
    return sorted(p for p in root.iterdir() if p.is_symlink() and p.is_dir() and (p / PARTS_DIR).is_dir())


def scan_file(path: Path, siblings: set[str], label: str) -> list[str]:
    """The allowlist scan of one file in parts/, before it is imported."""
    try:
        tree = ast.parse(path.read_text(), filename=str(path))
    except SyntaxError as e:
        return [f"{label}: does not parse: {e.msg} (line {e.lineno})"]
    failures = []
    if path.name.startswith("_"):
        # Any binding of PARTS, at any depth and in any target form (a tuple,
        # a conditional, a `global` in a function).
        for node in ast.walk(tree):
            if ((isinstance(node, ast.Name) and node.id == PARTS_NAME and not isinstance(node.ctx, ast.Load))
                    or (isinstance(node, (ast.Global, ast.Nonlocal)) and PARTS_NAME in node.names)):
                failures.append(f"{label}:{node.lineno}: a helper declares no part; move {PARTS_NAME} to a part "
                                "file not starting with _")

    def allowed(module: str) -> bool:
        return module in ALLOWED_IMPORTS or module in siblings

    for node in ast.walk(tree):
        line = getattr(node, "lineno", "?")
        if isinstance(node, ast.Import):
            for alias in node.names:
                if not allowed(alias.name):
                    failures.append(f"{label}:{line}: imports {alias.name}, outside the allowlist")
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                failures.append(f"{label}:{line}: relative import; import a sibling by its name")
            elif not allowed(node.module or ""):
                failures.append(f"{label}:{line}: imports from {node.module}, outside the allowlist")
            for alias in node.names:
                if alias.name == "*":
                    failures.append(f"{label}:{line}: imports *; import each name, so the scan sees it")
                elif banned_build123d(alias.name):
                    failures.append(f"{label}:{line}: imports {alias.name}, which reads a file")
                elif dunder(alias.name) or alias.name in module_attributes():
                    failures.append(f"{label}:{line}: imports {alias.name}, another module through an allowed one")
        elif isinstance(node, ast.Name) and (node.id in BANNED_NAMES or banned_build123d(node.id)):
            failures.append(f"{label}:{line}: uses {node.id}, which reads a file or loads code")
        elif isinstance(node, ast.Attribute) and (banned_build123d(node.attr) or dunder(node.attr)
                                                  or node.attr in module_attributes()):
            failures.append(f"{label}:{line}: uses .{node.attr}, which reads a file, loads code or reaches "
                            "another module")
    return failures


def valid_name(name: str) -> bool:
    return (name == name.strip() and not name.startswith(".") and "/" not in name and "\\" not in name
            and all(ch.isprintable() for ch in name))


def module_name(source: str, stem: str) -> str:
    return "_spooder_reference_part__" + re.sub(r"\W", "_", source) + "__" + stem


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def check_exceptions(label: str, name: str, exceptions, gates: frozenset[str]) -> list[str]:
    if not isinstance(exceptions, dict):
        return [f"{label}: {name}: exceptions must be a dict of gate to limit"]
    failures = []
    for gate, spec in exceptions.items():
        if gate not in gates:
            failures.append(f"{label}: {name}: exception for unknown gate {gate!r} (gates: {', '.join(sorted(gates))})")
            continue
        if not isinstance(spec, dict) or set(spec) != EXCEPTION_KEYS:
            failures.append(f"{label}: {name}: exception {gate} needs exactly {', '.join(sorted(EXCEPTION_KEYS))}")
            continue
        limit = spec["limit"]
        if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not limit > 0:
            failures.append(f"{label}: {name}: exception {gate}'s limit must be a positive number")
        for key in ("feature", "reason"):
            if not isinstance(spec[key], str) or not spec[key].strip():
                failures.append(f"{label}: {name}: exception {gate} must name its {key}")
    return failures


def declarations(module, path: Path, label: str, repo_root: Path) -> tuple[list[Declaration], list[str]]:
    """A discovered file's PARTS, checked against the contract."""
    from build123d import Plane

    from gates import EXCEPTION_GATES

    parts = getattr(module, "PARTS", None)
    if not isinstance(parts, (list, tuple)) or not parts:
        return [], [f"{label}: declares no part (PARTS missing or empty)"]
    if len(parts) > 2:
        return [], [f"{label}: declares {len(parts)} parts; a file declares one, or two for a mirrored pair"]
    failures, out = [], []
    for i, d in enumerate(parts):
        if not isinstance(d, dict):
            failures.append(f"{label}: PARTS[{i}] is not a dict")
            continue
        unknown = set(d) - DECLARATION_KEYS
        if unknown:
            failures.append(f"{label}: PARTS[{i}] has unknown keys {', '.join(sorted(unknown))}")
        name, stl = d.get("name"), d.get("stl")
        if not isinstance(name, str) or not name.strip():
            failures.append(f"{label}: PARTS[{i}] has no name")
            continue
        if not valid_name(name):
            failures.append(f"{label}: {name!r}: a part name is one file name (no path separator, control "
                            "character or leading dot), as its STEP file is named after it")
            continue
        repo = repo_root.resolve()
        stl_path = (repo / stl).resolve() if isinstance(stl, str) else None
        if (not isinstance(stl, str) or not stl.lower().endswith(".stl") or Path(stl).is_absolute()
                or not stl_path.is_relative_to(repo)):
            failures.append(f"{label}: {name}: stl must be the reference STL's path relative to the repository "
                            "root, inside it")
            continue
        if not stl_path.is_file():
            failures.append(f"{label}: {name}: STL {stl} not found")
        build, mirror_of, plane = d.get("build"), d.get("mirror_of"), d.get("plane")
        if i == 0:
            if mirror_of is not None or plane is not None:
                failures.append(f"{label}: {name}: the first declaration is modelled; mirror_of and plane belong to "
                                "the second side of a pair")
            if not callable(build):
                failures.append(f"{label}: {name}: build must be a function returning the solid")
        else:
            first = parts[0].get("name") if isinstance(parts[0], dict) else None
            if build is not None:
                failures.append(f"{label}: {name}: the second side of a mirrored pair has no build of its own; "
                                "it is the mirror of the first")
            if mirror_of != first:
                failures.append(f"{label}: {name}: a second declaration must be a mirrored pair's second side, "
                                f"with mirror_of the first side's name ({first!r}), not {mirror_of!r}")
            if not isinstance(plane, Plane):
                failures.append(f"{label}: {name}: a mirrored side needs plane, a build123d Plane")
        exceptions = d.get("exceptions", {})
        failures += check_exceptions(label, name, exceptions, EXCEPTION_GATES)
        out.append(Declaration(name, stl_path, path, build=build, mirror_of=mirror_of, plane=plane,
                               exceptions=dict(exceptions) if isinstance(exceptions, dict) else {}))
    return out, failures


def load_source(source_dir: Path, root: Path, repo_root: Path) -> tuple[Source, list[str]]:
    parts_dir = source_dir / PARTS_DIR
    files = sorted(parts_dir.glob("*.py"))
    siblings = {f.stem for f in files}
    failures = []
    for f in files:
        failures += scan_file(f, siblings, str(f.relative_to(root)))
    source = Source(source_dir.name, source_dir, [])
    if failures:
        return source, failures  # nothing in this source is imported
    sys.path.insert(0, str(parts_dir))
    try:
        for f in files:
            if f.name.startswith("_"):
                continue
            label = str(f.relative_to(root))
            try:
                module = load_module(f, module_name(source_dir.name, f.stem))
            except Exception as e:  # noqa: BLE001 - any error in a part file is its failure
                failures.append(f"{label}: import failed: {type(e).__name__}: {e}")
                continue
            decls, errs = declarations(module, f, label, repo_root)
            source.parts += decls
            failures += errs
    finally:
        sys.path.remove(str(parts_dir))
        # Siblings imported by their bare names belong to this source only.
        for key, mod in list(sys.modules.items()):
            mfile = getattr(mod, "__file__", None)
            if mfile and Path(mfile).parent == parts_dir and not key.startswith("_spooder_reference_part__"):
                del sys.modules[key]
    return source, failures


def discover(root: Path, repo_root: Path) -> tuple[list[Source], list[str]]:
    """Every source under `root` with its parts, and every contract failure."""
    sources, failures = [], []
    for d in linked_sources(root):
        failures.append(f"{d.relative_to(root)}: a source is a folder here, not a symbolic link")
    for d in find_sources(root):
        source, errs = load_source(d, root, repo_root)
        sources.append(source)
        failures += errs
    names: dict[str, Path] = {}
    stls: dict[Path, str] = {}
    for source in sources:
        for p in source.parts:
            label = str(p.file.relative_to(root))
            if p.name in names:
                failures.append(f"{label}: part name {p.name!r} is also declared in {names[p.name].relative_to(root)}")
            else:
                names[p.name] = p.file
            if p.stl in stls:
                failures.append(f"{label}: {p.name}: STL {p.stl.name} is also declared by {stls[p.stl]}")
            else:
                stls[p.stl] = p.name
    return sources, failures
