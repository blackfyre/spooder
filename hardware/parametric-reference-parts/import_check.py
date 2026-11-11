"""Import checks of the committed STEP files in FreeCAD and solidcraft-cli
(design: "Import checks").

- FreeCAD: `freecad_import.py` under FreeCADCmd, from `PATH` or the flatpak
  `org.freecad.FreeCAD`; the files go by absolute path, which the flatpak
  reads through its host file-system access.
- solidcraft: `solidcraft-cli eval <file>`, whose JSON gives `bodies[]` with
  `closed` and `volume_mm3`.

Each part must import as one closed solid (FreeCAD: valid too) with its
volume within 0.1 % of the built part's. Each check prints `PASS` with the
tool's version, `FAIL` with the measured value, or `SKIPPED` with the reason;
the summary counts each per tool. A skip is never a pass and never fails the
build; a failure does.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
FREECAD_SCRIPT = HERE / "freecad_import.py"
FREECAD_FLATPAK = "org.freecad.FreeCAD"
FREECAD_MARKER = "SPOODER-FREECAD-JSON:"
FREECAD = "FreeCAD"
SOLIDCRAFT = "solidcraft"
TOOLS = (FREECAD, SOLIDCRAFT)
# Source: design.md, "Tolerances": the STEP holds the same exact geometry.
VOLUME_PCT = 0.1
TIMEOUT = 600  # s, per tool run


@dataclass(frozen=True)
class Checked:
    """One STEP file to check, with the built part's volume."""
    name: str
    step: Path
    volume: float


@dataclass(frozen=True)
class Result:
    part: str
    tool: str
    status: str  # PASS, FAIL or SKIPPED
    detail: str

    def line(self) -> str:
        return f"{self.status} {self.part} [{self.tool}] {self.detail}"


def run(cmd: list[str], env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=TIMEOUT, env=env)


def freecad_command() -> list[str] | None:
    """How to start FreeCADCmd here, or None when it is not installed."""
    if shutil.which("FreeCADCmd"):
        return ["FreeCADCmd"]
    if shutil.which("flatpak") and run(["flatpak", "info", FREECAD_FLATPAK]).returncode == 0:
        return ["flatpak", "run", "--command=FreeCADCmd", FREECAD_FLATPAK]
    return None


def solidcraft_command() -> list[str] | None:
    return ["solidcraft-cli"] if shutil.which("solidcraft-cli") else None


def is_number(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def is_count(x) -> bool:
    return isinstance(x, int) and not isinstance(x, bool)


def volume_failure(measured: float, built: float) -> str | None:
    if not abs(measured - built) <= built * VOLUME_PCT / 100:  # NaN fails too
        pct = abs(measured - built) / built * 100 if built else float("inf")
        return f"volume {measured:.3f} mm3, built {built:.3f} mm3 ({pct:.3f} %; limit {VOLUME_PCT} %)"
    return None


def judge(part: Checked, tool: str, version: str, solids: int, closed: bool, valid: bool | None,
          volume: float) -> Result:
    problems = []
    if solids != 1:
        problems.append(f"{solids} solids, not one")
    if not closed:
        problems.append("an open shell")
    if valid is False:
        problems.append("not a valid shape")
    if solids == 1 and (v := volume_failure(volume, part.volume)):
        problems.append(v)
    if problems:
        return Result(part.name, tool, "FAIL", f"{tool} {version}: " + "; ".join(problems))
    return Result(part.name, tool, "PASS", f"{tool} {version}: one closed solid, volume {volume:.3f} mm3")


def check_freecad(parts: list[Checked], command: list[str] | None) -> list[Result]:
    if command is None:
        return [Result(p.name, FREECAD, "SKIPPED", f"FreeCAD not found (no FreeCADCmd on PATH, no flatpak "
                                                   f"{FREECAD_FLATPAK})") for p in parts]
    if not parts:
        return []
    files = os.pathsep.join(str(p.step.resolve()) for p in parts)
    if command[0] == "flatpak":
        cmd = [*command[:2], f"--env=SPOODER_STEP_FILES={files}", *command[2:], str(FREECAD_SCRIPT)]
        env = None
    else:
        cmd = [*command, str(FREECAD_SCRIPT)]
        env = {**os.environ, "SPOODER_STEP_FILES": files}
    try:
        proc = run(cmd, env=env)
    except (OSError, subprocess.TimeoutExpired) as e:
        return [Result(p.name, FREECAD, "FAIL", f"FreeCAD did not run: {e}") for p in parts]
    payload = next((ln[len(FREECAD_MARKER):] for ln in proc.stdout.splitlines() if ln.startswith(FREECAD_MARKER)),
                   None)
    if payload is None:
        tail = (proc.stdout + proc.stderr).strip().splitlines()[-3:]
        return [Result(p.name, FREECAD, "FAIL", f"FreeCAD gave no result (exit {proc.returncode}): "
                       + " | ".join(tail)) for p in parts]
    if proc.returncode != 0:
        tail = (proc.stdout + proc.stderr).strip().splitlines()[-3:]
        return [Result(p.name, FREECAD, "FAIL", f"FreeCAD exited {proc.returncode} after its result: "
                       + " | ".join(tail)) for p in parts]
    try:
        data = json.loads(payload)
        version = data["version"]
        if not isinstance(version, str) or not version.strip():
            raise TypeError("no version")
        files = data["files"]
        if not isinstance(files, dict):
            raise TypeError("files is not an object")
    except (json.JSONDecodeError, AttributeError, KeyError, TypeError) as e:
        return [Result(p.name, FREECAD, "FAIL", f"FreeCAD gave an unreadable result: {e}") for p in parts]
    results = []
    for p in parts:
        got = files.get(str(p.step.resolve()))
        if not isinstance(got, dict) or "error" in got:
            why = got["error"] if isinstance(got, dict) else "not reported"
            results.append(Result(p.name, FREECAD, "FAIL", f"FreeCAD {version}: does not open {p.step}: {why}"))
            continue
        fields = got.get("solids"), got.get("closed"), got.get("valid"), got.get("volume")
        if not (is_count(fields[0]) and isinstance(fields[1], bool) and isinstance(fields[2], bool)
                and is_number(fields[3])):
            results.append(Result(p.name, FREECAD, "FAIL", f"FreeCAD {version}: unreadable result {got!r}"))
            continue
        results.append(judge(p, FREECAD, version, fields[0], fields[1], fields[2], float(fields[3])))
    return results


def check_solidcraft(parts: list[Checked], command: list[str] | None) -> list[Result]:
    if command is None:
        return [Result(p.name, SOLIDCRAFT, "SKIPPED", "solidcraft-cli not found on PATH") for p in parts]
    if not parts:
        return []
    try:
        version_proc = run([*command, "--version"])
    except (OSError, subprocess.TimeoutExpired) as e:
        return [Result(p.name, SOLIDCRAFT, "FAIL", f"solidcraft-cli did not run: {e}") for p in parts]
    version_out = version_proc.stdout.strip()
    if version_proc.returncode != 0 or not version_out:
        return [Result(p.name, SOLIDCRAFT, "FAIL", f"solidcraft-cli --version exited {version_proc.returncode} "
                       f"with {version_out or 'no output'!r}") for p in parts]
    version = version_out.split()[-1]
    results = []
    for p in parts:
        try:
            proc = run([*command, "eval", str(p.step.resolve())])
            data = json.loads(proc.stdout) if proc.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as e:
            results.append(Result(p.name, SOLIDCRAFT, "FAIL", f"solidcraft {version}: eval failed: {e}"))
            continue
        if data is None:
            why = (proc.stderr or proc.stdout).strip().splitlines()[-1:] or ["no output"]
            results.append(Result(p.name, SOLIDCRAFT, "FAIL", f"solidcraft {version}: eval exit {proc.returncode}: "
                                  + why[0]))
            continue
        try:
            bodies = data["bodies"]
            if not isinstance(bodies, list) or not all(isinstance(b, dict) for b in bodies):
                raise TypeError("bodies is not a list of objects")
            if not all(isinstance(b.get("closed"), bool) and is_number(b.get("volume_mm3")) for b in bodies):
                raise TypeError("a body lacks a boolean closed or a numeric volume_mm3")
            closed = bool(bodies) and all(b["closed"] for b in bodies)
            volume = float(sum(b["volume_mm3"] for b in bodies))
        except (KeyError, TypeError, ValueError) as e:
            results.append(Result(p.name, SOLIDCRAFT, "FAIL", f"solidcraft {version}: unreadable result: {e}"))
            continue
        results.append(judge(p, SOLIDCRAFT, version, len(bodies), closed, None, volume))
    return results


def check_all(parts: list[Checked]) -> list[Result]:
    return check_freecad(parts, freecad_command()) + check_solidcraft(parts, solidcraft_command())


def summary(results: list[Result]) -> list[str]:
    lines = []
    for tool in TOOLS:
        mine = [r for r in results if r.tool == tool]
        counts = {s: sum(r.status == s for r in mine) for s in ("PASS", "FAIL", "SKIPPED")}
        line = f"import checks, {tool}: {counts['PASS']} passed, {counts['FAIL']} failed, {counts['SKIPPED']} skipped"
        if counts["SKIPPED"]:
            reasons = sorted({r.detail for r in mine if r.status == "SKIPPED"})
            line += " (" + "; ".join(reasons) + ")"
        lines.append(line)
    return lines


def failed(results: list[Result]) -> bool:
    return any(r.status == "FAIL" for r in results)
