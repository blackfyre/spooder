"""Run inside FreeCADCmd: import each STEP and report what FreeCAD reads.

    flatpak run --env=SPOODER_STEP_FILES=<a.step>:<b.step> --command=FreeCADCmd org.freecad.FreeCAD freecad_import.py

FreeCADCmd opens any further argument as a document, so the files come in
SPOODER_STEP_FILES (absolute paths, separated by os.pathsep). Prints one line,
the marker and a JSON object: FreeCAD's version and, per file, the number of
solids, whether each is closed and valid, and the volume, or the error.
"""

import json
import os

import FreeCAD  # noqa: F401 - provided by FreeCADCmd
import Part

MARKER = "SPOODER-FREECAD-JSON:"


def inspect(path: str) -> dict:
    try:
        shape = Part.read(path)
    except Exception as e:  # noqa: BLE001 - reported per file
        return {"error": str(e)}
    solids = shape.Solids
    return {
        "solids": len(solids),
        "closed": all(s.isClosed() for s in solids) and bool(solids),
        "valid": shape.isValid(),
        "volume": sum(s.Volume for s in solids),
    }


files = [f for f in os.environ.get("SPOODER_STEP_FILES", "").split(os.pathsep) if f]
print(MARKER + json.dumps({"version": ".".join(FreeCAD.Version()[:3]), "files": {f: inspect(f) for f in files}}))
