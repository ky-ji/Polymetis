"""Resolve package provenance without shell commands or process-wide chdir."""
import json
import os
from pathlib import Path
import subprocess


def _version():
    explicit = os.environ.get("POLYMETIS_BUILD_VERSION")
    if explicit:
        return explicit
    package = Path(__file__).resolve().parent
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix and Path(prefix).resolve() in package.parents:
        for record in sorted((Path(prefix) / "conda-meta").glob("polymetis-*.json")):
            info = json.loads(record.read_text())
            if info.get("name") == "polymetis":
                return info["version"]
    try:
        result = subprocess.run(["git", "rev-parse", "--short=12", "HEAD"], cwd=str(package),
                                check=True, capture_output=True, text=True, timeout=2)
        return "0.2+" + result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "0.2_local"


__version__ = _version()
