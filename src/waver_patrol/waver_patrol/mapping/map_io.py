from __future__ import annotations

from pathlib import Path


def map_exists(path: str | Path) -> bool:
    path = Path(path).expanduser()
    return path.exists() and path.suffix in {".yaml", ".pbstream", ".db"}
