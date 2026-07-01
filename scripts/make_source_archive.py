#!/usr/bin/env python3
from __future__ import annotations

import argparse
import tarfile
import time
from pathlib import Path


EXCLUDED_DIR_NAMES = {
    ".git",
    ".colcon",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "bags",
    "rosbag",
    "rosbags",
    "experiment_results",
    "experiments_result",
    "experiments_result_compact",
    "waver_experiments",
    "quality_gate",
    "agent_iterations",
    "metrics",
    "plots",
    "annotations",
    "screenshots",
    "images",
}

EXCLUDED_SUFFIXES = {
    ".pyc",
    ".pyo",
    ".pyd",
    ".bag",
    ".db3",
    ".mcap",
    ".sqlite3",
    ".log",
    ".zip",
    ".tar",
    ".tgz",
    ".gz",
}

EXCLUDED_FILE_NAMES = {
    ".env",
    ".env.local",
    ".env.private",
}


def _parts(path: Path) -> tuple[str, ...]:
    return tuple(part for part in path.parts if part not in {"", "."})


def should_exclude(path: Path, root: Path) -> bool:
    """Return true when a path is generated, local, or unsafe for source release."""
    try:
        rel = path.relative_to(root)
    except ValueError:
        rel = path
    parts = _parts(rel)
    if not parts:
        return False
    if parts[-1] in EXCLUDED_FILE_NAMES:
        return True
    if parts[-1].startswith(".env.") and parts[-1] != ".env.example":
        return True
    if any(part in EXCLUDED_DIR_NAMES for part in parts):
        return True
    if any(part.startswith("rosbag") for part in parts):
        return True
    if any(part.startswith("waver_experiments") for part in parts):
        return True
    return path.suffix in EXCLUDED_SUFFIXES


def iter_source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for path in sorted(root.rglob("*")):
        if should_exclude(path, root):
            if path.is_dir():
                # rglob cannot prune; children are filtered by the same predicate.
                continue
            continue
        if path.is_file():
            files.append(path)
    return files


def top_level_summary(root: Path) -> tuple[list[str], list[str]]:
    included: set[str] = set()
    excluded: set[str] = set()
    for child in sorted(root.iterdir()):
        if should_exclude(child, root):
            excluded.add(child.name)
        else:
            included.add(child.name)
    return sorted(included), sorted(excluded)


def make_archive(root: Path, output: Path) -> int:
    files = iter_source_files(root)
    output.parent.mkdir(parents=True, exist_ok=True)
    base = root.name
    with tarfile.open(output, "w:gz") as tar:
        for path in files:
            tar.add(path, arcname=str(Path(base) / path.relative_to(root)))
    return len(files)


def main() -> int:
    parser = argparse.ArgumentParser(description="Create a clean Waver source archive.")
    parser.add_argument("--root", default=".", help="FSD_Vehicle workspace root")
    parser.add_argument("--output", default="", help="Output .tar.gz path")
    parser.add_argument("--dry-run", action="store_true", help="List source/excluded top-level paths without writing")
    parser.add_argument("--list", action="store_true", help="List included files")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    if not root.exists():
        raise SystemExit(f"root does not exist: {root}")
    output = Path(args.output).expanduser().resolve() if args.output else root.parent / f"{root.name}_source_{time.strftime('%Y%m%d_%H%M%S')}.tar.gz"

    included, excluded = top_level_summary(root)
    print(f"SOURCE_ARCHIVE_ROOT={root}")
    print(f"SOURCE_ARCHIVE_OUTPUT={output}")
    print("TOP_LEVEL_INCLUDED=" + ",".join(included))
    if not args.list:
        print("TOP_LEVEL_EXCLUDED=" + ",".join(excluded))
    files = iter_source_files(root)
    print(f"INCLUDED_FILE_COUNT={len(files)}")
    if args.list:
        for path in files:
            print(path.relative_to(root))
    if args.dry_run:
        print("DRY_RUN=1 archive not written")
        return 0
    count = make_archive(root, output)
    print(f"ARCHIVE_WRITTEN={output}")
    print(f"ARCHIVE_FILE_COUNT={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
