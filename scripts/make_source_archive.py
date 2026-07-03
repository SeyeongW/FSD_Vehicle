#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import subprocess
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

EXCLUDED_REL_PATHS = {
    "config/waver_field_env",
    "config/waver_field_env.local",
    "reports/source_manifest.json",
    "reports/source_sha256_manifest.csv",
}

EXCLUDED_REL_PREFIXES = (
    "maps/archive/",
    "reports/field_docker_ssh_check/",
    "reports/full_readiness_loop/",
    "reports/gazebo_functional_validation/",
    "reports/remote_ui_validation/",
    "reports/pre_existing_",
)


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
    rel_posix = rel.as_posix()
    if rel_posix in EXCLUDED_REL_PATHS:
        return True
    if any(rel_posix.startswith(prefix) for prefix in EXCLUDED_REL_PREFIXES):
        return True
    if parts[-1] in EXCLUDED_FILE_NAMES:
        return True
    if parts[-1].startswith(".env.") and parts[-1] != ".env.example":
        return True
    if any(part in EXCLUDED_DIR_NAMES for part in parts):
        return True
    if any(part.startswith("rosbag") and part != "rosbag_replay" for part in parts):
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
    generated = generated_release_metadata(root, files)
    write_generated_metadata(root, generated)
    output.parent.mkdir(parents=True, exist_ok=True)
    base = root.name
    with tarfile.open(output, "w:gz") as tar:
        for path in files:
            tar.add(path, arcname=str(Path(base) / path.relative_to(root)))
        for rel_path, payload in generated.items():
            info = tarfile.TarInfo(str(Path(base) / rel_path))
            info.size = len(payload)
            info.mtime = int(time.time())
            info.mode = 0o644
            tar.addfile(info, io.BytesIO(payload))
    return len(files) + len(generated)


def git_output(root: Path, args: list[str]) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(root), *args], text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "UNKNOWN"


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    if path.is_symlink():
        h.update(f"SYMLINK->{path.readlink()}".encode())
        return h.hexdigest()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def generated_release_metadata(root: Path, files: list[Path]) -> dict[Path, bytes]:
    status = git_output(root, ["status", "--short"])
    status_lines = [line for line in status.splitlines() if line and line != "UNKNOWN"]
    dirty_count = sum(1 for line in status_lines if not line.startswith("??"))
    untracked_count = sum(1 for line in status_lines if line.startswith("??"))
    rows = [
        {"path": path.relative_to(root).as_posix(), "sha256": file_sha256(path), "type": "source_file"}
        for path in files
    ]
    generated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    manifest = {
        "generated_at": generated_at,
        "source_root": root.name,
        "git_branch_or_unknown": git_output(root, ["branch", "--show-current"]),
        "git_commit_or_unknown": git_output(root, ["rev-parse", "--short", "HEAD"]),
        "git_dirty_summary": {
            "dirty_file_count": dirty_count,
            "untracked_file_count": untracked_count,
            "status_short": status_lines,
            "submodule_status": git_output(root, ["submodule", "status", "--recursive"]).splitlines(),
        },
        "artifact_type": "source_release",
        "excluded_policy_version": "source_release_policy_v2",
        "included_file_count": len(files) + 2,
        "sha256_manifest_path": "reports/source_sha256_manifest.csv",
        "note": "The sha256 manifest lists source files and source_manifest.json; it intentionally excludes itself.",
    }
    manifest_payload = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
    rows.append({"path": "reports/source_manifest.json", "sha256": hashlib.sha256(manifest_payload).hexdigest(), "type": "generated_manifest"})

    csv_buf = io.StringIO()
    writer = csv.DictWriter(csv_buf, fieldnames=["path", "sha256", "type"], lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return {
        Path("reports/source_manifest.json"): manifest_payload,
        Path("reports/source_sha256_manifest.csv"): csv_buf.getvalue().encode(),
    }


def write_generated_metadata(root: Path, generated: dict[Path, bytes]) -> None:
    for rel_path, payload in generated.items():
        path = root / rel_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


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
