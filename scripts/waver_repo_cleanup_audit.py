#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import json
import os
from pathlib import Path


GENERATED_DIRS = {
    "build",
    "install",
    "log",
    ".pytest_cache",
    "__pycache__",
    "experiment_results",
    "experiments_result",
    "bags",
    "rosbag",
    "rosbags",
    "waver_experiments",
}
GENERATED_PATH_EXCEPTIONS = {
    # Vizanti ships a UI template named "rosbag"; it is source, not a generated bag.
    "src/ugv_else/vizanti/vizanti_server/public/templates/rosbag",
}
VENDOR_HINTS = {
    "src/livox_ros_driver2",
    "src/Livox-SDK2",
    "src/ugv_else",
    "src/ugv_main/ugv_gazebo",
}
BACKUP_TOKENS = ("backup", ".bak", ".old", "_old", "~")
LARGE_BYTES = 5 * 1024 * 1024


def rel(path: Path, root: Path) -> str:
    return str(path.relative_to(root))


def package_names(root: Path) -> list[dict[str, str]]:
    packages = []
    for pkg in sorted((root / "src").rglob("package.xml")):
        text = pkg.read_text(errors="replace")
        name = ""
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("<name>") and line.endswith("</name>"):
                name = line.replace("<name>", "").replace("</name>", "").strip()
                break
        packages.append({"name": name or "UNKNOWN", "path": rel(pkg.parent, root)})
    return packages


def console_scripts(setup_py: Path) -> list[str]:
    try:
        tree = ast.parse(setup_py.read_text(errors="replace"))
    except SyntaxError:
        return []
    scripts: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == "entry_points":
            value = ast.literal_eval(node.value)
            for item in value.get("console_scripts", []):
                scripts.append(str(item))
    return sorted(scripts)


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit Waver workspace cleanup candidates.")
    parser.add_argument("--root", default=".", help="FSD_Vehicle root")
    parser.add_argument("--json", action="store_true", help="Emit JSON only")
    parser.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="Output format. Equivalent to --json when set to json.",
    )
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()

    generated = []
    python_cache_files = []
    backups = []
    large_files = []
    setup_scripts = {}
    todo_files = []
    package_xml_with_todo = []

    for path in sorted(root.rglob("*")):
        if ".git" in path.parts:
            continue
        r = rel(path, root)
        parts = set(path.relative_to(root).parts)
        if path.is_dir() and path.name in GENERATED_DIRS and r not in GENERATED_PATH_EXCEPTIONS:
            generated.append(r)
        if path.is_file():
            if path.name.endswith((".pyc", ".pyo", ".pyd")) or "__pycache__" in path.parts:
                python_cache_files.append(r)
            lower = path.name.lower()
            if any(token in lower for token in BACKUP_TOKENS):
                backups.append(r)
            try:
                size = path.stat().st_size
            except OSError:
                size = 0
            if size >= LARGE_BYTES:
                large_files.append({"path": r, "bytes": size})
            if path.name == "setup.py":
                setup_scripts[r] = console_scripts(path)
            if path.name == "package.xml":
                text = path.read_text(errors="replace")
                if "TODO" in text or "FIXME" in text:
                    package_xml_with_todo.append(r)
            if path.suffix in {".py", ".cpp", ".hpp", ".launch.py", ".yaml", ".md", ".sh"}:
                text = path.read_text(errors="replace")
                if "TODO" in text or "FIXME" in text:
                    todo_files.append(r)

    vendor_like = [hint for hint in sorted(VENDOR_HINTS) if (root / hint).exists()]
    top_level_scripts = sorted(rel(p, root) for p in root.glob("*.sh"))
    launch_files = sorted(rel(p, root) for p in (root / "src/waver_patrol/launch").glob("*.launch.py"))
    config_files = sorted(rel(p, root) for p in (root / "src/waver_patrol/config").glob("*.yaml"))
    waver_patrol_scripts = sorted(
        rel(p, root) for p in (root / "src/waver_patrol/scripts").glob("*") if p.is_file()
    )
    possible_unreferenced_scripts = [
        p for p in waver_patrol_scripts if p.endswith(".sh") and Path(p).name.startswith(("old_", "unused_", "deprecated_"))
    ]
    possible_unreferenced_launches = [
        p for p in launch_files if Path(p).name.startswith(("old_", "unused_", "deprecated_"))
    ]
    possible_unreferenced_configs = [
        p for p in config_files if Path(p).name.startswith(("old_", "unused_", "deprecated_"))
    ]
    large_files_sorted = sorted(large_files, key=lambda x: x["bytes"], reverse=True)
    result = {
        "root": str(root),
        "packages": package_names(root),
        "generated_dirs_present": generated,
        "python_cache_files": python_cache_files,
        "package_xml_with_todo": package_xml_with_todo,
        "backup_old_tmp_deprecated_files": backups,
        "top_level_scripts": top_level_scripts,
        "waver_patrol_scripts": waver_patrol_scripts,
        "launch_files": launch_files,
        "config_files": config_files,
        "possible_unreferenced_scripts": possible_unreferenced_scripts,
        "possible_unreferenced_launches": possible_unreferenced_launches,
        "possible_unreferenced_configs": possible_unreferenced_configs,
        "large_files": large_files_sorted,
        "large_files_over_5mb": large_files_sorted,
        "console_scripts": setup_scripts,
        "todo_or_fixme_files": todo_files,
        "vendor_like_paths_do_not_cleanup_blindly": vendor_like,
        "source_archive_exclude_candidates": sorted(generated + python_cache_files),
    }
    if args.json or args.format == "json":
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0

    print(f"WAVER_REPO_AUDIT_ROOT={root}")
    print(f"PACKAGE_COUNT={len(result['packages'])}")
    print(f"GENERATED_DIR_COUNT={len(generated)}")
    print(f"PYTHON_CACHE_FILE_COUNT={len(python_cache_files)}")
    print(f"PACKAGE_XML_WITH_TODO_COUNT={len(package_xml_with_todo)}")
    print(f"BACKUP_LIKE_FILE_COUNT={len(backups)}")
    print(f"LARGE_FILE_COUNT={len(large_files)}")
    print("VENDOR_LIKE_PATHS=" + ",".join(vendor_like))
    print("\nLarge files:")
    for item in result["large_files"][:20]:
        print(f"- {item['path']} {item['bytes']} bytes")
    print("\nBackup-like files:")
    for item in backups[:50]:
        print(f"- {item}")
    print("\nGenerated dirs:")
    for item in generated[:50]:
        print(f"- {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
