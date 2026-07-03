#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import tarfile
import zipfile
from pathlib import Path


FORBIDDEN_DIRS = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
}

FORBIDDEN_FILENAMES = {
    ".env",
    "waver_field_env",
    "waver_field_env.local",
}

ALLOWED_FILENAMES = {
    ".env.example",
    "waver_field_env.example",
    "waver_field_env.local.example",
}

FORBIDDEN_SUFFIXES = {
    ".db3",
    ".mcap",
    ".bag",
    ".log",
}

ABSOLUTE_SYMLINK_PREFIXES = ("/home/", "/mnt/", "/ros2_ws/")


def normalize(name: str) -> str:
    return name.strip("/").replace("\\", "/")


def check_path(name: str, findings: list[str]) -> None:
    path = normalize(name)
    if not path:
        return
    parts = [part for part in path.split("/") if part]
    basename = parts[-1]
    if basename in ALLOWED_FILENAMES:
        return
    if any(part in FORBIDDEN_DIRS for part in parts):
        findings.append(f"forbidden generated/private directory in archive: {path}")
    if basename in FORBIDDEN_FILENAMES or basename.startswith(".env."):
        findings.append(f"forbidden private file in archive: {path}")
    if any(basename.endswith(suffix) for suffix in FORBIDDEN_SUFFIXES):
        findings.append(f"forbidden data/log file in archive: {path}")
    if fnmatch.fnmatch(basename, "FSD_Vehicle_source_*.tar.gz"):
        findings.append(f"nested source archive inside source tree: {path}")


def check_tar(path: Path) -> list[str]:
    findings: list[str] = []
    with tarfile.open(path, "r:*") as tar:
        for member in tar.getmembers():
            check_path(member.name, findings)
            if member.issym():
                target = member.linkname or ""
                if target.startswith(ABSOLUTE_SYMLINK_PREFIXES):
                    findings.append(f"absolute host symlink target in archive: {member.name} -> {target}")
    return findings


def check_zip(path: Path) -> list[str]:
    findings: list[str] = []
    with zipfile.ZipFile(path) as zf:
        for info in zf.infolist():
            check_path(info.filename, findings)
            mode = (info.external_attr >> 16) & 0o170000
            if mode == 0o120000:
                target = zf.read(info.filename).decode(errors="replace")
                if target.startswith(ABSOLUTE_SYMLINK_PREFIXES):
                    findings.append(f"absolute host symlink target in archive: {info.filename} -> {target}")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description="Check a Waver paper submission source package for private/generated artifacts.")
    parser.add_argument("--path", required=True, help="Archive path (.zip, .tar, .tar.gz, .tgz)")
    args = parser.parse_args()

    package = Path(args.path).expanduser().resolve()
    if not package.exists():
        raise SystemExit(f"package does not exist: {package}")
    if zipfile.is_zipfile(package):
        findings = check_zip(package)
    elif tarfile.is_tarfile(package):
        findings = check_tar(package)
    else:
        raise SystemExit(f"unsupported archive format: {package}")

    if findings:
        print("SUBMISSION_PACKAGE_CHECK=FAIL")
        for item in findings:
            print(f"- {item}")
        return 1
    print("SUBMISSION_PACKAGE_CHECK=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
