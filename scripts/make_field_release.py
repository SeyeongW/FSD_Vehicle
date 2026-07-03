#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Create Waver field deployment source release.")
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.root).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    subprocess.check_call(["python3", str(root / "scripts/make_source_archive.py"), "--root", str(root), "--output", str(output)])
    print(f"FIELD_RELEASE_WRITTEN={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
