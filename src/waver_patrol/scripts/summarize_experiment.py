#!/usr/bin/env python3
"""Small CSV summary helper for Waver experiment folders."""

from __future__ import annotations

import argparse
import csv
import os


def count_rows(path: str) -> int:
    if not os.path.exists(path):
        return 0
    with open(path, newline="", encoding="utf-8") as f:
        return max(0, sum(1 for _ in csv.DictReader(f)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="~/ros2_ws/waver_experiments/<run>")
    args = parser.parse_args()
    run_dir = os.path.expanduser(args.run_dir)
    for name in sorted(f for f in os.listdir(run_dir) if f.endswith(".csv")):
        print(f"{name}: {count_rows(os.path.join(run_dir, name))} rows")


if __name__ == "__main__":
    main()
