#!/usr/bin/env python3
"""Placeholder helper for rosbag-to-CSV post-processing.

The online experiment logger writes curated CSV during the run. Use this script
as a starting point when raw rosbag2 replay/export is needed for a paper table.
"""

from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--out", default="csv_export", help="output directory")
    args = parser.parse_args()
    print(f"TODO: replay/export bag={args.bag} to {args.out}. Prefer rosbag2_py for full fidelity.")


if __name__ == "__main__":
    main()
