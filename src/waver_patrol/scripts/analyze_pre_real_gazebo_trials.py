#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    # 역할: pre-real 이름으로 기존 Gazebo trial analyzer를 호출한다.
    parser = argparse.ArgumentParser(description="Analyze Waver pre-real Gazebo validation trials")
    parser.add_argument("--input_dir", default="~/ugv_ws/FSD_Vehicle/experiments_result")
    parser.add_argument("--output_dir", default="~/ugv_ws/FSD_Vehicle/experiments_result/results")
    args = parser.parse_args()
    script = Path("~/ugv_ws/FSD_Vehicle/src/src/waver_patrol/scripts/analyze_gazebo_trials.py").expanduser()
    return subprocess.call(
        [sys.executable, str(script), "--input_dir", args.input_dir, "--output_dir", args.output_dir]
    )


if __name__ == "__main__":
    raise SystemExit(main())
