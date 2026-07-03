#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    # 역할: 논문용 pre-real plot 이름을 만들되 기존 plotter를 재사용한다.
    script_dir = Path(__file__).resolve().parent
    root = script_dir.parents[2]
    parser = argparse.ArgumentParser(description="Plot Waver pre-real Gazebo validation results")
    parser.add_argument("--results_dir", default=str(root / "experiments_result" / "results"))
    args = parser.parse_args()
    results_dir = Path(args.results_dir).expanduser()
    script = script_dir / "plot_gazebo_trial_results.py"
    code = subprocess.call([sys.executable, str(script), "--results_dir", str(results_dir)])
    copies = {
        "target_height_plot.png": "pre_real_target_height_plot.png",
        "mission_success_plot.png": "pre_real_safety_gate_result.png",
    }
    for src, dst in copies.items():
        src_path = results_dir / src
        if src_path.exists():
            shutil.copyfile(src_path, results_dir / dst)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
