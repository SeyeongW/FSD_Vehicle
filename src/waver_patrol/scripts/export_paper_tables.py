#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


TABLE_GROUPS = {
    "navigation": "navigation.",
    "lidar": "lidar.",
    "inspection": "inspection.",
    "camera": "camera.",
    "classification": "classification.",
    "sound": "sound.",
    "return": "return.",
    "safety": "safety.",
    "slam": "slam.",
}


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: export_paper_tables.py <experiment_result_dir>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).expanduser().resolve()
    summary = root / "paper_metrics_summary.json"
    if not summary.exists():
        print(f"missing {summary}; run compute_paper_metrics.py first", file=sys.stderr)
        return 2
    metrics = json.loads(summary.read_text(encoding="utf-8"))
    out_dir = root / "paper_tables"
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, prefix in TABLE_GROUPS.items():
        path = out_dir / f"{name}_metrics.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "value"])
            for key in sorted(k for k in metrics if k.startswith(prefix)):
                value = metrics[key]
                writer.writerow([key.removeprefix(prefix), json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value])
    readme = out_dir / "README.md"
    readme.write_text(
        "Paper metric tables exported from paper_metrics_summary.json.\n"
        "Precision/recall/F1/mAP and ATE/RPE are valid only when external ground truth is supplied; "
        "otherwise the scripts report operational/internal metrics.\n",
        encoding="utf-8",
    )
    print(f"wrote table CSVs under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
