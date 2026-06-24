#!/usr/bin/env python3
from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
from dataclasses import dataclass

import yaml


@dataclass
class TopicInfo:
    publisher_count: int
    publishers: list[str]


def run(command: list[str], timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)


def topic_info(topic: str) -> TopicInfo:
    result = run(["ros2", "topic", "info", "-v", topic], timeout=8.0)
    text = (result.stdout or "") + (result.stderr or "")
    match = re.search(r"Publisher count:\s*(\d+)", text)
    count = int(match.group(1)) if match else 0
    publishers = [
        match.group(1).strip()
        for match in re.finditer(
            r"Node name:\s*([^\n]+)(?:(?!\nNode name:).)*?Endpoint type:\s*PUBLISHER",
            text,
            flags=re.DOTALL,
        )
    ]
    return TopicInfo(count, publishers)


def topic_once(topic: str, timeout: float = 6.0) -> str:
    result = run(["timeout", str(timeout), "ros2", "topic", "echo", "--once", topic], timeout=timeout + 2.0)
    return (result.stdout or "") + (result.stderr or "")


def load_pgm_map(yaml_path: pathlib.Path) -> tuple[dict, int, int, bytes, pathlib.Path] | None:
    if not yaml_path.exists():
        return None
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    image = pathlib.Path(str(meta.get("image", "")))
    if not image.is_absolute():
        image = yaml_path.parent / image
    if not image.exists():
        return None
    raw = image.read_bytes()
    if not raw.startswith(b"P5"):
        return None
    cursor = 0
    header_lines: list[bytes] = []
    while len(header_lines) < 3:
        end = raw.find(b"\n", cursor)
        if end < 0:
            return None
        line = raw[cursor:end].strip()
        cursor = end + 1
        if not line or line.startswith(b"#"):
            continue
        header_lines.append(line)
    if header_lines[0] != b"P5":
        return None
    width, height = [int(v) for v in header_lines[1].split()[:2]]
    data = raw[cursor : cursor + width * height]
    return meta, width, height, data, image


def map_quality(yaml_path: pathlib.Path) -> tuple[bool, str]:
    loaded = load_pgm_map(yaml_path)
    if loaded is None:
        if not yaml_path.exists():
            return False, f"missing_map_yaml={yaml_path}"
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        image = pathlib.Path(str(meta.get("image", "")))
        if not image.is_absolute():
            image = yaml_path.parent / image
        if not image.exists():
            return False, f"missing_map_image={image}"
        return False, f"unsupported_or_bad_map_image={image}"
    _meta, width, height, data, image = loaded
    occupied = sum(1 for value in data if value < 80)
    free = sum(1 for value in data if value > 220)
    unknown = sum(1 for value in data if 80 <= value <= 220)
    known = occupied + free
    total = max(1, width * height)
    known_ratio = known / total
    occupied_points = [(i % width, i // width) for i, value in enumerate(data[: width * height]) if value < 80]
    if occupied_points:
        min_ox = min(x for x, _ in occupied_points)
        max_ox = max(x for x, _ in occupied_points)
        min_oy = min(y for _, y in occupied_points)
        max_oy = max(y for _, y in occupied_points)
        occ_bbox = f"{min_ox},{min_oy}-{max_ox},{max_oy}"
    else:
        occ_bbox = "none"
    ok = occupied >= 30 and free >= 200 and known_ratio >= 0.01
    return ok, (
        f"width={width} height={height} occupied={occupied} free={free} unknown={unknown} "
        f"known_ratio={known_ratio:.4f} occupied_bbox={occ_bbox}"
    )


def obstacle_quality(
    yaml_path: pathlib.Path,
    obstacle_x_m: float,
    obstacle_y_m: float,
    radius_m: float,
    min_occupied: int,
) -> tuple[bool, str]:
    loaded = load_pgm_map(yaml_path)
    if loaded is None:
        return False, "map_unavailable_for_obstacle_check"
    meta, width, height, data, _image = loaded
    resolution = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if not resolution > 0.0:
        return False, f"bad_resolution={resolution}"

    # map_saver stores PGM rows top-to-bottom while OccupancyGrid coordinates
    # are bottom-left origin.  Check both row conventions and accept the best.
    center_x = int(round((obstacle_x_m - float(origin[0])) / resolution))
    center_y_grid = int(round((obstacle_y_m - float(origin[1])) / resolution))
    center_y_pgm = int(round((height - 1) - center_y_grid))
    radius_px = max(1, int(round(radius_m / resolution)))

    counts: dict[str, int] = {}
    for label, center_y in {"grid_y": center_y_grid, "pgm_flipped_y": center_y_pgm}.items():
        count = 0
        for py in range(max(0, center_y - radius_px), min(height, center_y + radius_px + 1)):
            row = py * width
            for px in range(max(0, center_x - radius_px), min(width, center_x + radius_px + 1)):
                if data[row + px] < 80:
                    count += 1
        counts[label] = count
    best_label, best_count = max(counts.items(), key=lambda item: item[1])
    ok = best_count >= min_occupied
    return ok, (
        f"obstacle_x={obstacle_x_m:.2f} obstacle_y={obstacle_y_m:.2f} radius_m={radius_m:.2f} "
        f"min_occupied={min_occupied} best={best_label}:{best_count} counts={counts}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map-yaml",
        "--map",
        dest="map_yaml",
        default="~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml",
    )
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--skip-graph", action="store_true", help="Only check saved map files; use after launch shutdown.")
    parser.add_argument("--expect-obstacle", action="store_true")
    parser.add_argument("--obstacle-x", type=float, default=1.35)
    parser.add_argument("--obstacle-y", type=float, default=0.65)
    parser.add_argument("--obstacle-radius-m", type=float, default=0.8)
    parser.add_argument("--min-obstacle-occupied", type=int, default=10)
    args = parser.parse_args()

    failures: list[str] = []
    checks: list[str] = []

    if not args.skip_graph:
        cmd = topic_info("/cmd_vel")
        checks.append(f"/cmd_vel publishers={cmd.publisher_count} nodes={cmd.publishers}")
        if cmd.publisher_count != 1 or not any("safety_cmd_mux_node" in node for node in cmd.publishers):
            failures.append("final_cmd_vel_not_safety_mux")

        mode = topic_info("/waver/mode")
        checks.append(f"/waver/mode publishers={mode.publisher_count} nodes={mode.publishers}")
        if mode.publisher_count != 1 or not any("mission_patrol_manager_node" in node for node in mode.publishers):
            failures.append("mode_not_single_mission_manager")

        for topic in ["/scan", "/scan_slam", "/scan_safety", "/map"]:
            info = topic_info(topic)
            checks.append(f"{topic} publishers={info.publisher_count} nodes={info.publishers}")
            if topic != "/scan" and info.publisher_count != 1:
                failures.append(f"{topic}_publisher_count_{info.publisher_count}")

        state = topic_once("/waver/mapping_backend_state", timeout=5.0)
        checks.append("/waver/mapping_backend_state=" + state.strip().replace("\n", " ")[:300])
        if args.strict and "READY" not in state:
            failures.append("mapping_backend_not_ready")

    map_yaml = pathlib.Path(args.map_yaml).expanduser()
    ok, detail = map_quality(map_yaml)
    checks.append(f"map_quality={detail}")
    if not ok:
        failures.append("map_quality_failed")
    if args.expect_obstacle:
        obstacle_ok, obstacle_detail = obstacle_quality(
            map_yaml,
            args.obstacle_x,
            args.obstacle_y,
            args.obstacle_radius_m,
            args.min_obstacle_occupied,
        )
        checks.append(f"obstacle_quality={obstacle_detail}")
        if not obstacle_ok:
            failures.append("obstacle_not_mapped")

    print("REMOTE_UI_SLAM_MAPPING_CHECK")
    for line in checks:
        print(line)
    if failures:
        print("RESULT=FAIL " + ",".join(failures))
        return 1
    print("RESULT=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
