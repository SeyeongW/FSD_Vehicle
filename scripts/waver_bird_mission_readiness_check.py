#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODES = (
    "source",
    "sensor-live",
    "lidar-tracking",
    "detector-live",
    "fusion-live",
    "inspection-dry-run",
    "supervised-deterrence",
    "autonomous-patrol",
)
MODE_TO_LEVEL = {
    "source": "B0_SOURCE_AND_NAV_BASE",
    "sensor-live": "B1_SENSOR_LIVE",
    "lidar-tracking": "B2_LIDAR_OBJECT_TRACKING",
    "detector-live": "B3_CAMERA_BIRD_DETECTOR",
    "fusion-live": "B4_CAMERA_LIDAR_FUSION",
    "inspection-dry-run": "B5_INSPECTION_MISSION_DRY_RUN",
    "supervised-deterrence": "B6_DETERRENCE_HARDWARE_READY",
    "autonomous-patrol": "B7_AUTONOMOUS_BIRD_PATROL_READY",
}
SOURCE_REQUIRED = (
    "docs/bird_mission_readiness_levels.md",
    "docs/final_bird_patrol_architecture.md",
    "README_BIRD_PATROL_FIELD.md",
    "config/real_profiles/bird_patrol_production.yaml",
    "config/sensors/camera_lidar_extrinsic.yaml",
    "config/perception/bird_model_registry.yaml",
    "src/waver_patrol/launch/bird_patrol_production.launch.py",
    "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py",
    "src/waver_patrol/waver_patrol/perception/bird_detector_node.py",
    "src/waver_patrol/waver_patrol/perception/bird_3d_fusion_node.py",
    "src/waver_patrol/waver_patrol/control/camera_gimbal_controller_node.py",
    "src/waver_patrol/waver_patrol/bridges/sound_deterrent_node.py",
    "src/waver_patrol/waver_patrol/mission/target_goal_manager_node.py",
    "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py",
    "scripts/waver_bird_patrol_field_start.sh",
    "scripts/waver_livox_mid360_probe.py",
    "scripts/waver_camera_probe.py",
    "scripts/waver_bird_detector_probe.py",
    "scripts/waver_camera_lidar_calibration_check.py",
    "scripts/waver_bird_mission_supervisor.py",
    "scripts/make_bird_mission_field_release.py",
    "scripts/check_bird_mission_field_release.py",
)
TOPIC_REQUIREMENTS = {
    "sensor-live": ("/livox/lidar", "/scan_safety", "/camera/image_raw", "/camera/camera_info", "/odom", "/tf", "/tf_static", "/waver/safety_state"),
    "lidar-tracking": ("/waver/lidar_objects", "/waver/elevated_dynamic_targets", "/waver/moving_target_valid"),
    "detector-live": ("/waver/bird_detector_state", "/waver/bird_detections_2d", "/waver/target_class", "/waver/camera_bbox_center_error_px"),
    "fusion-live": ("/waver/bird_fusion_state", "/waver/bird_fusion_sync_state", "/waver/bird_target_valid", "/waver/bird_target_pose_map"),
    "inspection-dry-run": ("/waver/mission_state", "/waver/inspection_target_pose_map", "/waver/camera_alignment_state"),
    "supervised-deterrence": ("/waver/sound_alert_state", "/waver/sound_task_active", "/waver/sound_task_done"),
    "autonomous-patrol": ("/waver/bird_mission_supervisor_state", "/waver/target_departed"),
}


def run(cmd: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=timeout, check=False)
        return proc.returncode, proc.stdout + proc.stderr
    except Exception as exc:
        return 124, str(exc)


def ros_topic_exists(topic: str) -> bool:
    rc, out = run(["ros2", "topic", "info", topic], timeout=3.0)
    return rc == 0 and "Unknown topic" not in out and "Could not determine" not in out


def latest_topic_once(topic: str) -> str:
    rc, out = run(["timeout", "3", "ros2", "topic", "echo", "--once", topic], timeout=4.0)
    return out if rc == 0 else ""


def load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text()) if path.exists() else {}


def find_leaf(data: Any, key: str, default: Any = None) -> Any:
    if isinstance(data, dict):
        if key in data:
            return data[key]
        for value in data.values():
            found = find_leaf(value, key, None)
            if found is not None:
                return found
    return default


def load_report(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {"status": "REPORT_PARSE_FAIL", "path": str(path)}


def add_check(report: dict[str, Any], ok: bool, name: str, detail: str = "", *, degraded: bool = False, blocked: bool = False) -> None:
    item = {"name": name, "detail": detail}
    if ok and not degraded and not blocked:
        report["active_capabilities"].append(item)
    elif ok and degraded:
        report["degraded_capabilities"].append(item)
    elif ok and blocked:
        report["blocked_capabilities"].append(item)
    else:
        report["failed_checks"].append(item)


def add_blocked(report: dict[str, Any], name: str, detail: str = "") -> None:
    report["blocked_capabilities"].append({"name": name, "detail": detail})


def source_checks(report: dict[str, Any], profile_path: Path, profile: dict[str, Any], mode: str = "source") -> None:
    missing = [rel for rel in SOURCE_REQUIRED if not (ROOT / rel).exists()]
    add_check(report, not missing, "source_required_files", ", ".join(missing) if missing else "all present")
    add_check(report, profile_path.exists(), "profile_exists", str(profile_path))
    if profile:
        profile_name = str(profile.get("profile", profile_path.stem))
        bird_stack_expected = mode in {
            "source",
            "detector-live",
            "fusion-live",
            "inspection-dry-run",
            "supervised-deterrence",
            "autonomous-patrol",
        }
        if bird_stack_expected:
            add_check(report, find_leaf(profile, "enable_bird_detector") is True, "profile_enables_bird_detector")
            add_check(report, find_leaf(profile, "enable_bird_3d_fusion") is True, "profile_enables_bird_3d_fusion")
            add_check(report, find_leaf(profile, "enable_sound_deterrent") is True, "profile_enables_sound_deterrent")
        add_check(report, find_leaf(profile, "enable_sound_output") is False, "profile_sound_output_default_false")
        add_check(report, str(find_leaf(profile, "scan_topic", "")) == "/scan_safety", "profile_scan_topic_is_scan_safety")
        max_linear = float(find_leaf(profile, "max_linear_speed", 99.0))
        max_angular = float(find_leaf(profile, "max_angular_speed", 99.0))
        if profile_name == "supervised_bird_patrol":
            add_check(report, 0.08 <= max_linear <= 0.12, "profile_supervised_linear_speed_tier")
            add_check(report, 0.25 <= max_angular <= 0.35, "profile_supervised_angular_speed_tier")
        else:
            add_check(report, max_linear <= 0.05, "profile_linear_speed_cap")
            add_check(report, max_angular <= 0.20, "profile_angular_speed_cap")
        add_check(report, find_leaf(profile, "require_camera_lidar_extrinsic") is True, "profile_requires_camera_lidar_extrinsic")
        add_check(report, find_leaf(profile, "require_detector_model") is True, "profile_requires_detector_model")


def check_topics(report: dict[str, Any], mode: str) -> None:
    required: list[str] = []
    for item_mode in MODES[1 : MODES.index(mode) + 1]:
        required.extend(TOPIC_REQUIREMENTS.get(item_mode, ()))
    for topic in required:
        add_check(report, ros_topic_exists(topic), f"topic:{topic}")


def check_probe_reports(report: dict[str, Any], args: argparse.Namespace, profile: dict[str, Any]) -> None:
    used: dict[str, str] = {}
    livox = load_report(Path(args.livox_report).expanduser())
    camera = load_report(Path(args.camera_report).expanduser())
    detector = load_report(Path(args.detector_report).expanduser())
    calibration = load_report(Path(args.calibration_report).expanduser())
    if livox:
        report["lidar_status"] = livox
        used["livox"] = args.livox_report
    if camera:
        report["camera_status"] = camera
        used["camera"] = args.camera_report
    if detector:
        report["detector_status"] = detector
        used["detector"] = args.detector_report
    if calibration:
        report["fusion_status"]["calibration_report"] = calibration
        used["calibration"] = args.calibration_report
    report["probe_reports_used"] = used

    mode = args.mode
    if mode in ("sensor-live", "lidar-tracking", "detector-live", "fusion-live", "inspection-dry-run", "supervised-deterrence", "autonomous-patrol"):
        if not livox:
            add_check(report, False, "livox_report_present", args.livox_report)
        else:
            status = str(livox.get("status", ""))
            add_check(report, status == "LIDAR_READY", "livox_probe_ready", status, degraded=status == "LIDAR_DEGRADED")
            add_check(report, float(livox.get("pointcloud_rate_hz") or 0.0) >= float(find_leaf(profile, "min_pointcloud_rate_hz", 0.0)), "livox_pointcloud_rate")
            add_check(report, float(livox.get("point_count_mean") or 0.0) >= float(find_leaf(profile, "min_points_per_cloud", 0.0)), "livox_point_count")
        if not camera:
            add_check(report, False, "camera_report_present", args.camera_report)
        else:
            add_check(report, camera.get("status") == "CAMERA_READY", "camera_probe_ready", str(camera.get("status")))

    if mode in ("detector-live", "fusion-live", "inspection-dry-run", "supervised-deterrence", "autonomous-patrol"):
        if not detector:
            add_check(report, False, "detector_report_present", args.detector_report)
        else:
            add_check(report, detector.get("status") == "DETECTOR_READY", "detector_probe_ready", str(detector.get("status")))
            add_check(report, detector.get("class_map_ok") is True, "detector_class_map_ok")
            max_latency = float(find_leaf(profile, "max_detector_latency_ms", 300.0))
            add_check(report, float(detector.get("max_latency_ms") or 999999.0) <= max_latency, "detector_latency_cap")

    if mode in ("fusion-live", "inspection-dry-run", "supervised-deterrence", "autonomous-patrol"):
        if not calibration:
            add_check(report, False, "calibration_report_present", args.calibration_report)
        else:
            add_check(report, calibration.get("status") == "CALIBRATION_READY", "camera_lidar_calibration_ready", str(calibration.get("status")))
            add_check(report, calibration.get("calibrated") is True, "camera_lidar_calibrated")


def check_hardware_matrix(report: dict[str, Any], args: argparse.Namespace) -> None:
    matrix_path = Path(args.hardware_matrix).expanduser()
    if not matrix_path.exists():
        add_check(report, args.mode == "source", "hardware_matrix_present", str(matrix_path))
        return
    data = load_yaml(matrix_path)
    items = data.get("items", []) if isinstance(data, dict) else []
    required_failures: list[str] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        required_modes = item.get("required_for_modes", [])
        if isinstance(required_modes, str):
            required_modes = [required_modes]
        if args.mode not in required_modes:
            continue
        status = str(item.get("status", "TODO")).upper()
        if status not in {"PASS", "N/A"}:
            required_failures.append(f"{item.get('item', 'unknown')}={status}")
    if args.mode == "source":
        add_check(report, True, "hardware_matrix_source_mode_allows_todo", str(matrix_path), blocked=bool(required_failures))
    else:
        add_check(report, not required_failures, "hardware_matrix_required_items_pass", ", ".join(required_failures))


def check_runtime_states(report: dict[str, Any], mode: str, profile: dict[str, Any]) -> None:
    if mode in ("detector-live", "fusion-live", "inspection-dry-run", "supervised-deterrence", "autonomous-patrol"):
        state = latest_topic_once("/waver/bird_detector_state")
        add_check(report, "model_state=MODEL_READY" in state, "detector_model_ready", state[-300:])
        add_check(report, "camera_state=CAMERA_OK" in state, "detector_camera_ready", state[-300:])
        add_check(report, "class_map_ok=true" in state, "detector_class_map_ok_live", state[-300:])
    if mode in ("sensor-live", "detector-live"):
        extrinsic = ROOT / str(find_leaf(profile, "camera_lidar_extrinsic_path", "config/sensors/camera_lidar_extrinsic.yaml"))
        data = load_yaml(extrinsic)
        if data.get("calibrated") is not True:
            add_blocked(report, "camera_lidar_calibration_not_verified", str(extrinsic))
    if mode in ("fusion-live", "inspection-dry-run", "supervised-deterrence", "autonomous-patrol"):
        extrinsic = ROOT / str(find_leaf(profile, "camera_lidar_extrinsic_path", "config/sensors/camera_lidar_extrinsic.yaml"))
        data = load_yaml(extrinsic)
        add_check(report, bool(data and data.get("calibrated") is True), "camera_lidar_calibrated", str(extrinsic))
        fusion = latest_topic_once("/waver/bird_fusion_state")
        sync = latest_topic_once("/waver/bird_fusion_sync_state")
        add_check(report, bool(fusion), "fusion_state_live", fusion[-300:])
        add_check(report, bool(sync), "fusion_sync_state_live", sync[-300:])
        add_check(report, "FUSION_INVALID_NO_EXTRINSIC" not in fusion, "fusion_has_extrinsic", fusion[-300:])
        add_check(report, "FUSION_INVALID_CALIBRATION_NOT_VERIFIED" not in fusion, "fusion_calibration_verified", fusion[-300:])
    if mode in ("supervised-deterrence", "autonomous-patrol"):
        env_ok = all(os.environ.get(name) == "1" for name in ("WAVER_ACK_SOUND_HARDWARE", "WAVER_ACK_LOCAL_SOUND_LAW", "WAVER_ACK_OPERATOR_SUPERVISION"))
        add_check(report, env_ok, "sound_ack_env_present")


def check_cmd_chain(report: dict[str, Any], *, require_collision_monitor: bool, require_base_driver: bool) -> None:
    cmd_info = run(["ros2", "topic", "info", "-v", "/cmd_vel"], timeout=4.0)[1]
    safety_info = run(["ros2", "topic", "info", "-v", "/waver/cmd_vel_safety"], timeout=4.0)[1]
    base_state = latest_topic_once("/waver/base_driver_state")
    report["cmd_chain"] = {
        "cmd_vel_info": cmd_info[-4000:],
        "cmd_vel_safety_info": safety_info[-4000:],
        "base_driver_state": base_state[-1000:],
    }
    if require_collision_monitor:
        add_check(report, "collision_monitor" in cmd_info, "final_cmd_vel_publisher_collision_monitor", cmd_info[-1000:])
        add_check(report, "safety_cmd_mux_node" in safety_info, "safety_cmd_vel_publisher_safety_mux", safety_info[-1000:])
    if require_base_driver:
        add_check(report, "waver_base_driver_node" in cmd_info or "waver_base_driver_node" in base_state, "base_driver_connected", base_state[-500:])
        add_check(report, "ODOM_FEEDBACK_OK" in base_state or "odom_ok=True" in base_state, "base_driver_odom_feedback_ok", base_state[-500:])


def classify(report: dict[str, Any], mode: str) -> str:
    if report["failed_checks"]:
        return "FAIL"
    if mode == "autonomous-patrol" and (report["degraded_capabilities"] or report["blocked_capabilities"]):
        report["failed_checks"].append(
            {"name": "autonomous_patrol_requires_full_pass", "detail": "PASS_DEGRADED/BLOCKED is forbidden"}
        )
        return "FAIL"
    if report["degraded_capabilities"] or report["blocked_capabilities"]:
        return "PASS_DEGRADED"
    return "PASS"


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Waver production bird patrol readiness by mode.")
    parser.add_argument("--mode", choices=MODES, default="source")
    parser.add_argument("--level", choices=[f"B{i}" for i in range(8)], help="legacy level alias")
    parser.add_argument("--profile", default=str(ROOT / "config/real_profiles/bird_patrol_production.yaml"))
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--no-hardware", action="store_true")
    parser.add_argument("--use-probe-reports", action="store_true")
    parser.add_argument("--auto-run-probes", action="store_true")
    parser.add_argument("--allow-topic-only-smoke", action="store_true")
    parser.add_argument("--legacy-topic-only-check", action="store_true")
    parser.add_argument("--livox-report", default=str(ROOT / "reports/livox_mid360/latest.json"))
    parser.add_argument("--camera-report", default=str(ROOT / "reports/camera/latest.json"))
    parser.add_argument("--detector-report", default=str(ROOT / "reports/bird_detector/latest.json"))
    parser.add_argument("--calibration-report", default=str(ROOT / "reports/hardware_calibration/camera_lidar_latest.json"))
    parser.add_argument("--require-collision-monitor", action="store_true")
    parser.add_argument("--require-base-driver", action="store_true")
    parser.add_argument("--require-blackbox", action="store_true")
    parser.add_argument("--require-sound-hardware", action="store_true")
    parser.add_argument("--hardware-matrix", default=str(ROOT / "config/hardware_acceptance_matrix.yaml"))
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if args.level:
        args.mode = MODES[int(args.level[1])]

    profile_path = Path(args.profile).expanduser().resolve()
    profile = load_yaml(profile_path)
    report: dict[str, Any] = {
        "mode": args.mode,
        "level": MODE_TO_LEVEL[args.mode],
        "status": "UNKNOWN",
        "active_capabilities": [],
        "degraded_capabilities": [],
        "blocked_capabilities": [],
        "failed_checks": [],
        "warnings": [],
        "node_graph": {},
        "topic_rates": {},
        "tf_status": {},
        "cmd_chain": {},
        "lidar_status": {},
        "camera_status": {},
        "detector_status": {},
        "fusion_status": {},
        "alignment_status": {},
        "sound_status": {},
        "mission_status": {},
        "safety_status": {},
        "probe_reports_used": {},
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }

    source_checks(report, profile_path, profile, args.mode)
    if args.no_hardware and args.mode != "source":
        add_blocked(report, "hardware_access", "--no-hardware only supports source mode")
        report["failed_checks"].append({"name": "hardware_required_for_mode", "detail": args.mode})
    elif args.mode != "source":
        probe_paths = [Path(args.livox_report), Path(args.camera_report), Path(args.detector_report), Path(args.calibration_report)]
        reports_present = any(path.expanduser().exists() for path in probe_paths)
        if args.auto_run_probes:
            add_blocked(report, "auto_run_probes_not_executed_by_static_checker", "run probe scripts explicitly and pass reports")
        if args.use_probe_reports or reports_present or not (args.allow_topic_only_smoke or args.legacy_topic_only_check):
            check_probe_reports(report, args, profile)
        else:
            add_blocked(report, "topic_only_smoke_mode", "development smoke only; not production readiness")
            check_topics(report, args.mode)
            check_runtime_states(report, args.mode, profile)
        require_collision = (
            args.require_collision_monitor
            or args.mode == "autonomous-patrol"
            or bool(find_leaf(profile, "collision_monitor_required_for_autonomous", False) and args.mode == "autonomous-patrol")
        )
        require_base = args.require_base_driver or args.mode in ("inspection-dry-run", "supervised-deterrence", "autonomous-patrol")
        if require_collision or require_base:
            check_cmd_chain(report, require_collision_monitor=require_collision, require_base_driver=require_base)
        if args.require_blackbox or args.mode == "autonomous-patrol":
            add_check(report, ros_topic_exists("/waver/blackbox_state"), "blackbox_active")

    if args.require_sound_hardware and args.mode in ("supervised-deterrence", "autonomous-patrol"):
        env_ok = all(os.environ.get(name) == "1" for name in ("WAVER_ACK_SOUND_HARDWARE", "WAVER_ACK_LOCAL_SOUND_LAW", "WAVER_ACK_OPERATOR_SUPERVISION"))
        add_check(report, env_ok, "sound_hardware_ack_env_present")
    check_hardware_matrix(report, args)

    report["status"] = classify(report, args.mode)
    out = Path(args.output).expanduser().resolve() if args.output else ROOT / "reports/bird_mission_readiness" / f"{int(time.time())}_{args.mode}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"BIRD_MISSION_READINESS={report['status']}")
    print(f"BIRD_MISSION_READINESS_REPORT={out}")
    return 1 if args.strict and report["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
