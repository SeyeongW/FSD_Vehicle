#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


WEIGHTS = {
    "CRITICAL": 1000,
    "HIGH": 300,
    "MEDIUM": 50,
    "LOW": 10,
    "INFO": 0,
}


@dataclass
class Issue:
    severity: str
    category: str
    message: str
    path: str = ""
    recommendation: str = ""


def read_text(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except FileNotFoundError:
        return ""


def rel(root: Path, path: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def git_branch(root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "branch", "--show-current"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return ""


def source_files(root: Path, patterns: Iterable[str]) -> list[Path]:
    out: list[Path] = []
    ignored = {
        "/build/",
        "/install/",
        "/log/",
        "/__pycache__/",
        "/experiment_results/",
        "/field_results/",
        "/reports/quality_gate/",
        "/reports/agent_iterations/",
    }
    for pattern in patterns:
        for path in root.glob(pattern):
            marker = "/" + rel(root, path)
            if any(token in marker for token in ignored):
                continue
            if path.is_file():
                out.append(path)
    return sorted(set(out))


class ContractCheck:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.issues: list[Issue] = []
        self.summary: dict[str, object] = {
            "workspace_root": str(self.root),
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "branch": git_branch(self.root),
        }

    def add(self, severity: str, category: str, message: str, path: Path | str = "", recommendation: str = "") -> None:
        self.issues.append(Issue(severity, category, message, str(path), recommendation))

    def require_file(self, path: str, category: str, severity: str = "HIGH") -> Path:
        p = self.root / path
        if not p.exists():
            self.add(severity, category, f"Missing required file: {path}", path)
        return p

    def check_workspace(self) -> None:
        if self.summary["branch"] != "jo":
            self.add("CRITICAL", "workspace", f"Current git branch is {self.summary['branch']!r}, expected 'jo'.")
        if not (self.root / "src/waver_patrol/package.xml").exists():
            self.add("CRITICAL", "workspace", "src/waver_patrol/package.xml was not found.")
        for artifact in ("build", "install", "log", "experiment_results"):
            if (self.root / artifact).exists():
                self.add("INFO", "workspace", f"Local generated artifact exists and must not be edited as source: {artifact}", artifact)

    def check_required_files(self) -> None:
        required = {
            "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py": "real_launch",
            "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py": "mission_launch",
            "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml": "real_config",
            "src/waver_patrol/config/nav2_params_waver_real.yaml": "nav2_real_config",
            "src/waver_patrol/config/ekf_waver_real.yaml": "ekf_real_config",
            "src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py": "cmd_vel_contract",
            "src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py": "serial_contract",
            "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py": "mission_contract",
            "src/waver_patrol/waver_patrol/mission/target_departure_monitor_node.py": "mission_contract",
            "src/waver_patrol/waver_patrol/control/auto_behavior_mux_node.py": "cmd_vel_contract",
            "src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py": "ui_contract",
        }
        for path, category in required.items():
            self.require_file(path, category)

    def check_release_hygiene(self) -> None:
        required_files = {
            ".gitignore": "release_hygiene",
            ".dockerignore": "release_hygiene",
            "pytest.ini": "test_hygiene",
            "scripts/make_source_archive.py": "release_hygiene",
            "scripts/run_no_ros_unit_tests.sh": "test_hygiene",
            "src/waver_patrol/docs/operator_modes.md": "operator_docs",
            "src/waver_patrol/docs/real_bird_detector_contract.md": "detector_contract",
            "src/waver_patrol/docs/evidence_levels.md": "evidence_contract",
            "src/waver_patrol/docs/PACKAGE_METADATA_AUDIT.md": "metadata_audit",
            "src/waver_patrol/docs/future_package_split.md": "architecture_note",
        }
        for path, category in required_files.items():
            self.require_file(path, category, severity="HIGH")

        expected_excludes = [
            "build/",
            "install/",
            "log/",
            ".colcon/",
            "__pycache__/",
            "*.pyc",
            ".pytest_cache/",
            "experiment_results/",
            "experiments_result/",
            ".env",
        ]
        for path in (".gitignore", ".dockerignore"):
            text = read_text(self.root / path)
            for token in expected_excludes:
                if token not in text:
                    self.add("HIGH", "release_hygiene", f"{path} does not exclude {token}.", path)
        archive = read_text(self.root / "scripts/make_source_archive.py")
        for token in (".git", ".env", "build", "install", "log", "__pycache__", "experiment_results"):
            if token not in archive:
                self.add("HIGH", "release_hygiene", f"Archive script does not visibly exclude {token}.", "scripts/make_source_archive.py")

    def check_cmd_vel_contract(self) -> None:
        safety = self.root / "src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py"
        safety_text = read_text(safety)
        if "cmd_vel_out_topic" not in safety_text or '"/cmd_vel"' not in safety_text:
            self.add("CRITICAL", "cmd_vel", "safety_cmd_mux_node does not clearly own final /cmd_vel.", rel(self.root, safety))

        ui = self.root / "src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py"
        ui_text = read_text(ui)
        if 'declare_parameter("publish_direct_cmd_vel", False)' not in ui_text:
            self.add("CRITICAL", "cmd_vel", "Remote UI lacks default publish_direct_cmd_vel=false.", rel(self.root, ui))
        if "self.profile == \"real\" and self.publish_direct_cmd_vel" not in ui_text:
            self.add("CRITICAL", "cmd_vel", "Remote UI does not clearly force direct /cmd_vel off in real profile.", rel(self.root, ui))
        if 'manual_cmd_vel_topic", "/waver/manual_cmd_vel"' not in ui_text:
            self.add("HIGH", "cmd_vel", "Remote UI manual command topic is not clearly /waver/manual_cmd_vel.", rel(self.root, ui))

        direct_publishers: list[str] = []
        for path in source_files(self.root, ["src/**/*.py"]):
            text = read_text(path)
            if path == safety:
                continue
            if re.search(r"create_publisher\s*\(\s*Twist\s*,\s*['\"]/?cmd_vel['\"]", text):
                direct_publishers.append(rel(self.root, path))
            if re.search(r"create_publisher\s*\(\s*Twist\s*,\s*self\.final_cmd_vel_topic", text) and "publish_direct_cmd_vel" not in text:
                direct_publishers.append(rel(self.root, path))
        for path in sorted(set(direct_publishers)):
            self.add("CRITICAL", "cmd_vel", "Possible direct /cmd_vel publisher outside safety mux.", path)

        mission_launch = self.root / "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py"
        mission_text = read_text(mission_launch)
        if "remap_nav2_cmd_vel" not in mission_text or "nav2_controller_cmd_topic" not in mission_text:
            self.add("HIGH", "cmd_vel", "Mission launch does not clearly remap Nav2 controller output away from /cmd_vel.", rel(self.root, mission_launch))
        if "/waver/cmd_vel_target_track" not in mission_text and "/waver/cmd_vel_target_track" not in read_text(self.root / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"):
            self.add("MEDIUM", "cmd_vel", "Target tracking command topic is not visible in mission launch/config.")
        if "auto_behavior_mux_node" not in mission_text:
            self.add("CRITICAL", "cmd_vel", "Mission launch does not include auto_behavior_mux_node for target tracking commands.", rel(self.root, mission_launch))
        if "cmd_vel_auto_topic" not in mission_text or "/waver/cmd_vel_auto" not in mission_text:
            self.add("CRITICAL", "cmd_vel", "Mission launch does not clearly route auto behavior output to /waver/cmd_vel_auto.", rel(self.root, mission_launch))
        if '"cmd_vel_auto_topic": ""' in mission_text:
            self.add("HIGH", "cmd_vel", "Mission launch leaves safety cmd_vel_auto_topic empty while target tracking may be enabled.", rel(self.root, mission_launch))

    def check_serial_contract(self) -> None:
        real = self.root / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py"
        text = read_text(real)
        required_tokens = [
            "enable_waver_base_driver",
            "start_serial_bridge",
            "start_base_feedback",
            "requires serial_port",
            "forbids split serial nodes",
            "split command and feedback serial owners are forbidden",
            "command_protocol",
            '"lr"',
        ]
        for token in required_tokens:
            if token not in text:
                self.add("CRITICAL", "serial", f"Real launch is missing serial single-owner guard token: {token}", rel(self.root, real))
        if "/dev/ttyUSB0" in text:
            self.add("MEDIUM", "serial", "Real launch mentions /dev/ttyUSB0; prefer /dev/serial/by-id for field use.", rel(self.root, real))
        if "enable_legacy_ugv_base_odometry_node" not in text:
            self.add("CRITICAL", "serial", "Real launch does not pass a legacy base odometry disable arg.", rel(self.root, real))

    def check_fake_node_blocking(self) -> None:
        real = self.root / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py"
        text = read_text(real)
        for token in ("enable_test_publishers", "enable_deep_learning_stub", "mock_for_sim_only"):
            if token not in text:
                self.add("HIGH", "real_profile", f"Real launch does not explicitly guard {token}.", rel(self.root, real))
        forbidden = ["simple_sim_odom", "gazebo_live_mapping_node", "battery_test_publisher_node", "fake_camera_classification"]
        for token in forbidden:
            if token in text:
                self.add("CRITICAL", "real_profile", f"Gazebo/test-only token appears in real launch: {token}", rel(self.root, real))

    def check_odom_tf_contract(self) -> None:
        real = self.root / "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py"
        real_text = read_text(real)
        ekf = self.root / "src/waver_patrol/config/ekf_waver_real.yaml"
        ekf_text = read_text(ekf)
        if "publish_tf: true" not in ekf_text:
            self.add("HIGH", "odom_tf", "EKF real config does not publish odom->base_link TF.", rel(self.root, ekf))
        if '"publish_tf"' not in real_text or "odom_source" not in real_text:
            self.add("CRITICAL", "odom_tf", "Real launch does not explicitly disable base-driver TF when odom_source=ekf.", rel(self.root, real))
        if "/odom_raw" not in real_text:
            self.add("HIGH", "odom_tf", "Real launch does not clearly route base-driver Odometry to /odom_raw for EKF mode.", rel(self.root, real))
        if "publish_legacy_float32_odom_raw" not in real_text:
            self.add("HIGH", "odom_tf", "Real launch does not explicitly disable legacy Float32 wheel odom in EKF mode.", rel(self.root, real))
        base = self.root / "src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py"
        base_text = read_text(base)
        if "voltage_scale" not in base_text:
            self.add("HIGH", "voltage", "Base driver lacks voltage_scale parameter for field calibration.", rel(self.root, base))
        bringup = self.root / "src/ugv_main/ugv_bringup/launch/bringup_lidar.launch.py"
        bringup_text = read_text(bringup)
        if "enable_legacy_ugv_base_odometry_node" not in bringup_text or "condition=IfCondition" not in bringup_text:
            self.add("CRITICAL", "odom_tf", "bringup_lidar.launch.py base_node lacks a launch condition guard.", rel(self.root, bringup))
        if "odom0: /odom_raw" not in ekf_text or "imu0: /imu/data_raw" not in ekf_text:
            self.add("CRITICAL", "odom_tf", "EKF real config must consume /odom_raw and /imu/data_raw.", rel(self.root, ekf))

    def check_mission_behavior(self) -> None:
        mission = self.root / "src/waver_patrol/waver_patrol/mission/mission_patrol_manager_node.py"
        text = read_text(mission)
        expectations = {
            "interrupted_departure_pose": "departure pose snapshot storage",
            "snapshot_departure_pose": "departure pose snapshot function",
            "RETURN_TO_DEPARTURE_POSE": "return-to-departure state",
            "target_departed": "target departure subscription/logic",
            "RESUME_PATROL": "patrol resume state",
        }
        for token, desc in expectations.items():
            if token not in text:
                self.add("HIGH", "mission", f"Mission manager missing {desc}.", rel(self.root, mission))
        target_departure = self.root / "src/waver_patrol/waver_patrol/mission/target_departure_monitor_node.py"
        if not target_departure.exists():
            self.add("HIGH", "mission", "target_departure_monitor_node.py is missing.", rel(self.root, target_departure))
        else:
            departure_text = read_text(target_departure)
            if "normalize_mission_state" not in departure_text or "split()[0]" not in departure_text:
                self.add("CRITICAL", "mission", "target_departure_monitor_node must normalize suffixed /waver/mission_state strings.", rel(self.root, target_departure))
        mission_launch = self.root / "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py"
        mission_launch_text = read_text(mission_launch)
        if "target_departure_monitor_node" not in mission_launch_text:
            self.add("CRITICAL", "mission", "Real mission launch path does not include target_departure_monitor_node.", rel(self.root, mission_launch))
        real_config = self.root / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"
        real_config_text = read_text(real_config)
        if "wait_bird_clear_after_sound: true" not in real_config_text:
            self.add("CRITICAL", "mission", "Real config must wait for bird/target departure after sound.", rel(self.root, real_config))
        if 'resume_policy: "RETURN_TO_DEPARTURE_POSE"' not in real_config_text:
            self.add("HIGH", "mission", "Real config resume_policy should be RETURN_TO_DEPARTURE_POSE.", rel(self.root, real_config))

    def check_sound_gating(self) -> None:
        real_config = self.root / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"
        text = read_text(real_config)
        must_have = [
            "require_bird_confirmed_for_sound: true",
            "require_camera_classification_before_sound: true",
            "enable_sound_output: false",
            "legal_safety_ack_required: true",
            "safety_ack: false",
        ]
        for token in must_have:
            if token not in text:
                self.add("CRITICAL", "sound", f"Real config missing conservative sound gate: {token}", rel(self.root, real_config))
        if "deterrence_classes: [\"bird\"]" not in text:
            self.add("HIGH", "sound", "Real config should restrict deterrence_classes to bird.", rel(self.root, real_config))

    def check_detector_contract(self) -> None:
        detector = self.root / "src/waver_patrol/waver_patrol/perception/bird_detector_node.py"
        detector_text = read_text(detector)
        for token in (
            "detector_required_for_real",
            "detector_model_state",
            "MODEL_MISSING",
            "bird_confirmed=false",
            "mock_for_sim_only backend is forbidden",
        ):
            if token not in detector_text:
                self.add("CRITICAL", "detector", f"Bird detector missing conservative token: {token}", rel(self.root, detector))
        pure_logic = self.root / "src/waver_patrol/waver_patrol/perception/bird_classification.py"
        logic_text = read_text(pure_logic)
        for token in ("ClassificationDecision", "normalize_class_name", "classify_candidate", "detector_model_state"):
            if token not in logic_text:
                self.add("HIGH", "detector", f"Pure bird classification helper missing {token}.", rel(self.root, pure_logic))
        real_config = self.root / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"
        config_text = read_text(real_config)
        for token in (
            "bird_backend",
            "bird_model_path",
            "detector_required_for_real: true",
            "max_detector_latency_sec",
            "accepted_bird_classes",
            "non_bird_classes",
            "require_3d_fusion_valid: true",
            "require_dynamic_valid_for_inspection: true",
            "require_bird_confirmed_for_sound: true",
        ):
            if token not in config_text:
                self.add("HIGH", "detector", f"Real config missing detector/fusion gate token: {token}", rel(self.root, real_config))

    def check_tracking_chain(self) -> None:
        config_path = self.root / "src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"
        config = read_text(config_path)
        mission = read_text(self.root / "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py")
        if "/waver/cmd_vel_target_track" not in config:
            self.add("HIGH", "tracking", "Target tracking fallback command topic is not configured.", rel(self.root, config_path))
        required = [
            "auto_behavior_mux_node",
            "/waver/cmd_vel_target_track",
            "/waver/cmd_vel_auto",
            "cmd_vel_patrol_topic",
            "cmd_vel_auto_topic",
        ]
        combined = config + "\n" + mission
        for token in required:
            if token not in combined:
                self.add("CRITICAL", "tracking", f"Target tracking auto behavior chain missing token: {token}", rel(self.root, config_path))

    def check_remote_ui_security(self) -> None:
        sensitive = ["1234" + "1234"]
        for path in source_files(self.root, ["scripts/*.sh", "src/**/*.py"]):
            text = read_text(path)
            if any(token in text for token in sensitive):
                self.add("CRITICAL", "remote_ui", "Hardcoded field SSH password remains in source.", rel(self.root, path))
        ui = self.root / "src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py"
        ui_text = read_text(ui)
        if not re.search(r"remote_bridge_password[\"']\s*,\s*\n\s*[\"']{2}", ui_text):
            self.add("HIGH", "remote_ui", "Remote bridge password default is not clearly empty.", rel(self.root, ui))
        if "remote_bridge_key_filename" not in ui_text:
            self.add("MEDIUM", "remote_ui", "Remote UI lacks remote_bridge_key_filename parameter for key-based SSH.", rel(self.root, ui))
        if "WAVER_ALLOW_PASSWORD_SSH" not in ui_text:
            self.add("HIGH", "remote_ui", "Remote UI password SSH is not guarded by WAVER_ALLOW_PASSWORD_SSH.", rel(self.root, ui))

    def check_rosbag_readiness(self) -> None:
        recorders = [
            self.root / "src/waver_patrol/scripts/record_waver_experiment_bag.sh",
            self.root / "src/waver_patrol/scripts/record_waver_lidar_bird_mission_bag.sh",
        ]
        if not any(p.exists() for p in recorders):
            self.add("MEDIUM", "rosbag", "No field rosbag record script found.")
        replay_doc = self.root / "reports/rosbag_replay/README.md"
        if not replay_doc.exists():
            self.add("MEDIUM", "rosbag", "rosbag replay report/template is missing.", rel(self.root, replay_doc))

    def run(self) -> dict[str, object]:
        self.check_workspace()
        self.check_required_files()
        self.check_cmd_vel_contract()
        self.check_serial_contract()
        self.check_fake_node_blocking()
        self.check_odom_tf_contract()
        self.check_mission_behavior()
        self.check_sound_gating()
        self.check_detector_contract()
        self.check_tracking_chain()
        self.check_release_hygiene()
        self.check_remote_ui_security()
        self.check_rosbag_readiness()
        counts = {sev: 0 for sev in WEIGHTS}
        for issue in self.issues:
            counts[issue.severity] = counts.get(issue.severity, 0) + 1
        score = sum(counts[sev] * WEIGHTS[sev] for sev in WEIGHTS)
        result = "PASS" if counts["CRITICAL"] == 0 and counts["HIGH"] == 0 else "FAIL"
        self.summary.update(
            {
                "result": result,
                "safety_regression": result != "PASS",
                "score": score,
                "counts": counts,
                "issue_count": len(self.issues),
                "issues": [asdict(issue) for issue in self.issues],
                "next_recommended_fixes": [
                    issue.message for issue in self.issues if issue.severity in {"CRITICAL", "HIGH"}
                ][:10],
            }
        )
        return self.summary


def write_reports(report: dict[str, object], report_dir: Path) -> None:
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "contract_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    lines = [
        f"WAVER_CONTRACT_CHECK={report['result']}",
        f"WORKSPACE_ROOT={report['workspace_root']}",
        f"BRANCH={report['branch']}",
        f"SCORE={report['score']}",
        f"COUNTS={report['counts']}",
        "",
        "Issues:",
    ]
    for issue in report["issues"]:
        lines.append(
            f"- [{issue['severity']}] {issue['category']}: {issue['message']}"
            + (f" ({issue['path']})" if issue.get("path") else "")
        )
        if issue.get("recommendation"):
            lines.append(f"  recommendation: {issue['recommendation']}")
    (report_dir / "contract_report.txt").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Hardware-free Waver real/sim contract checker.")
    parser.add_argument("--root", default=".", help="Workspace root")
    parser.add_argument("--report-dir", default="", help="Directory for contract_report.txt/json")
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    checker = ContractCheck(root)
    report = checker.run()
    if args.report_dir:
        write_reports(report, Path(args.report_dir).expanduser())
    else:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"WAVER_CONTRACT_CHECK={report['result']} SCORE={report['score']} COUNTS={report['counts']}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
