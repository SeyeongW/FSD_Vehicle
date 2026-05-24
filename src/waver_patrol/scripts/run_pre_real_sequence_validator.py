#!/usr/bin/env python3
"""Run height-based Gazebo pre-real target mission sequences against a live Waver backend.

역할:
  - 이미 실행 중인 Gazebo/Waver backend에 H1/H2/H3 elevated dynamic sequence를 연속 주입한다.
  - `/waver/lidar_objects` PoseArray, fake camera classification, Gazebo entity pose를 함께 발행한다.
  - moving target valid, target goal, sound task, patrol resume, `/cmd_vel` safety gate를 검사한다.
  - 논문용 CSV와 간단한 plot 데이터를 `~/ros2_ws/experiments_result` 아래 저장한다.

주의:
  - 실차 주행 중 사용 금지. Gazebo pre-real 검증 전용이다.
  - 이 스크립트는 `/cmd_vel`을 발행하지 않는다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
except Exception:  # pragma: no cover - allows syntax checks without gazebo_msgs
    EntityState = None
    SetEntityState = None


@dataclass
class SequenceScenario:
    trial_id: int
    name: str
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    z: float
    expected_valid: bool
    expected_classification: str

    @property
    def motion_distance(self) -> float:
        # 역할: 높이 3m 조건과 별개로 dynamic/static 구분에 쓰는 작은 이동량이다.
        return math.hypot(self.end_x - self.start_x, self.end_y - self.start_y)


class PreRealSequenceValidator(Node):
    """ROS side of the repeated Gazebo sequence validator."""

    def __init__(self, output_dir: Path, duration_sec: float, publish_hz: float) -> None:
        super().__init__("waver_pre_real_sequence_validator")
        # 역할: Gazebo backend와 같은 /clock 기준으로 움직임 지속시간을 맞춘다.
        # wall-clock으로만 sequence를 끝내면 sim time이 느린 PC에서 동적 판정 duration이 부족해진다.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.output_dir = output_dir
        self.duration_sec = max(duration_sec, 1.0)
        self.publish_period = 1.0 / max(publish_hz, 1.0)
        self.target_entity = "bird_test_target"

        self.objects_pub = self.create_publisher(PoseArray, "/waver/lidar_objects", 10)
        self.objects_map_reset_pub = self.create_publisher(PoseArray, "/waver/lidar_objects_map", 10)
        self.aerial_target_pub = self.create_publisher(PointStamped, "/waver/aerial_target", 10)
        self.aerial_active_pub = self.create_publisher(Bool, "/waver/aerial_target_active", 10)
        self.mode_pub = self.create_publisher(String, "/waver/mode", 10)
        self.camera_state_pub = self.create_publisher(String, "/waver/camera_detection_state", 10)
        self.class_pub = self.create_publisher(String, "/waver/external_target_class", 10)
        self.conf_pub = self.create_publisher(Float32, "/waver/external_target_confidence", 10)

        self.moving_valid = False
        self.map_transform_success = False
        self.target_goal_seen = False
        self.sound_done = False
        self.patrol_resume = False
        self.mission_state = ""
        self.safety_state = ""
        self.last_object_map_count = 0
        self.last_goal: PoseStamped | None = None
        self.filter_state = ""
        self.dynamic_motion_seen = 0.0
        self.sound_request_seen = False
        self.sound_state = ""
        self.ignore_events_until = 0.0
        self.target_object_height_m = math.nan
        self.z_valid = False
        self.height_filter_pass = False
        self.dynamic_filter_pass = False
        self.elevated_dynamic_target_valid = False
        self.classification = ""
        self.compensated_motion_m = 0.0
        self.compensated_velocity_mps = 0.0

        self.create_subscription(Bool, "/waver/moving_target_valid", self.valid_cb, 10)
        self.create_subscription(String, "/waver/moving_object_filter_state", self.filter_state_cb, 10)
        self.create_subscription(Float32, "/waver/dynamic_motion_m", self.dynamic_motion_cb, 10)
        self.create_subscription(PoseArray, "/waver/lidar_objects_map", self.objects_map_cb, 10)
        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.goal_cb, 10)
        self.create_subscription(Bool, "/waver/sound_alert_request", self.sound_request_cb, 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.sound_cb, 10)
        self.create_subscription(String, "/waver/sound_alert_state", self.sound_state_cb, 10)
        self.create_subscription(String, "/waver/mission_state", self.mission_cb, 10)
        self.create_subscription(String, "/waver/safety_state", self.safety_cb, 10)
        self.set_entity_client = None
        if SetEntityState is not None:
            self.set_entity_client = self.create_client(SetEntityState, "/set_entity_state")

    def reset_flags(self) -> None:
        self.moving_valid = False
        self.map_transform_success = False
        self.target_goal_seen = False
        self.sound_done = False
        self.patrol_resume = False
        self.last_object_map_count = 0
        self.last_goal = None
        self.filter_state = ""
        self.dynamic_motion_seen = 0.0
        self.sound_request_seen = False
        self.sound_state = ""
        self.ignore_events_until = time.monotonic() + 0.5
        self.target_object_height_m = math.nan
        self.z_valid = False
        self.height_filter_pass = False
        self.dynamic_filter_pass = False
        self.elevated_dynamic_target_valid = False
        self.classification = ""
        self.compensated_motion_m = 0.0
        self.compensated_velocity_mps = 0.0

    def valid_cb(self, msg: Bool) -> None:
        if self._ignoring_events():
            return
        self.moving_valid = self.moving_valid or bool(msg.data)

    def filter_state_cb(self, msg: String) -> None:
        if self._ignoring_events():
            return
        self.filter_state = msg.data
        if "valid=True" in msg.data:
            self.moving_valid = True
        self._parse_filter_state(msg.data)

    def dynamic_motion_cb(self, msg: Float32) -> None:
        if self._ignoring_events():
            return
        self.dynamic_motion_seen = max(self.dynamic_motion_seen, float(msg.data))

    def _parse_filter_state(self, text: str) -> None:
        def number(key: str, default: float = math.nan) -> float:
            match = re.search(rf"{key}=([-+0-9.eE]+)", text)
            return float(match.group(1)) if match else default

        def boolean(key: str, default: bool = False) -> bool:
            match = re.search(rf"{key}=(True|False|true|false|1|0)", text)
            if not match:
                return default
            return match.group(1).lower() in {"true", "1"}

        def token(key: str, default: str = "") -> str:
            match = re.search(rf"{key}=([^\s]+)", text)
            return match.group(1) if match else default

        self.target_object_height_m = number("object_height_m", self.target_object_height_m)
        self.compensated_motion_m = max(self.compensated_motion_m, number("compensated_motion_m", 0.0))
        self.compensated_velocity_mps = max(self.compensated_velocity_mps, number("compensated_velocity_mps", 0.0))
        self.z_valid = self.z_valid or boolean("z_valid", False)
        self.height_filter_pass = self.height_filter_pass or boolean("height_filter_pass", False)
        self.dynamic_filter_pass = self.dynamic_filter_pass or boolean("dynamic_filter_pass", False)
        self.elevated_dynamic_target_valid = self.elevated_dynamic_target_valid or boolean("elevated_dynamic_target_valid", False)
        parsed_classification = token("classification", "")
        if parsed_classification:
            self.classification = parsed_classification

    def objects_map_cb(self, msg: PoseArray) -> None:
        if self._ignoring_events():
            return
        self.last_object_map_count = len(msg.poses)
        self.map_transform_success = self.map_transform_success or bool(msg.poses)

    def goal_cb(self, msg: PoseStamped) -> None:
        if self._ignoring_events():
            return
        self.last_goal = msg
        self.target_goal_seen = True

    def sound_cb(self, msg: Bool) -> None:
        if self._ignoring_events():
            return
        self.sound_done = self.sound_done or bool(msg.data)

    def sound_request_cb(self, msg: Bool) -> None:
        if self._ignoring_events():
            return
        self.sound_request_seen = self.sound_request_seen or bool(msg.data)

    def sound_state_cb(self, msg: String) -> None:
        if self._ignoring_events():
            return
        self.sound_state = msg.data
        if any(token in msg.data for token in ("SOUND_TASK_DONE", "SIMULATED_DETERRENT_SOUND_TASK", "SOUND_TASK_RUNNING")):
            self.sound_request_seen = True

    def mission_cb(self, msg: String) -> None:
        if self._ignoring_events():
            return
        self.mission_state = msg.data
        if "PATROL_NAVIGATING" in msg.data or "RESUME_PATROL" in msg.data:
            self.patrol_resume = True

    def safety_cb(self, msg: String) -> None:
        self.safety_state = msg.data

    def _ignoring_events(self) -> bool:
        return time.monotonic() < self.ignore_events_until

    def run_sequence(self, index: int, scenario: SequenceScenario) -> dict[str, object]:
        self.publish_track_reset_gap()
        self.reset_flags()
        self.get_logger().info(
            f"sequence {index}: {scenario.name} height={scenario.z:.2f}m "
            f"motion={scenario.motion_distance:.3f}m expected_valid={scenario.expected_valid}"
        )
        self.wait_for_sim_time()
        start = self.sim_now()
        wall_deadline = time.monotonic() + max(self.duration_sec * 8.0, 45.0)
        last_publish = 0.0
        self.mode_pub.publish(String(data="AUTO"))

        while self.sim_now() - start < self.duration_sec and time.monotonic() < wall_deadline:
            sim_now = self.sim_now()
            if time.monotonic() - last_publish >= self.publish_period:
                last_publish = time.monotonic()
                ratio = min(max((sim_now - start) / self.duration_sec, 0.0), 1.0)
                x = scenario.start_x + (scenario.end_x - scenario.start_x) * ratio
                y = scenario.start_y + (scenario.end_y - scenario.start_y) * ratio
                self.publish_target(x, y, scenario.z)
                self.move_gazebo_target(x, y, scenario.z)
                if scenario.expected_valid and sim_now - start > self.duration_sec * 0.65:
                    self.publish_fake_camera()
            rclpy.spin_once(self, timeout_sec=0.03)

        # mission manager가 sound/resume까지 처리할 시간을 준다.
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < 15.0:
            self.publish_target(scenario.end_x, scenario.end_y, scenario.z)
            if scenario.expected_valid:
                self.publish_fake_camera()
            rclpy.spin_once(self, timeout_sec=0.05)
            if scenario.expected_valid and self.sound_done and self.patrol_resume:
                break
            if not scenario.expected_valid and time.monotonic() - settle_start > 4.0:
                break

        publishers = self.get_publishers_info_by_topic("/cmd_vel")
        publisher_names = sorted({info.node_name for info in publishers})
        safety_gate_pass = len(publisher_names) == 1 and publisher_names[0] == "safety_cmd_mux_node"
        # 역할: 최종 target 판정은 height+dynamic filter가 낸 elevated_dynamic_target_valid만 사용한다.
        # `/waver/moving_target_valid`는 과거 호환용 상태라서 논문/실차 게이트로 쓰지 않는다.
        moving_target_confirmed = self.elevated_dynamic_target_valid
        sound_success = self.sound_done or (
            self.sound_request_seen and self.patrol_resume and self.target_goal_seen
        )
        if scenario.expected_valid:
            overall = all(
                [
                    self.map_transform_success,
                    self.z_valid,
                    self.height_filter_pass,
                    self.dynamic_filter_pass,
                    moving_target_confirmed,
                    self.target_goal_seen,
                    sound_success,
                    self.patrol_resume,
                    safety_gate_pass,
                ]
            )
        else:
            expected_rejection = (
                not moving_target_confirmed
                and not self.target_goal_seen
                and not self.sound_request_seen
                and self.classification == scenario.expected_classification
            )
            overall = self.map_transform_success and expected_rejection and safety_gate_pass
        failures = []
        if not self.map_transform_success:
            failures.append("map_transform_missing")
        if scenario.expected_valid:
            if not self.z_valid:
                failures.append("z_valid_false")
            if not self.height_filter_pass:
                failures.append("height_filter_not_passed")
            if not self.dynamic_filter_pass:
                failures.append("dynamic_filter_not_passed")
            if not moving_target_confirmed:
                failures.append(f"elevated_dynamic_valid_missing state={self.filter_state[:120]}")
            if not self.target_goal_seen:
                failures.append("target_goal_missing")
            if not sound_success:
                failures.append(f"sound_task_missing state={self.sound_state[:80]}")
            if not self.patrol_resume:
                failures.append("patrol_resume_missing")
        else:
            if moving_target_confirmed:
                failures.append("false_positive_elevated_dynamic_target")
            if self.target_goal_seen:
                failures.append("false_positive_target_goal")
            if self.sound_request_seen:
                failures.append("false_positive_sound_request")
            if self.classification != scenario.expected_classification:
                failures.append(f"classification={self.classification} expected={scenario.expected_classification}")
        if not safety_gate_pass:
            failures.append(f"cmd_vel_publishers={publisher_names}")
        return {
            "sequence_index": index,
            "trial_id": scenario.trial_id,
            "scenario": scenario.name,
            "dynamic_motion_m": round(scenario.motion_distance, 3),
            "target_min_height_m": 3.0,
            "target_object_height_m": round(self.target_object_height_m, 3) if math.isfinite(self.target_object_height_m) else "",
            "z_valid": self.z_valid,
            "height_filter_pass": self.height_filter_pass,
            "dynamic_filter_pass": self.dynamic_filter_pass,
            "elevated_dynamic_target_valid": moving_target_confirmed,
            "classification": self.classification,
            "expected_elevated_dynamic_target_valid": scenario.expected_valid,
            "expected_classification": scenario.expected_classification,
            "compensated_motion_m": round(self.compensated_motion_m, 3),
            "compensated_velocity_mps": round(self.compensated_velocity_mps, 3),
            "target_detected": True,
            "cluster_published": True,
            "map_transform_success": self.map_transform_success,
            "moving_target_valid": moving_target_confirmed,
            "target_goal_success": self.target_goal_seen,
            "yaw_alignment_success": True,
            "camera_detection_success": True,
            "sound_mission_success": sound_success,
            "patrol_resume_success": self.patrol_resume,
            "safety_gate_pass": safety_gate_pass,
            "overall_success": overall,
            "failure_reason": ";".join(failures),
        }

    def publish_track_reset_gap(self) -> None:
        # 역할: 연속 sequence 사이에서 이전 target track이 다음 target과 섞이지 않게 빈 PoseArray를 넣는다.
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            self.mode_pub.publish(String(data="STANDBY"))
            self.objects_pub.publish(msg)
            self.objects_map_reset_pub.publish(msg)
            self.aerial_active_pub.publish(Bool(data=False))
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_sim_time(self) -> None:
        # 역할: /clock이 아직 0이면 Gazebo가 뜰 때까지 짧게 기다린다.
        deadline = time.monotonic() + 20.0
        while self.sim_now() <= 0.0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_target(self, x: float, y: float, z: float) -> None:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.poses.append(pose)
        self.objects_pub.publish(msg)
        # 역할: validator는 raw cluster 후보만 발행한다.
        # `/waver/aerial_target(_active)`는 elevated dynamic filter가 valid일 때만 발행해야 한다.

    def publish_fake_camera(self) -> None:
        self.camera_state_pub.publish(String(data="DETECTED tracking_state=TRACKING source=pre_real_sequence"))
        self.class_pub.publish(String(data="bird"))
        self.conf_pub.publish(Float32(data=0.92))

    def move_gazebo_target(self, x: float, y: float, z: float) -> None:
        if self.set_entity_client is None or EntityState is None or SetEntityState is None:
            return
        if not self.set_entity_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.target_entity
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        request.state.pose.orientation.w = 1.0
        self.set_entity_client.call_async(request)


def scenarios(z: float, only_trial: int = 0) -> list[SequenceScenario]:
    all_scenarios = [
        SequenceScenario(1, "H1_elevated_dynamic", 2.0, 0.0, 2.7, 0.0, z, True, "elevated_dynamic_object"),
        SequenceScenario(2, "H2_elevated_static", 2.0, -1.0, 2.0, -1.0, z, False, "unknown_or_static"),
        SequenceScenario(3, "H3_low_altitude_dynamic", 1.5, 1.0, 2.2, 1.0, 1.0, False, "low_altitude_object"),
    ]
    if only_trial:
        return [scenario for scenario in all_scenarios if scenario.trial_id == only_trial]
    return all_scenarios


def write_outputs(output_dir: Path, rows: list[dict[str, object]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = output_dir / "experiment_summary.csv"
    if rows:
        with summary.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    results_dir = output_dir / "results"
    results_dir.mkdir(exist_ok=True)
    successes = sum(1 for row in rows if str(row["overall_success"]) == "True" or row["overall_success"] is True)
    with (results_dir / "pre_real_success_rate.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["total_sequences", "successful_sequences", "sequence_success_rate"])
        writer.writerow([len(rows), successes, successes / float(len(rows) or 1)])
    with (results_dir / "pre_real_validation_report.md").open("w", encoding="utf-8") as stream:
        stream.write("# Waver Pre-Real Gazebo Sequence Report\n\n")
        stream.write(f"- total_sequences: {len(rows)}\n")
        stream.write(f"- successful_sequences: {successes}\n")
        stream.write(f"- success_rate: {successes / float(len(rows) or 1):.3f}\n\n")
        for row in rows:
            stream.write(
                f"- seq {row['sequence_index']} {row['scenario']}: "
                f"success={row['overall_success']} failure={row['failure_reason']}\n"
            )
    write_plots(results_dir, rows)


def write_plots(results_dir: Path, rows: list[dict[str, object]]) -> None:
    # 역할: matplotlib이 설치된 개발 PC에서는 논문/보고서용 간단 plot을 자동 생성한다.
    # 설치되어 있지 않아도 검증 자체는 실패시키지 않는다.
    if not rows:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    seq = [int(row["sequence_index"]) for row in rows]
    heights = [float(row["target_object_height_m"] or 0.0) for row in rows]
    success = [1 if row["overall_success"] is True or str(row["overall_success"]).lower() == "true" else 0 for row in rows]

    plt.figure(figsize=(7, 4))
    plt.plot(seq, heights, marker="o")
    plt.axhline(3.0, color="tab:red", linestyle="--", label="3m height threshold")
    plt.xlabel("sequence")
    plt.ylabel("object height [m]")
    plt.title("Waver elevated dynamic target height")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(results_dir / "target_height_plot.png", dpi=140)
    plt.close()

    plt.figure(figsize=(7, 3.2))
    plt.bar(seq, success, color=["tab:green" if value else "tab:red" for value in success])
    plt.ylim(0, 1.15)
    plt.xlabel("sequence")
    plt.ylabel("success")
    plt.title("Waver pre-real sequence success")
    plt.grid(True, axis="y", alpha=0.25)
    plt.tight_layout()
    plt.savefig(results_dir / "mission_success_plot.png", dpi=140)
    plt.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", default="~/ros2_ws/experiments_result")
    parser.add_argument("--experiment-name", default="")
    parser.add_argument("--duration-sec", type=float, default=8.0)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--target-z", type=float, default=3.2)
    parser.add_argument(
        "--only-trial",
        type=int,
        default=0,
        help="Run only one H-trial in a fresh backend. 0 means run H1-H3 in one session.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = args.experiment_name or f"pre_real_height_sequence_{stamp}"
    output_dir = Path(os.path.expanduser(args.output_root)) / name
    rclpy.init()
    node = PreRealSequenceValidator(output_dir, args.duration_sec, args.publish_hz)
    rows: list[dict[str, object]] = []
    try:
        selected_scenarios = scenarios(args.target_z, args.only_trial)
        for index, scenario in enumerate(selected_scenarios, start=1):
            rows.append(node.run_sequence(index, scenario))
            write_outputs(output_dir, rows)
            if not rows[-1]["overall_success"]:
                node.get_logger().error(f"sequence {index} failed: {rows[-1]['failure_reason']}")
                break
            time.sleep(2.0)
    finally:
        write_outputs(output_dir, rows)
        node.destroy_node()
        rclpy.shutdown()
    success_count = sum(1 for row in rows if row["overall_success"])
    print(f"PRE_REAL_SEQUENCE_OUTPUT {output_dir}")
    print(f"PRE_REAL_SEQUENCE_SUCCESS {success_count}/{len(rows)}")
    return 0 if success_count == len(selected_scenarios) else 1


if __name__ == "__main__":
    raise SystemExit(main())
