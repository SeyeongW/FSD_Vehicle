#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WAVER_ROS2_WS5_ROOT:-$HOME/ros2_ws5/FSD_Vehicle}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
TIMEOUT_SEC="${TIMEOUT_SEC:-220}"
RUN_ID="${RUN_ID:-ui_slam_bird_$(date +%Y%m%d_%H%M%S)}"
REPORT_ROOT="${REPORT_ROOT:-$WORKSPACE_ROOT/reports/ui_slam_bird_detection}"
REPORT_DIR="$REPORT_ROOT/$RUN_ID"
LATEST_REPORT="$REPORT_ROOT/latest.json"

cd "$WORKSPACE_ROOT"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID
export GAZEBO_MASTER_URI
if [ -n "${WAVER_RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION="$WAVER_RMW_IMPLEMENTATION"
else
  unset RMW_IMPLEMENTATION || true
fi

mkdir -p "$REPORT_DIR" maps log/ui_slam

cleanup() {
  set +e
  [ -n "${BIRD_PUB_PID:-}" ] && kill "$BIRD_PUB_PID" >/dev/null 2>&1 || true
  [ -n "${MONITOR_PID:-}" ] && kill "$MONITOR_PID" >/dev/null 2>&1 || true
  wait "${BIRD_PUB_PID:-}" >/dev/null 2>&1 || true
  wait "${MONITOR_PID:-}" >/dev/null 2>&1 || true
  bash scripts/waver_ui_slam_cleanup.sh >/dev/null 2>&1 || true
}

bash scripts/waver_ui_slam_cleanup.sh >/dev/null 2>&1 || true
trap cleanup EXIT

python3 - "$REPORT_DIR/deterministic_bird_ui_status_publisher.json" <<'PY' &
import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class DeterministicBirdUiStatusPublisher(Node):
    def __init__(self, report_path: str) -> None:
        super().__init__("deterministic_bird_ui_status_publisher")
        self.report_path = report_path
        self.target_class_pub = self.create_publisher(String, "/waver/target_class", 10)
        self.target_confidence_pub = self.create_publisher(Float32, "/waver/target_confidence", 10)
        self.bird_confirmed_pub = self.create_publisher(Bool, "/waver/bird_confirmed", 10)
        self.detector_state_pub = self.create_publisher(String, "/waver/bird_detector_state", 10)
        self.fusion_state_pub = self.create_publisher(String, "/waver/bird_fusion_state", 10)
        self.lidar_state_pub = self.create_publisher(String, "/waver/lidar_target_state", 10)
        self.camera_alignment_pub = self.create_publisher(String, "/waver/camera_alignment_state", 10)
        self.camera_centered_pub = self.create_publisher(Bool, "/waver/camera_target_centered", 10)
        self.sound_state_pub = self.create_publisher(String, "/waver/sound_alert_state", 10)
        self.sound_done_pub = self.create_publisher(Bool, "/waver/sound_task_done", 10)
        self.count = 0
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self.target_class_pub.publish(String(data="bird"))
        self.target_confidence_pub.publish(Float32(data=0.86))
        self.bird_confirmed_pub.publish(Bool(data=True))
        self.detector_state_pub.publish(String(data=f"SIM_ONLY_DETECTOR_READY t={now:.2f}"))
        self.fusion_state_pub.publish(String(data=f"SIM_ONLY_FUSION_VALID t={now:.2f}"))
        self.lidar_state_pub.publish(String(data=f"SIM_ONLY_TARGET_LOCK display_only t={now:.2f}"))
        self.camera_alignment_pub.publish(String(data=f"SIM_ONLY_CAMERA_CENTERED display_only t={now:.2f}"))
        self.camera_centered_pub.publish(Bool(data=True))
        self.sound_state_pub.publish(String(data="SOUND_BLOCKED_MAPPING_MODE SIM_ONLY_DISPLAY_ONLY"))
        self.sound_done_pub.publish(Bool(data=False))
        self.count += 1
        if self.count % 10 == 0:
            with open(self.report_path, "w", encoding="utf-8") as stream:
                json.dump({"node": self.get_name(), "sim_only": True, "messages": self.count}, stream, indent=2)


def main() -> None:
    rclpy.init()
    node = DeterministicBirdUiStatusPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
BIRD_PUB_PID=$!

python3 - "$REPORT_DIR/monitor.json" <<'PY' &
import json
import re
import subprocess
import sys
import time
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def topic_info(topic: str) -> tuple[int, list[str]]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "-v", topic],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
    except Exception:
        return 0, []
    text = (result.stdout or "") + (result.stderr or "")
    match = re.search(r"Publisher count:\s*(\d+)", text)
    count = int(match.group(1)) if match else 0
    nodes = [
        item.group(1).strip()
        for item in re.finditer(
            r"Node name:\s*([^\n]+)(?:(?!\nNode name:).)*?Endpoint type:\s*PUBLISHER",
            text,
            flags=re.DOTALL,
        )
    ]
    return count, nodes


class CombinedMonitor(Node):
    def __init__(self, output_path: str) -> None:
        super().__init__("ui_slam_bird_detection_monitor")
        self.output_path = output_path
        self.started = time.monotonic()
        self.last: dict[str, float] = {}
        self.values: dict[str, Any] = {}
        self.slam_live_seen = False
        self.create_subscription(String, "/waver/current_map_source", self.str_cb("current_map_source"), 10)
        self.create_subscription(OccupancyGrid, "/map", self.msg_cb("map"), 10)
        self.create_subscription(Path, "/waver/mapping_path", self.msg_cb("mapping_path"), 10)
        self.create_subscription(String, "/waver/target_class", self.str_cb("target_class"), 10)
        self.create_subscription(Float32, "/waver/target_confidence", self.float_cb("target_confidence"), 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", self.bool_cb("bird_confirmed"), 10)
        self.create_subscription(String, "/waver/bird_detector_state", self.str_cb("bird_detector_state"), 10)
        self.create_subscription(String, "/waver/bird_fusion_state", self.str_cb("bird_fusion_state"), 10)
        self.create_subscription(String, "/waver/lidar_target_state", self.str_cb("lidar_target_state"), 10)
        self.create_subscription(String, "/waver/camera_alignment_state", self.str_cb("camera_alignment_state"), 10)
        self.create_subscription(Bool, "/waver/camera_target_centered", self.bool_cb("camera_target_centered"), 10)
        self.create_subscription(String, "/waver/sound_alert_state", self.str_cb("sound_alert_state"), 10)
        self.create_subscription(Bool, "/waver/sound_task_done", self.bool_cb("sound_task_done"), 10)
        self.create_subscription(String, "/waver/mission_state", self.str_cb("mission_state"), 10)
        self.create_subscription(String, "/waver/patrol_state", self.str_cb("patrol_state"), 10)
        self.create_subscription(String, "/waver/object_mission_goal_state", self.str_cb("object_goal_state"), 10)
        self.create_timer(1.0, self.write_report)

    def mark(self, name: str, value: Any) -> None:
        self.last[name] = time.monotonic()
        self.values[name] = value
        if name == "current_map_source" and "SLAM_LIVE_MAP" in str(value).upper():
            self.slam_live_seen = True

    def msg_cb(self, name: str):
        def cb(_msg):
            self.mark(name, True)

        return cb

    def str_cb(self, name: str):
        def cb(msg: String):
            self.mark(name, msg.data)

        return cb

    def bool_cb(self, name: str):
        def cb(msg: Bool):
            self.mark(name, bool(msg.data))

        return cb

    def float_cb(self, name: str):
        def cb(msg: Float32):
            self.mark(name, float(msg.data))

        return cb

    def fresh(self, name: str, max_age: float = 5.0) -> bool:
        stamp = self.last.get(name, 0.0)
        return stamp > 0.0 and time.monotonic() - stamp <= max_age

    def ever(self, name: str) -> bool:
        return name in self.last

    def write_report(self) -> None:
        map_count, map_nodes = topic_info("/map")
        cmd_count, cmd_nodes = topic_info("/cmd_vel")
        mode_count, mode_nodes = topic_info("/waver/mode")
        mission = str(self.values.get("mission_state", ""))
        patrol = str(self.values.get("patrol_state", ""))
        object_goal = str(self.values.get("object_goal_state", ""))
        sound = str(self.values.get("sound_alert_state", ""))
        current_map_source = str(self.values.get("current_map_source", ""))
        bird_topic_names = [
            "target_class",
            "target_confidence",
            "bird_confirmed",
            "bird_detector_state",
            "bird_fusion_state",
            "lidar_target_state",
            "camera_alignment_state",
            "camera_target_centered",
            "sound_alert_state",
            "sound_task_done",
        ]
        report = {
            "schema": "ui_slam_bird_detection_monitor_v1",
            "sim_only": True,
            "elapsed_sec": round(time.monotonic() - self.started, 3),
            "current_map_source": current_map_source,
            "current_map_source_slam_live": self.slam_live_seen,
            "map_publisher_count": map_count,
            "map_publisher_nodes": map_nodes,
            "cmd_vel_publisher_count": cmd_count,
            "cmd_vel_publisher_nodes": cmd_nodes,
            "mode_publisher_count": mode_count,
            "mode_publisher_nodes": mode_nodes,
            "bird_topics_visible": all(self.ever(name) for name in bird_topic_names),
            "bird_topics_fresh": all(self.fresh(name) for name in bird_topic_names),
            "bird_detector_state_fresh": self.fresh("bird_detector_state"),
            "bird_fusion_state_fresh": self.fresh("bird_fusion_state"),
            "mapping_path_visible": self.ever("mapping_path"),
            "map_visible": self.ever("map"),
            "no_patrol_conflict": "PATROL_NAVIGATING" not in mission.upper(),
            "no_patrol_emergency_stop": "EMERGENCY_STOP" not in patrol.upper(),
            "no_target_approach_without_arm": "APPROACH" not in object_goal.upper(),
            "no_sound_without_arm": "ACTIVE" not in sound.upper() and "REQUEST" not in sound.upper(),
            "values": self.values,
            "last_age_sec": {
                name: round(time.monotonic() - stamp, 3)
                for name, stamp in self.last.items()
            },
        }
        with open(self.output_path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)


def main() -> None:
    rclpy.init()
    node = CombinedMonitor(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
MONITOR_PID=$!

set +e
timeout --foreground "${TIMEOUT_SEC}" \
  ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py \
    use_sim_time:=true \
    use_gui:="${WAVER_USE_GUI:-false}" \
    mapping_backend:="${WAVER_MAPPING_BACKEND:-scan_mapper}" \
    start_rviz:="${WAVER_START_RVIZ:-false}" \
    spawn_static_obstacle:="${WAVER_SPAWN_TEST_OBSTACLE:-true}" \
    static_obstacle_x:="${WAVER_TEST_OBSTACLE_X:-2.20}" \
    static_obstacle_y:="${WAVER_TEST_OBSTACLE_Y:-1.60}" \
    static_obstacle_z:="${WAVER_TEST_OBSTACLE_Z:-0.0}" \
    start_remote_panel:=true \
    remote_panel_demo_script:="${WAVER_REMOTE_PANEL_DEMO:-mapping_workflow_smoke}" \
    demo_close_on_finish:=true \
    save_dir:="$WORKSPACE_ROOT/maps" \
  2>&1 | tee "$REPORT_DIR/launch.log"
launch_status=${PIPESTATUS[0]}
set -e
if [ "$launch_status" -ne 0 ] && [ "$launch_status" -ne 124 ]; then
  echo "UI_SLAM_BIRD_DETECTION_SMOKE=FAIL launch_status=$launch_status"
  exit "$launch_status"
fi

sleep 2
kill "$MONITOR_PID" >/dev/null 2>&1 || true
wait "$MONITOR_PID" >/dev/null 2>&1 || true

set +e
python3 scripts/check_remote_ui_slam_mapping_result.py \
  --map-yaml "$WORKSPACE_ROOT/maps/waver_latest_map.yaml" \
  --skip-graph \
  --expect-obstacle \
  --obstacle-x "${WAVER_TEST_OBSTACLE_X:-2.20}" \
  --obstacle-y "${WAVER_TEST_OBSTACLE_Y:-1.60}" \
  --obstacle-radius-m "${WAVER_TEST_OBSTACLE_CHECK_RADIUS_M:-0.8}" \
  --min-obstacle-occupied "${WAVER_TEST_OBSTACLE_MIN_OCCUPIED:-10}" \
  >"$REPORT_DIR/map_quality.txt" 2>&1
map_quality_status=$?
set -e

python3 - "$REPORT_DIR/monitor.json" "$REPORT_DIR/map_quality.txt" "$REPORT_DIR/summary.json" "$LATEST_REPORT" <<'PY'
import json
import pathlib
import shutil
import sys

monitor_path = pathlib.Path(sys.argv[1])
map_quality_path = pathlib.Path(sys.argv[2])
summary_path = pathlib.Path(sys.argv[3])
latest_path = pathlib.Path(sys.argv[4])

monitor = json.loads(monitor_path.read_text(encoding="utf-8")) if monitor_path.exists() else {}
map_quality_text = map_quality_path.read_text(encoding="utf-8") if map_quality_path.exists() else ""
map_quality_pass = "RESULT=PASS" in map_quality_text
summary = dict(monitor)
summary["map_quality_pass"] = map_quality_pass
summary["map_quality_text"] = map_quality_text[-2000:]
required_true = [
    "current_map_source_slam_live",
    "bird_topics_visible",
    "bird_topics_fresh",
    "bird_detector_state_fresh",
    "bird_fusion_state_fresh",
    "mapping_path_visible",
    "no_patrol_conflict",
    "no_patrol_emergency_stop",
    "no_target_approach_without_arm",
    "no_sound_without_arm",
    "map_quality_pass",
]
summary["result"] = "PASS" if all(bool(summary.get(name, False)) for name in required_true) and summary.get("map_publisher_count") == 1 and summary.get("cmd_vel_publisher_count") == 1 and summary.get("mode_publisher_count") == 1 else "FAIL"
summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
latest_path.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(summary_path, latest_path)
PY

python3 scripts/check_ui_slam_bird_detection_result.py --report "$LATEST_REPORT"
echo "UI_SLAM_BIRD_DETECTION_SMOKE=PASS"
