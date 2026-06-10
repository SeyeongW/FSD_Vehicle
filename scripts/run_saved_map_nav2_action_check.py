#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def run(command: list[str], timeout: float = 8.0) -> str:
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        return str(stdout) + str(stderr) + "\nTIMEOUT\n"
    return (result.stdout or "") + (result.stderr or "")


def topic_publishers(topic: str) -> tuple[int, list[str]]:
    text = run(["ros2", "topic", "info", "-v", topic], timeout=8.0)
    match = re.search(r"Publisher count:\s*(\d+)", text)
    count = int(match.group(1)) if match else 0
    nodes = [
        match.group(1).strip()
        for match in re.finditer(
            r"Node name:\s*([^\n]+)(?:(?!\nNode name:).)*?Endpoint type:\s*PUBLISHER",
            text,
            flags=re.DOTALL,
        )
    ]
    return count, nodes


def lifecycle_active(node_name: str) -> bool:
    text = run(["ros2", "lifecycle", "get", node_name], timeout=10.0)
    return "active" in text.lower()


class SavedMapNav2Check(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("saved_map_nav2_action_check")
        self.args = args
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.mode_pub = self.create_publisher(String, "/waver/mode_cmd", 10)
        self.mission_pub = self.create_publisher(String, "/waver/mission_command", 10)
        self.odom: Odometry | None = None
        self.amcl: PoseWithCovarianceStamped | None = None
        self.cmd_vel: Twist | None = None
        self.nav_cmd: Twist | None = None
        self.safety_state = ""
        self.odom_path: list[tuple[float, float]] = []
        self.max_abs_y = 0.0
        self.min_obstacle_distance = math.inf
        self.max_final_linear = 0.0
        self.max_final_angular = 0.0
        self.max_nav_linear = 0.0
        self.max_nav_angular = 0.0
        self.final_nonzero_count = 0
        self.nav_nonzero_count = 0
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Twist, "/waver/cmd_vel_nav2", self._nav_cmd_cb, 10)
        self.create_subscription(String, "/waver/safety_state", self._safety_cb, 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not self.odom_path or math.hypot(x - self.odom_path[-1][0], y - self.odom_path[-1][1]) >= 0.01:
            self.odom_path.append((x, y))
            if len(self.odom_path) > 5000:
                self.odom_path = self.odom_path[-5000:]
        self.max_abs_y = max(self.max_abs_y, abs(y - float(self.args.initial_y)))
        if self.args.require_obstacle_clearance:
            distance = math.hypot(x - self.args.obstacle_x, y - self.args.obstacle_y)
            self.min_obstacle_distance = min(self.min_obstacle_distance, distance)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.max_final_linear = max(self.max_final_linear, abs(float(msg.linear.x)))
        self.max_final_angular = max(self.max_final_angular, abs(float(msg.angular.z)))
        if abs(float(msg.linear.x)) > 1e-4 or abs(float(msg.angular.z)) > 1e-4:
            self.final_nonzero_count += 1

    def _nav_cmd_cb(self, msg: Twist) -> None:
        self.nav_cmd = msg
        self.max_nav_linear = max(self.max_nav_linear, abs(float(msg.linear.x)))
        self.max_nav_angular = max(self.max_nav_angular, abs(float(msg.angular.z)))
        if abs(float(msg.linear.x)) > 1e-4 or abs(float(msg.angular.z)) > 1e-4:
            self.nav_nonzero_count += 1

    def _safety_cb(self, msg: String) -> None:
        self.safety_state = msg.data

    def spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # Use stamp=0 so AMCL/Nav2 accept the pose on the current simulation
        # time line even when this checker itself is not using /clock.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = self.args.initial_x
        msg.pose.pose.position.y = self.args.initial_y
        qx, qy, qz, qw = yaw_to_quaternion(self.args.initial_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        for _ in range(12):
            self.initial_pose_pub.publish(msg)
            self.mode_pub.publish(String(data="AUTO"))
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_nav2_active(self) -> bool:
        required = [
            "/map_server",
            "/amcl",
            "/controller_server",
            "/planner_server",
            "/behavior_server",
            "/bt_navigator",
        ]
        deadline = time.monotonic() + self.args.nav2_active_timeout
        while time.monotonic() < deadline:
            if all(lifecycle_active(name) for name in required):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def send_goal_and_wait(self) -> tuple[bool, int, float]:
        if not self.client.wait_for_server(timeout_sec=self.args.action_server_timeout):
            return False, -1, math.inf

        start = self.odom
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = self.args.goal_x
        goal.pose.pose.position.y = self.args.goal_y
        qx, qy, qz, qw = yaw_to_quaternion(self.args.goal_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, -2, 0.0

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.args.result_timeout
        while rclpy.ok() and time.monotonic() < deadline and not result_future.done():
            self.mode_pub.publish(String(data="AUTO"))
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False, -3, self._odom_distance_from(start)

        status = int(result_future.result().status)
        return status == GoalStatus.STATUS_SUCCEEDED, status, self._odom_distance_from(start)

    def _odom_distance_from(self, start: Odometry | None) -> float:
        if start is None or self.odom is None:
            return 0.0
        sx = start.pose.pose.position.x
        sy = start.pose.pose.position.y
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        return math.hypot(x - sx, y - sy)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=0.45)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--min-odom-distance", type=float, default=0.15)
    parser.add_argument("--startup-timeout", type=float, default=70.0)
    parser.add_argument("--nav2-active-timeout", type=float, default=80.0)
    parser.add_argument("--action-server-timeout", type=float, default=30.0)
    parser.add_argument("--result-timeout", type=float, default=90.0)
    parser.add_argument("--require-obstacle-clearance", action="store_true")
    parser.add_argument("--obstacle-x", type=float, default=0.65)
    parser.add_argument("--obstacle-y", type=float, default=0.0)
    parser.add_argument("--min-obstacle-distance", type=float, default=0.34)
    parser.add_argument("--min-lateral-deviation", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = SavedMapNav2Check(args)
    try:
        odom_ok = node.spin_until(lambda: node.odom is not None, args.startup_timeout)
        nav2_active_ok = node.wait_nav2_active()
        node.publish_initial_pose()
        amcl_ok = node.spin_until(lambda: node.amcl is not None, 20.0)
        cmd_count, cmd_nodes = topic_publishers("/cmd_vel")
        nav_count, nav_nodes = topic_publishers("/waver/cmd_vel_nav2")
        map_count, map_nodes = topic_publishers("/map")

        goal_ok, status, distance = node.send_goal_and_wait()
        final_cmd_seen = node.cmd_vel is not None
        nav_cmd_seen = node.nav_cmd is not None

        print("SAVED_MAP_NAV2_CHECK")
        print(f"odom_ok={odom_ok}")
        print(f"nav2_active_ok={nav2_active_ok}")
        print(f"amcl_pose_ok={amcl_ok}")
        print(f"cmd_vel_publishers={cmd_count} nodes={cmd_nodes}")
        print(f"nav_cmd_publishers={nav_count} nodes={nav_nodes}")
        print(f"map_publishers={map_count} nodes={map_nodes}")
        print(f"nav2_goal_status={status}")
        print(f"odom_distance_m={distance:.3f}")
        print(f"odom_path_points={len(node.odom_path)}")
        print(f"max_lateral_deviation_m={node.max_abs_y:.3f}")
        if args.require_obstacle_clearance:
            min_obs = node.min_obstacle_distance if math.isfinite(node.min_obstacle_distance) else -1.0
            print(
                "obstacle_clearance="
                f"center=({args.obstacle_x:.3f},{args.obstacle_y:.3f}) "
                f"min_distance_m={min_obs:.3f} required_m={args.min_obstacle_distance:.3f}"
            )
        print(f"final_cmd_seen={final_cmd_seen}")
        print(f"nav_cmd_seen={nav_cmd_seen}")
        print(
            "cmd_stats="
            f"nav_max=({node.max_nav_linear:.3f},{node.max_nav_angular:.3f}) "
            f"final_max=({node.max_final_linear:.3f},{node.max_final_angular:.3f}) "
            f"nav_nonzero={node.nav_nonzero_count} final_nonzero={node.final_nonzero_count}"
        )
        print(f"safety_state={node.safety_state or 'NONE'}")

        failures: list[str] = []
        if not odom_ok:
            failures.append("odom_missing")
        if not nav2_active_ok:
            failures.append("nav2_lifecycle_not_active")
        if not amcl_ok:
            failures.append("amcl_pose_missing")
        if cmd_count != 1 or not any("safety_cmd_mux_node" in n for n in cmd_nodes):
            failures.append("final_cmd_vel_not_safety_mux")
        if map_count != 1 or not any("map_server" in n for n in map_nodes):
            failures.append("map_not_from_map_server")
        if nav_count < 1:
            failures.append("nav_cmd_missing")
        if not final_cmd_seen:
            failures.append("final_cmd_not_seen")
        if not nav_cmd_seen:
            failures.append("nav_cmd_not_seen")
        if node.nav_cmd is not None and node.nav_nonzero_count == 0:
            failures.append("nav_cmd_zero_only")
        if node.cmd_vel is not None and node.final_nonzero_count == 0:
            failures.append("final_cmd_zero_only")
        if not goal_ok:
            failures.append(f"nav2_goal_not_succeeded_status_{status}")
        if distance < args.min_odom_distance:
            failures.append(f"odom_distance_too_small_{distance:.3f}")
        if args.require_obstacle_clearance:
            if not math.isfinite(node.min_obstacle_distance):
                failures.append("obstacle_clearance_not_measured")
            elif node.min_obstacle_distance < args.min_obstacle_distance:
                failures.append(f"obstacle_clearance_too_small_{node.min_obstacle_distance:.3f}")
        if args.min_lateral_deviation > 0.0 and node.max_abs_y < args.min_lateral_deviation:
            failures.append(f"lateral_deviation_too_small_{node.max_abs_y:.3f}")

        if failures:
            print("RESULT=FAIL " + ",".join(failures))
            return 1
        print("RESULT=PASS")
        return 0
    finally:
        try:
            node.mission_pub.publish(String(data="STOP"))
            node.mode_pub.publish(String(data="STANDBY"))
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
