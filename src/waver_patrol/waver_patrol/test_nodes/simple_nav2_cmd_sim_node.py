from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waver_patrol.autonomy_common import clamp, normalize_angle, yaw_from_quaternion


class SimpleNav2CmdSimNode(Node):
    """Gazebo-only stand-in for Nav2 controller output.

    역할:
      - 실차용 노드가 아니다. Nav2 action server가 없는 headless Gazebo 반복시험에서만 쓴다.
      - mission manager가 발행한 `/waver/active_nav_goal`을 `/odom` 기준으로 따라가며
        `/waver/cmd_vel_nav2` 후보 명령을 만든다.
      - 목표 반경에 들어오면 `/waver/sim_nav_goal_arrived=true`를 한 번 발행해
        mission 상태머신이 Nav2 성공 결과를 받은 것처럼 다음 단계로 넘어가게 한다.
      - Gazebo 동적장애물 smoke test에서는 `/waver/dynamic_obstacle_map`을 받아 임시 detour
        waypoint를 생성한다. 이는 실제 Nav2 planner 대체가 아니라 UI/path/safety 연결 검증이다.

    한계:
      - 실차 장애물 회피 경로 생성은 실제 Nav2 planner/controller가 담당한다.
    """

    def __init__(self) -> None:
        super().__init__("simple_nav2_cmd_sim_node")
        self.declare_parameter("active_goal_topic", "/waver/active_nav_goal")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/waver/cmd_vel_nav2")
        self.declare_parameter("arrived_topic", "/waver/sim_nav_goal_arrived")
        self.declare_parameter("state_topic", "/waver/sim_nav2_state")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("dynamic_obstacle_topic", "/waver/dynamic_obstacle_map")
        self.declare_parameter("enable_dynamic_obstacle_avoidance", False)
        self.declare_parameter("avoidance_corridor_radius_m", 0.55)
        self.declare_parameter("avoidance_offset_m", 0.85)
        self.declare_parameter("obstacle_timeout_sec", 0.8)
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("goal_tolerance_m", 0.18)
        self.declare_parameter("yaw_tolerance_rad", 0.35)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("linear_kp", 0.55)
        self.declare_parameter("angular_kp", 1.4)
        self.declare_parameter("heading_slowdown_rad", 0.55)
        self.declare_parameter("rotate_in_place_heading_error_rad", 0.75)
        self.declare_parameter("slowdown_distance_m", 1.0)
        self.declare_parameter("min_linear_speed_near_goal", 0.025)
        self.declare_parameter("max_linear_accel_mps2", 0.35)
        self.declare_parameter("max_angular_accel_radps2", 1.2)
        self.declare_parameter("arrived_latch_sec", 0.15)
        self.declare_parameter("odom_timeout_sec", 0.7)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("timer_hz", 20.0)

        self.goal: PoseStamped | None = None
        self.odom: Odometry | None = None
        self.goal_time = 0.0
        self.last_odom_time = 0.0
        self.obstacle: PointStamped | None = None
        self.goal_meta: dict[str, object] = {}
        self.last_obstacle_time = 0.0
        self.arrived_latched = False
        self.arrived_latch_until = 0.0
        self.mode = "AUTO"
        self.estop = False
        self.external_stop = False
        self.last_tick_time = self._now()
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.arrived_pub = self.create_publisher(Bool, str(self.get_parameter("arrived_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.debug_pub = self.create_publisher(String, "/waver/sim_nav2_debug", 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("active_goal_topic").value), self.goal_callback, 10)
        self.create_subscription(String, "/waver/active_nav_goal_meta", self.goal_meta_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self.obstacle_callback,
            10,
        )
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), lambda m: setattr(self, "mode", m.data.strip().upper()), 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("simple_nav2_cmd_sim_node is Gazebo/test only; never launch it on the real Waver")

    def goal_callback(self, msg: PoseStamped) -> None:
        # 역할: mission manager의 새 goal을 받아 기존 arrival latch를 풀고 추종을 시작한다.
        self.goal = msg
        self.goal_time = self._now()
        self.arrived_latched = False
        self.arrived_latch_until = 0.0
        self.arrived_pub.publish(Bool(data=False))
        self.state_pub.publish(
            String(
                data=(
                    f"GOAL_RECEIVED frame={msg.header.frame_id} "
                    f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}"
                )
            )
        )

    def goal_meta_callback(self, msg: String) -> None:
        try:
            self.goal_meta = json.loads(msg.data)
        except Exception:
            self.goal_meta = {"raw": msg.data}

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = self._now()

    def obstacle_callback(self, msg: PointStamped) -> None:
        self.obstacle = msg
        self.last_obstacle_time = self._now()

    def tick(self) -> None:
        now = self._now()
        dt = max(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), now - self.last_tick_time)
        self.last_tick_time = now
        self.arrived_pub.publish(Bool(data=False))
        if self.estop or self.external_stop or self.mode in {"EMERGENCY", "DISABLED", "STANDBY", "MANUAL"}:
            self.publish_stop(f"BLOCKED mode={self.mode} estop={self.estop} external_stop={self.external_stop}")
            return
        if self.goal is None:
            self.publish_stop("IDLE_NO_GOAL")
            return
        if self.odom is None or now - self.last_odom_time > float(self.get_parameter("odom_timeout_sec").value):
            self.publish_stop("ODOM_TIMEOUT")
            return
        if now - self.goal_time > float(self.get_parameter("goal_timeout_sec").value):
            self.publish_stop("GOAL_TIMEOUT")
            return

        robot = self.odom.pose.pose
        goal = self.goal.pose
        robot_x = float(robot.position.x)
        robot_y = float(robot.position.y)
        goal_x = float(goal.position.x)
        goal_y = float(goal.position.y)
        target_x, target_y, avoiding, avoid_detail = self.avoidance_target(robot_x, robot_y, goal_x, goal_y)
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        yaw = yaw_from_quaternion(
            float(robot.orientation.x),
            float(robot.orientation.y),
            float(robot.orientation.z),
            float(robot.orientation.w),
        )
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - yaw)
        goal_yaw = yaw_from_quaternion(
            float(goal.orientation.x),
            float(goal.orientation.y),
            float(goal.orientation.z),
            float(goal.orientation.w),
        )
        yaw_error = normalize_angle(goal_yaw - yaw)
        if distance <= float(self.get_parameter("goal_tolerance_m").value):
            if abs(yaw_error) <= float(self.get_parameter("yaw_tolerance_rad").value):
                if not self.arrived_latched:
                    self.arrived_latched = True
                    self.arrived_pub.publish(Bool(data=True))
                self.arrived_latch_until = max(self.arrived_latch_until, now + float(self.get_parameter("arrived_latch_sec").value))
                self.publish_stop(f"ARRIVED distance={distance:.3f} yaw_error={yaw_error:.3f}", robot_x, robot_y, yaw, goal_x, goal_y, goal_yaw, distance, heading_error, yaw_error, True)
                return
            cmd = Twist()
            target_angular = clamp(
                float(self.get_parameter("angular_kp").value) * yaw_error,
                -float(self.get_parameter("max_angular_speed").value),
                float(self.get_parameter("max_angular_speed").value),
            )
            cmd.angular.z = self.limit_rate(self.last_cmd_angular, target_angular, float(self.get_parameter("max_angular_accel_radps2").value), dt)
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = cmd.angular.z
            self.cmd_pub.publish(cmd)
            self.state_pub.publish(String(data=f"ALIGN_YAW distance={distance:.3f} yaw_error={yaw_error:.3f}"))
            self.publish_debug("ALIGN_YAW", robot_x, robot_y, yaw, goal_x, goal_y, goal_yaw, distance, heading_error, yaw_error, cmd.linear.x, cmd.angular.z, False, "")
            return

        max_linear = float(self.get_parameter("max_linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)
        cmd = Twist()
        target_angular = clamp(float(self.get_parameter("angular_kp").value) * heading_error, -max_angular, max_angular)
        if abs(heading_error) < float(self.get_parameter("heading_slowdown_rad").value):
            target_linear = clamp(float(self.get_parameter("linear_kp").value) * distance, 0.0, max_linear)
            if distance < float(self.get_parameter("slowdown_distance_m").value):
                target_linear = min(target_linear, max(max_linear * distance / max(float(self.get_parameter("slowdown_distance_m").value), 1e-3), float(self.get_parameter("min_linear_speed_near_goal").value)))
        else:
            target_linear = 0.0
        if abs(heading_error) >= float(self.get_parameter("rotate_in_place_heading_error_rad").value):
            target_linear = 0.0
        cmd.linear.x = self.limit_rate(self.last_cmd_linear, target_linear, float(self.get_parameter("max_linear_accel_mps2").value), dt)
        cmd.angular.z = self.limit_rate(self.last_cmd_angular, target_angular, float(self.get_parameter("max_angular_accel_radps2").value), dt)
        self.last_cmd_linear = cmd.linear.x
        self.last_cmd_angular = cmd.angular.z
        self.cmd_pub.publish(cmd)
        state_name = "AVOIDING" if avoiding else "TRACKING"
        self.state_pub.publish(
            String(
                data=(
                    f"{state_name} distance={distance:.3f} heading_error={heading_error:.3f} "
                    f"linear={cmd.linear.x:.3f} angular={cmd.angular.z:.3f} {avoid_detail}"
                )
            )
        )
        self.publish_debug(state_name, robot_x, robot_y, yaw, goal_x, goal_y, goal_yaw, distance, heading_error, yaw_error, cmd.linear.x, cmd.angular.z, False, avoid_detail)

    def avoidance_target(
        self,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
    ) -> tuple[float, float, bool, str]:
        if not bool(self.get_parameter("enable_dynamic_obstacle_avoidance").value):
            return goal_x, goal_y, False, ""
        if self.obstacle is None or self._now() - self.last_obstacle_time > float(self.get_parameter("obstacle_timeout_sec").value):
            return goal_x, goal_y, False, "obstacle=none"
        ox = float(self.obstacle.point.x)
        oy = float(self.obstacle.point.y)
        vx = goal_x - robot_x
        vy = goal_y - robot_y
        length = math.hypot(vx, vy)
        if length < 1e-3:
            return goal_x, goal_y, False, "goal_near"
        wx = ox - robot_x
        wy = oy - robot_y
        along = (wx * vx + wy * vy) / length
        if along < 0.0 or along > length:
            return goal_x, goal_y, False, f"obstacle_outside_path along={along:.2f}"
        cross = abs(vx * wy - vy * wx) / length
        radius = float(self.get_parameter("avoidance_corridor_radius_m").value)
        if cross > radius:
            return goal_x, goal_y, False, f"obstacle_clear lateral={cross:.2f}"
        nx = -vy / length
        ny = vx / length
        side = -1.0 if (vx * wy - vy * wx) > 0.0 else 1.0
        offset = float(self.get_parameter("avoidance_offset_m").value)
        detour_x = ox + side * nx * offset
        detour_y = oy + side * ny * offset
        return detour_x, detour_y, True, f"obstacle=({ox:.2f},{oy:.2f}) detour=({detour_x:.2f},{detour_y:.2f})"

    def publish_stop(
        self,
        state: str,
        robot_x: float = math.nan,
        robot_y: float = math.nan,
        robot_yaw: float = math.nan,
        goal_x: float = math.nan,
        goal_y: float = math.nan,
        goal_yaw: float = math.nan,
        distance: float = math.nan,
        heading_error: float = math.nan,
        yaw_error: float = math.nan,
        arrived: bool = False,
    ) -> None:
        self.cmd_pub.publish(Twist())
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.state_pub.publish(String(data=state))
        self.publish_debug(state, robot_x, robot_y, robot_yaw, goal_x, goal_y, goal_yaw, distance, heading_error, yaw_error, 0.0, 0.0, arrived, state)

    def publish_debug(
        self,
        state: str,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        distance: float,
        heading_error: float,
        yaw_error: float,
        cmd_linear: float,
        cmd_angular: float,
        arrived: bool,
        blocked_reason: str,
    ) -> None:
        payload = {
            "time_sec": self._now(),
            "state": state,
            "goal_role": self.goal_meta.get("goal_role", "UNKNOWN"),
            "mission_state": self.goal_meta.get("mission_state", ""),
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_yaw": goal_yaw,
            "distance_to_goal_m": distance,
            "heading_error_rad": heading_error,
            "yaw_error_rad": yaw_error,
            "cmd_linear_x": cmd_linear,
            "cmd_angular_z": cmd_angular,
            "arrived": bool(arrived),
            "blocked_reason": blocked_reason,
        }
        self.debug_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    @staticmethod
    def limit_rate(previous: float, target: float, max_rate: float, dt: float) -> float:
        step = max(0.0, max_rate) * max(0.0, dt)
        return clamp(target, previous - step, previous + step)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleNav2CmdSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok() and "context is not valid" not in str(exc):
            raise
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
                node.arrived_pub.publish(Bool(data=False))
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
