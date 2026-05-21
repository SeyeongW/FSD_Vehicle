from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
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

    한계:
      - 장애물 회피 경로 생성은 실제 Nav2가 담당한다. 이 노드는 단순 P-controller라
        회피/전역 재계획을 검증하지 않고, safety_cmd_mux의 stop/timeout 경로만 검증한다.
    """

    def __init__(self) -> None:
        super().__init__("simple_nav2_cmd_sim_node")
        self.declare_parameter("active_goal_topic", "/waver/active_nav_goal")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/waver/cmd_vel_nav2")
        self.declare_parameter("arrived_topic", "/waver/sim_nav_goal_arrived")
        self.declare_parameter("state_topic", "/waver/sim_nav2_state")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("goal_tolerance_m", 0.18)
        self.declare_parameter("yaw_tolerance_rad", 0.35)
        self.declare_parameter("max_linear_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("linear_kp", 0.55)
        self.declare_parameter("angular_kp", 1.4)
        self.declare_parameter("heading_slowdown_rad", 0.55)
        self.declare_parameter("odom_timeout_sec", 0.7)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("timer_hz", 20.0)

        self.goal: PoseStamped | None = None
        self.odom: Odometry | None = None
        self.goal_time = 0.0
        self.last_odom_time = 0.0
        self.arrived_latched = False
        self.mode = "AUTO"
        self.estop = False
        self.external_stop = False

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.arrived_pub = self.create_publisher(Bool, str(self.get_parameter("arrived_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("active_goal_topic").value), self.goal_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 10)
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
        self.arrived_pub.publish(Bool(data=False))
        self.state_pub.publish(
            String(
                data=(
                    f"GOAL_RECEIVED frame={msg.header.frame_id} "
                    f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}"
                )
            )
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.last_odom_time = self._now()

    def tick(self) -> None:
        now = self._now()
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
        dx = float(goal.position.x) - float(robot.position.x)
        dy = float(goal.position.y) - float(robot.position.y)
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
                self.publish_stop(f"ARRIVED distance={distance:.3f} yaw_error={yaw_error:.3f}")
                return
            cmd = Twist()
            cmd.angular.z = clamp(
                float(self.get_parameter("angular_kp").value) * yaw_error,
                -float(self.get_parameter("max_angular_speed").value),
                float(self.get_parameter("max_angular_speed").value),
            )
            self.cmd_pub.publish(cmd)
            self.state_pub.publish(String(data=f"ALIGN_YAW distance={distance:.3f} yaw_error={yaw_error:.3f}"))
            return

        max_linear = float(self.get_parameter("max_linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)
        cmd = Twist()
        cmd.angular.z = clamp(float(self.get_parameter("angular_kp").value) * heading_error, -max_angular, max_angular)
        if abs(heading_error) < float(self.get_parameter("heading_slowdown_rad").value):
            cmd.linear.x = clamp(float(self.get_parameter("linear_kp").value) * distance, 0.0, max_linear)
        else:
            cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(
            String(
                data=(
                    f"TRACKING distance={distance:.3f} heading_error={heading_error:.3f} "
                    f"linear={cmd.linear.x:.3f} angular={cmd.angular.z:.3f}"
                )
            )
        )

    def publish_stop(self, state: str) -> None:
        self.cmd_pub.publish(Twist())
        self.state_pub.publish(String(data=state))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleNav2CmdSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.arrived_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
