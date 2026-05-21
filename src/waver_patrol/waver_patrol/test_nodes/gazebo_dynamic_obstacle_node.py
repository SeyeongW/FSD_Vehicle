from __future__ import annotations

import math

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class GazeboDynamicObstacleNode(Node):
    """Move a Gazebo box across Waver's path for safety-mux smoke tests.

    역할:
      - 실차에서 절대 사용하지 않는 Gazebo 전용 테스트 노드다.
      - `/cmd_vel`이나 Waver mission topic은 발행하지 않는다.
      - Gazebo entity pose만 갱신해서 /scan 기반 slow/hard stop을 검증한다.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_dynamic_obstacle_node")
        self.declare_parameter("entity_name", "dynamic_test_box")
        self.declare_parameter("x_m", 0.75)
        self.declare_parameter("y_amplitude_m", 1.15)
        self.declare_parameter("z_m", 0.25)
        self.declare_parameter("period_sec", 5.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("timer_hz", 15.0)
        self.declare_parameter("set_entity_state_services", ["/set_entity_state", "/gazebo/set_entity_state"])
        self.declare_parameter("state_topic", "/waver/gazebo_dynamic_obstacle_state")

        self._gazebo_state_clients = [
            self.create_client(SetEntityState, service_name)
            for service_name in list(self.get_parameter("set_entity_state_services").value)
        ]
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.start_time = self._now()
        self.warned_waiting = False
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("GazeboDynamicObstacleNode is simulation-only; do not launch on the real Waver")

    def tick(self) -> None:
        client = self._ready_client()
        if client is None:
            if not self.warned_waiting:
                self.get_logger().warn("Waiting for Gazebo set_entity_state service")
                self.warned_waiting = True
            return
        elapsed = self._now() - self.start_time
        delay = float(self.get_parameter("start_delay_sec").value)
        phase_time = max(elapsed - delay, 0.0)
        period = max(float(self.get_parameter("period_sec").value), 0.5)
        y = float(self.get_parameter("y_amplitude_m").value) * math.sin(2.0 * math.pi * phase_time / period)
        request = SetEntityState.Request()
        state = EntityState()
        state.name = str(self.get_parameter("entity_name").value)
        state.pose.position.x = float(self.get_parameter("x_m").value)
        state.pose.position.y = y
        state.pose.position.z = float(self.get_parameter("z_m").value)
        state.pose.orientation.w = 1.0
        state.twist.linear.y = (2.0 * math.pi / period) * float(self.get_parameter("y_amplitude_m").value) * math.cos(
            2.0 * math.pi * phase_time / period
        )
        state.reference_frame = "world"
        request.state = state
        client.call_async(request)
        self.state_pub.publish(String(data=f"MOVING entity={state.name} x={state.pose.position.x:.2f} y={y:.2f}"))

    def _ready_client(self):
        for client in self._gazebo_state_clients:
            if client.service_is_ready():
                return client
        return None

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboDynamicObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
