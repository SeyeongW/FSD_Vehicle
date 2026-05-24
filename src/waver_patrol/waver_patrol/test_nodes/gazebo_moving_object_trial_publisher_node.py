from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
except Exception:  # pragma: no cover - gazebo_msgs may be absent on desk-only installs
    EntityState = None
    SetEntityState = None


@dataclass
class TrialPath:
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    z: float


class GazeboMovingObjectTrialPublisherNode(Node):
    """Gazebo/test-only elevated-target and fake cluster publisher.

    역할:
      - 높이 3m 이상 동적 객체 검증용 trajectory를 만든다.
      - cluster_node.py가 없는 환경에서도 같은 인터페이스인 `/waver/lidar_objects` PoseArray를 발행한다.
      - Gazebo entity가 있으면 `/set_entity_state`로 시각 target도 같이 이동시킨다.
      - 실차 launch에서는 절대 사용하지 않는다.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_moving_object_trial_publisher_node")
        self.declare_parameter("trial_id", 1)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("output_topic", "/waver/lidar_objects")
        self.declare_parameter("state_topic", "/waver/gazebo_trial_target_state")
        self.declare_parameter("camera_state_topic", "/waver/camera_detection_state")
        self.declare_parameter("external_class_topic", "/waver/external_target_class")
        self.declare_parameter("external_confidence_topic", "/waver/external_target_confidence")
        self.declare_parameter("duration_sec", 8.0)
        self.declare_parameter("timer_hz", 10.0)
        self.declare_parameter("target_z", 3.2)
        self.declare_parameter("gazebo_entity_name", "bird_test_target")
        self.declare_parameter("move_gazebo_entity", True)
        self.declare_parameter("gazebo_entity_move_delay_sec", 5.0)
        self.declare_parameter("publish_fake_camera_after_sec", 12.0)
        self.declare_parameter("fake_class_name", "bird")
        self.declare_parameter("fake_confidence", 0.92)

        self.pub = self.create_publisher(PoseArray, str(self.get_parameter("output_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.camera_state_pub = self.create_publisher(String, str(self.get_parameter("camera_state_topic").value), 10)
        self.class_pub = self.create_publisher(String, str(self.get_parameter("external_class_topic").value), 10)
        self.conf_pub = self.create_publisher(Float32, str(self.get_parameter("external_confidence_topic").value), 10)
        # 역할: Gazebo가 이미 실행 중인 상태에서 이 노드만 나중에 붙으면
        # use_sim_time clock이 0이 아닌 값으로 시작한다. 첫 tick에서 mission 시작
        # 시각을 잡아야 target이 첫 프레임부터 종료점으로 점프하지 않는다.
        self.start_time: float | None = None
        self.path = self._trial_path(int(self.get_parameter("trial_id").value))
        self.set_entity_client = None
        if SetEntityState is not None:
            self.set_entity_client = self.create_client(SetEntityState, "/set_entity_state")
        self.create_timer(1.0 / max(float(self.get_parameter("timer_hz").value), 1.0), self.tick)
        self.get_logger().warn("Gazebo moving object trial publisher is simulation-only")

    def tick(self) -> None:
        now = self._now()
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time
        duration = max(float(self.get_parameter("duration_sec").value), 0.1)
        ratio = min(max(t / duration, 0.0), 1.0)
        x = self.path.start_x + (self.path.end_x - self.path.start_x) * ratio
        y = self.path.start_y + (self.path.end_y - self.path.start_y) * ratio
        z = self.path.z
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.poses.append(pose)
        self.pub.publish(msg)
        displacement = math.hypot(x - self.path.start_x, y - self.path.start_y)
        total = math.hypot(self.path.end_x - self.path.start_x, self.path.end_y - self.path.start_y)
        self.state_pub.publish(
            String(
                data=(
                    f"TRIAL_TARGET trial_id={self.get_parameter('trial_id').value} "
                    f"x={x:.3f} y={y:.3f} z={z:.3f} displacement={displacement:.3f} "
                    f"target_total={total:.3f}"
                )
            )
        )
        self._move_gazebo_entity(x, y, z)
        if t >= float(self.get_parameter("publish_fake_camera_after_sec").value):
            self.camera_state_pub.publish(String(data="DETECTED tracking_state=TRACKING source=gazebo_trial"))
            self.class_pub.publish(String(data=str(self.get_parameter("fake_class_name").value)))
            self.conf_pub.publish(Float32(data=float(self.get_parameter("fake_confidence").value)))

    def _move_gazebo_entity(self, x: float, y: float, z: float) -> None:
        if not bool(self.get_parameter("move_gazebo_entity").value):
            return
        if self.start_time is None:
            return
        if self._now() - self.start_time < float(self.get_parameter("gazebo_entity_move_delay_sec").value):
            return
        if self.set_entity_client is None or EntityState is None or SetEntityState is None:
            return
        if not self.set_entity_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = str(self.get_parameter("gazebo_entity_name").value)
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        request.state.pose.orientation.w = 1.0
        self.set_entity_client.call_async(request)

    def _trial_path(self, trial_id: int) -> TrialPath:
        z = float(self.get_parameter("target_z").value)
        # H1: height>=3m dynamic, H2: height>=3m static, H3: low-altitude dynamic.
        if trial_id == 2:
            return TrialPath(2.0, -1.0, 2.0, -1.0, z)
        if trial_id == 3:
            return TrialPath(1.5, 1.0, 2.1, 1.0, 1.0)
        if trial_id == 4:
            return TrialPath(2.0, 0.0, 2.0, 0.0, z)
        if trial_id == 5:
            return TrialPath(2.0, -2.0, 2.7, -1.4, z)
        return TrialPath(2.0, 0.0, 2.7, 0.0, z)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboMovingObjectTrialPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
