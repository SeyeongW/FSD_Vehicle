#!/usr/bin/env python3
"""
trajectory_viz_node.py — 새의 (x,y) 지상 궤적 vs 로봇의 실제 주행 궤적 시각화

/bird_target(odom 프레임, 새의 실시간 위치)과 /odom(로봇 위치)을 각각 누적해서
nav_msgs/Path로 발행한다. rviz2에서 두 Path를 겹쳐 보면 "로봇이 새의 수평면
경로를 얼마나 잘 따라가는지"를 실시간으로 확인할 수 있다.

z는 0으로 눌러서 순수 2D 궤적으로 본다 (추격 제어 자체도 x,y만 쓰므로 동일 기준).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path


class TrajectoryVizNode(Node):
    def __init__(self):
        super().__init__('trajectory_viz_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_points', 1000)
        # 새 경로는 최근 이 시간(초)만큼만 남기고 그 전 구간은 잘라내서, 지나간
        # 경로가 뒤로 갈수록 자연스럽게 사라지는 것처럼 보이게 함
        self.declare_parameter('bird_path_window_sec', 3.0)
        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.bird_path_window_sec = float(self.get_parameter('bird_path_window_sec').value)

        self.bird_path = Path()
        self.bird_path.header.frame_id = self.frame_id
        self.robot_path = Path()
        self.robot_path.header.frame_id = self.frame_id

        self.bird_path_pub = self.create_publisher(Path, '/bird_track_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/robot_track_path', 10)

        self.create_subscription(PointStamped, '/bird_target', self._bird_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.get_logger().info(
            f'Trajectory viz node started (frame={self.frame_id}, max_points={self.max_points}).'
        )

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _append(self, path: Path, x: float, y: float, stamp, publisher):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)
        if len(path.poses) > self.max_points:
            path.poses = path.poses[-self.max_points:]

        path.header.stamp = stamp
        publisher.publish(path)

    def _bird_cb(self, msg: PointStamped):
        cutoff = self._stamp_to_sec(msg.header.stamp) - self.bird_path_window_sec
        self.bird_path.poses = [
            p for p in self.bird_path.poses if self._stamp_to_sec(p.header.stamp) >= cutoff
        ]
        self._append(self.bird_path, msg.point.x, msg.point.y, msg.header.stamp, self.bird_path_pub)

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._append(self.robot_path, pos.x, pos.y, msg.header.stamp, self.robot_path_pub)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
