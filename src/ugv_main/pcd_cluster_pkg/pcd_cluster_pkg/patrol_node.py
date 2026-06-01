#!/usr/bin/env python3
"""
patrol_node.py — 지그재그 순찰 + 새 추적 상태 기계

[상태]
  PATROL  : Nav2 NavigateToPose로 웨이포인트 순차 순찰
  TRACKING: /bird_detected True → Nav2 취소, cluster_node가 cmd_vel 제어

[초기화]
  AMCL 초기 위치를 (0, 0, yaw=0)으로 자동 퍼블리시.
  로봇은 항상 (0,0)에서 +X 방향을 바라보고 스폰됨.

[토픽]
  /bird_detected      (Bool, sub) — 새 감지 여부
  /tracking_active    (Bool, pub) — cluster_node cmd_vel 허용 여부
  /initialpose        (PoseWithCovarianceStamped, pub) — AMCL 초기 위치
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _lawnmower_waypoints():
    """
    맵 내부(-5 ~ 5m) 지그재그 웨이포인트.
    X축 방향 2m 간격 트랙, 각 트랙은 Y축 방향으로 왕복.
    관제탑(-6, 6)과 겹치지 않는 범위.
    """
    x_tracks = [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0]
    y_low, y_high = -5.0, 5.0
    wps = []
    for i, x in enumerate(x_tracks):
        if i % 2 == 0:
            wps.append((x, y_low, -math.pi / 2))
            wps.append((x, y_high,  math.pi / 2))
        else:
            wps.append((x, y_high,  math.pi / 2))
            wps.append((x, y_low, -math.pi / 2))
    return wps


PATROL   = 'patrol'
TRACKING = 'tracking'


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.waypoints  = _lawnmower_waypoints()
        self.wp_idx     = 0
        self.state      = PATROL
        self._goal_handle   = None
        self._started       = False
        self._startup_count = 0
        self._retry_timer   = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 5
        )
        self.tracking_active_pub = self.create_publisher(Bool, '/tracking_active', 10)

        self.bird_sub = self.create_subscription(
            Bool, '/bird_detected', self._bird_cb, 10
        )

        # 1초마다 startup 체크 (Nav2 준비 대기)
        self.create_timer(1.0, self._startup_tick)
        self.get_logger().info('patrol_node started — waiting for Nav2...')

    # ─────────────────────────────────────────────────────────────
    # 시작 시퀀스
    # ─────────────────────────────────────────────────────────────
    def _startup_tick(self):
        if self._started:
            return
        self._startup_count += 1

        if self._startup_count == 5:
            self._publish_initial_pose()
            self.get_logger().info('Initial pose published to AMCL (0, 0, yaw=0).')

        if self._startup_count >= 8:
            if self._nav_client.wait_for_server(timeout_sec=1.0):
                self._started = True
                self._set_tracking_active(False)
                self._send_next_waypoint()
                self.get_logger().info('Patrol started.')
            else:
                self.get_logger().warn('Nav2 server not ready yet...')

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x    = 0.0
        msg.pose.pose.position.y    = 0.0
        msg.pose.pose.orientation   = yaw_to_quat(0.0)
        msg.pose.covariance[0]  = 0.25   # x 분산
        msg.pose.covariance[7]  = 0.25   # y 분산
        msg.pose.covariance[35] = 0.07   # yaw 분산
        self.initial_pose_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────
    # 새 감지 콜백 → 상태 전환
    # ─────────────────────────────────────────────────────────────
    def _bird_cb(self, msg: Bool):
        if msg.data and self.state == PATROL:
            self.get_logger().info('[STATE] PATROL → TRACKING (bird detected)')
            self.state = TRACKING
            self._cancel_current_goal()
            self._set_tracking_active(True)

        elif not msg.data and self.state == TRACKING:
            self.get_logger().info('[STATE] TRACKING → PATROL (bird lost)')
            self.state = PATROL
            self._set_tracking_active(False)
            self._send_next_waypoint()

    def _set_tracking_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.tracking_active_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────
    # Nav2 웨이포인트 전송
    # ─────────────────────────────────────────────────────────────
    def _send_next_waypoint(self):
        if self.state != PATROL or not self._started:
            return

        wp = self.waypoints[self.wp_idx]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id   = 'map'
        goal.pose.header.stamp      = self.get_clock().now().to_msg()
        goal.pose.pose.position.x   = float(wp[0])
        goal.pose.pose.position.y   = float(wp[1])
        goal.pose.pose.position.z   = 0.0
        goal.pose.pose.orientation  = yaw_to_quat(float(wp[2]))

        self.get_logger().info(
            f'[PATROL] WP {self.wp_idx}/{len(self.waypoints)}: '
            f'({wp[0]:.1f}, {wp[1]:.1f})'
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected. Advancing to next waypoint.')
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self._schedule_retry()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if self.state != PATROL:
            return
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'WP {self.wp_idx} reached.')
        else:
            self.get_logger().warn(
                f'WP {self.wp_idx} ended with status {result.status}.'
            )
        self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        self._goal_handle = None
        self._send_next_waypoint()

    def _schedule_retry(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        self._retry_timer = self.create_timer(2.0, self._retry_once)

    def _retry_once(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._send_next_waypoint()

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
