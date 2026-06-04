#!/usr/bin/env python3
"""
/quit_signal 수신 → 로봇을 초기 위치(0,0,yaw=0)로 복귀 → 노드 종료
patrol_nav.launch.py 에서 OnProcessExit 으로 Shutdown() 트리거됨.
"""
import math
import os
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class QuitHandler(Node):

    HOME_TIMEOUT_SEC = 40.0  # 복귀 제한 시간

    def __init__(self):
        super().__init__('quit_handler')
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._done = False

        self.create_subscription(Empty, '/quit_signal', self._on_quit, 10)
        self.get_logger().info('[QuitHandler] 대기 중 — /quit_signal 수신 시 복귀 시작')

    def _on_quit(self, _):
        if self._done:
            return
        self._done = True
        self.get_logger().info('[QuitHandler] 복귀 명령 수신 — (0, 0) 으로 이동합니다.')

        if not self._nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('[QuitHandler] Nav2 없음 — 즉시 종료')
            self._shutdown()
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation = _yaw_to_quat(0.0)

        send_future = self._nav.send_goal_async(goal)
        send_future.add_done_callback(self._on_accepted)

        # 타임아웃 타이머
        self._timeout_timer = self.create_timer(
            self.HOME_TIMEOUT_SEC, self._on_timeout
        )

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[QuitHandler] Goal 거부 — 즉시 종료')
            self._shutdown()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info('[QuitHandler] 복귀 완료 — 종료합니다.')
        self._shutdown()

    def _on_timeout(self):
        self.get_logger().warn(
            f'[QuitHandler] {self.HOME_TIMEOUT_SEC:.0f}초 초과 — 강제 종료'
        )
        self._shutdown()

    def _shutdown(self):
        # 별도 스레드에서 종료 — 콜백 안에서 직접 호출 시 deadlock 방지
        threading.Thread(target=self._do_exit, daemon=True).start()

    def _do_exit(self):
        import time
        self.get_logger().info('[QuitHandler] 종료합니다.')
        time.sleep(0.3)
        os._exit(0)  # 프로세스 즉시 종료 → OnProcessExit → Shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = QuitHandler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
