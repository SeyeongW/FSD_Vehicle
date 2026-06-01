#!/usr/bin/env python3
"""
bird_input_node.py — 터미널 입력으로 새 출현/소멸 제어

사용:
  ros2 run pcd_cluster_pkg bird_input_node

입력:
  bird_in  → /bird_command "bird_in"  퍼블리시 → bird_manager 새 활성화
  bird_out → /bird_command "bird_out" 퍼블리시 → bird_manager 새 숨김
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BirdInputNode(Node):
    def __init__(self):
        super().__init__('bird_input_node')
        self.pub = self.create_publisher(String, '/bird_command', 10)
        self.get_logger().info(
            'BirdInputNode ready.\n'
            '  bird_in  → 새 출현\n'
            '  bird_out → 새 소멸\n'
            '입력 대기 중...'
        )
        self._thread = threading.Thread(target=self._input_loop, daemon=True)
        self._thread.start()

    def _input_loop(self):
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd in ('bird_in', 'bird_out'):
                    msg = String()
                    msg.data = cmd
                    self.pub.publish(msg)
                    self.get_logger().info(f'Sent: {cmd}')
                elif cmd:
                    self.get_logger().warn(
                        f'Unknown command: "{cmd}". Use bird_in or bird_out.'
                    )
            except Exception as e:
                self.get_logger().error(f'Input error: {e}')
                break


def main(args=None):
    rclpy.init(args=args)
    node = BirdInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
