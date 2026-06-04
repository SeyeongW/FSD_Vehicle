#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')
        self.create_subscription(String, '/robot_status', self._cb, 10)

    def _cb(self, msg: String):
        print(msg.data, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
