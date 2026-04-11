#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')

        self.declare_parameter('input_topic', '/mid360_PointCloud2')
        self.declare_parameter('output_topic', '/scan')

        self.declare_parameter('frame_id', 'base_link')

        self.declare_parameter('min_height', -0.10)
        self.declare_parameter('max_height', 0.40)

        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0058)

        self.declare_parameter('range_min', 0.20)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('scan_time', 0.1)

        self.declare_parameter('use_inf', True)
        self.declare_parameter('inf_epsilon', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            10
        )

        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(f'Subscribed to PointCloud2: {input_topic}')
        self.get_logger().info(f'Publishing LaserScan: {output_topic}')

    def cloud_callback(self, msg: PointCloud2):
        frame_id = self.get_parameter('frame_id').value

        min_height = float(self.get_parameter('min_height').value)
        max_height = float(self.get_parameter('max_height').value)

        angle_min = float(self.get_parameter('angle_min').value)
        angle_max = float(self.get_parameter('angle_max').value)
        angle_increment = float(self.get_parameter('angle_increment').value)

        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        scan_time = float(self.get_parameter('scan_time').value)

        use_inf = bool(self.get_parameter('use_inf').value)
        inf_epsilon = float(self.get_parameter('inf_epsilon').value)

        if angle_max <= angle_min:
            self.get_logger().error('angle_max must be greater than angle_min')
            return

        num_readings = int(math.ceil((angle_max - angle_min) / angle_increment))
        if num_readings <= 0:
            self.get_logger().error('Invalid number of scan bins')
            return

        if use_inf:
            ranges = [float('inf')] * num_readings
        else:
            ranges = [range_max + inf_epsilon] * num_readings

        try:
            points = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )

            valid_points = 0

            for p in points:
                x, y, z = p[0], p[1], p[2]

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                if z < min_height or z > max_height:
                    continue

                r = math.sqrt(x * x + y * y)
                if r < range_min or r > range_max:
                    continue

                angle = math.atan2(y, x)
                if angle < angle_min or angle > angle_max:
                    continue

                index = int((angle - angle_min) / angle_increment)
                if 0 <= index < num_readings:
                    if r < ranges[index]:
                        ranges[index] = r
                    valid_points += 1

            scan_msg = LaserScan()
            scan_msg.header.stamp = msg.header.stamp
            scan_msg.header.frame_id = frame_id if frame_id else msg.header.frame_id

            scan_msg.angle_min = angle_min
            scan_msg.angle_max = angle_max
            scan_msg.angle_increment = angle_increment

            scan_msg.time_increment = 0.0
            scan_msg.scan_time = scan_time

            scan_msg.range_min = range_min
            scan_msg.range_max = range_max

            scan_msg.ranges = ranges
            scan_msg.intensities = []

            self.pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to convert PointCloud2 to LaserScan: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()