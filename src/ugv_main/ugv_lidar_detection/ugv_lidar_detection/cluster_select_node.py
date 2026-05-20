"""Click a moving LiDAR object in RViz; lock and follow it.

Use RViz's "Publish Point" tool to click near a highlighted (moving) object.
This node locks onto the nearest moving track and then keeps following it by
track id as it moves.

Input :
  /clicked_point          (PointStamped)  -- RViz Publish Point
  /lidar/tracked_markers  (MarkerArray)   -- from lidar_motion_tracker_node
Output:
  /target/position        (PointStamped)  -- drives gimbal_controller_node
  /v2v/selected_target    (PoseStamped)   -- the chosen target, for sharing
  /lidar/selected_marker  (Marker)        -- bright highlight in RViz

Frame note: coordinates are in the LiDAR frame; transform to a shared map
frame for true vehicle-to-vehicle exchange.
"""
from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class ClusterSelectNode(Node):
    def __init__(self):
        super().__init__('cluster_select_node')

        self.declare_parameter('clicked_topic', '/clicked_point')
        self.declare_parameter('tracks_topic', '/lidar/tracked_markers')
        self.declare_parameter('target_topic', '/target/position')
        self.declare_parameter('v2v_topic', '/v2v/selected_target')
        self.declare_parameter('click_gate', 1.5)     # max click->object distance (m)
        self.declare_parameter('lost_timeout', 1.0)   # release lock after N s missing

        self.click_gate = float(self.get_parameter('click_gate').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)

        # current moving objects: id -> (x, y, z)
        self.moving: dict[int, tuple] = {}
        self.frame_id = 'unilidar_lidar'
        self.locked_id = None
        self.last_seen = None

        self.create_subscription(MarkerArray,
                                 self.get_parameter('tracks_topic').value,
                                 self.tracks_cb, 10)
        self.create_subscription(PointStamped,
                                 self.get_parameter('clicked_topic').value,
                                 self.click_cb, 10)
        self.target_pub = self.create_publisher(PointStamped,
                                                self.get_parameter('target_topic').value, 5)
        self.v2v_pub = self.create_publisher(PoseStamped,
                                             self.get_parameter('v2v_topic').value, 5)
        self.highlight_pub = self.create_publisher(Marker, '/lidar/selected_marker', 5)

        self.get_logger().info('Cluster select ready. Use RViz "Publish Point" on a moving object.')

    def tracks_cb(self, msg: MarkerArray):
        self.moving = {}
        for m in msg.markers:
            if m.ns == 'moving' and m.action == Marker.ADD:
                self.frame_id = m.header.frame_id or self.frame_id
                self.moving[m.id] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

        if self.locked_id is None:
            return

        now = self.get_clock().now()
        if self.locked_id in self.moving:
            self.last_seen = now
            self._emit(self.moving[self.locked_id])
        elif self.last_seen is not None and (now - self.last_seen) > Duration(seconds=self.lost_timeout):
            self.get_logger().info(f'Target #{self.locked_id} lost — releasing lock')
            self.locked_id = None
            self.last_seen = None

    def click_cb(self, msg: PointStamped):
        if not self.moving:
            self.get_logger().warn('Click ignored: no moving objects right now')
            return
        cx, cy = msg.point.x, msg.point.y
        best_id, best_d = None, self.click_gate
        for tid, (x, y, _z) in self.moving.items():
            d = math.hypot(x - cx, y - cy)
            if d < best_d:
                best_d, best_id = d, tid
        if best_id is None:
            self.get_logger().warn(
                f'Click ignored: nearest moving object > {self.click_gate} m away')
            return
        self.locked_id = best_id
        self.last_seen = self.get_clock().now()
        self.get_logger().info(f'Locked moving target #{best_id}')
        self._emit(self.moving[best_id])

    def _emit(self, pos):
        x, y, z = pos
        stamp = self.get_clock().now().to_msg()

        target = PointStamped()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        target.point.x, target.point.y, target.point.z = x, y, z
        self.target_pub.publish(target)

        v2v = PoseStamped()
        v2v.header.stamp = stamp
        v2v.header.frame_id = self.frame_id
        v2v.pose.position.x, v2v.pose.position.y, v2v.pose.position.z = x, y, z
        v2v.pose.orientation.w = 1.0
        self.v2v_pub.publish(v2v)

        hl = Marker()
        hl.header.stamp = stamp
        hl.header.frame_id = self.frame_id
        hl.ns = 'selected'
        hl.id = 0
        hl.type = Marker.SPHERE
        hl.action = Marker.ADD
        hl.pose.position.x, hl.pose.position.y, hl.pose.position.z = x, y, z
        hl.pose.orientation.w = 1.0
        hl.scale.x = hl.scale.y = hl.scale.z = 0.8
        hl.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.7)
        hl.lifetime.sec = 1
        self.highlight_pub.publish(hl)


def main(args=None):
    rclpy.init(args=args)
    node = ClusterSelectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
