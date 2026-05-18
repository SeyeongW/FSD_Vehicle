"""Fuse LiDAR cluster centroids with camera YOLO detections.

Pipeline:
  /lidar/detections (PoseArray, lidar frame) ─┐
                                              ├─► /target/position (PointStamped, lidar frame)
  /camera/detections (Detection2DArray) ──────┘

A LiDAR centroid is accepted as a confirmed target only if, after projection
into the camera image, it lies inside one of the YOLO bounding boxes (and the
camera detection is fresh). If multiple centroids match, the closest in 2D
range is published.
"""
from __future__ import annotations

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, PointStamped, PoseStamped
from vision_msgs.msg import Detection2DArray

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class LidarCameraFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion_node')

        # Topics
        self.declare_parameter('lidar_topic', '/lidar/detections')
        self.declare_parameter('camera_topic', '/camera/detections')
        self.declare_parameter('target_topic', '/target/position')

        # Frames
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('lidar_frame', 'unilidar_lidar')

        # Pinhole intrinsics (fx, fy, cx, cy) in pixels
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Matching
        self.declare_parameter('camera_stale_sec', 0.3)
        self.declare_parameter('bbox_padding_px', 10.0)

        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx_px = float(self.get_parameter('cx').value)
        self.cy_px = float(self.get_parameter('cy').value)
        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.stale = float(self.get_parameter('camera_stale_sec').value)
        self.pad = float(self.get_parameter('bbox_padding_px').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_cam: Detection2DArray | None = None

        self.create_subscription(Detection2DArray,
                                 self.get_parameter('camera_topic').value,
                                 self.camera_cb, 5)
        self.create_subscription(PoseArray,
                                 self.get_parameter('lidar_topic').value,
                                 self.lidar_cb, 5)
        self.target_pub = self.create_publisher(PointStamped,
                                                self.get_parameter('target_topic').value, 5)

        self.get_logger().info(
            f'Fusion ready: lidar({self.lidar_frame}) x camera({self.camera_frame})')

    def camera_cb(self, msg: Detection2DArray):
        self.latest_cam = msg

    def lidar_cb(self, msg: PoseArray):
        if not msg.poses:
            return
        if self.latest_cam is None:
            return

        cam_t = Time.from_msg(self.latest_cam.header.stamp)
        now = self.get_clock().now()
        if (now - cam_t) > Duration(seconds=self.stale):
            return

        # Static transform lidar -> camera_optical
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, msg.header.frame_id,
                Time(), timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF unavailable: {e}', throttle_duration_sec=2.0)
            return

        best = None  # (score_distance_2d, lidar_pose)
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            cam_pose = do_transform_pose(ps.pose, tf)
            x, y, z = cam_pose.position.x, cam_pose.position.y, cam_pose.position.z
            if z <= 0.05:  # behind / on the image plane
                continue
            u = self.fx * x / z + self.cx_px
            v = self.fy * y / z + self.cy_px
            if not (0 <= u < self.img_w and 0 <= v < self.img_h):
                continue
            if not self._inside_any_bbox(u, v):
                continue
            r = math.hypot(pose.position.x, pose.position.y)
            if best is None or r < best[0]:
                best = (r, pose)

        if best is None:
            return

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.point.x = best[1].position.x
        out.point.y = best[1].position.y
        out.point.z = best[1].position.z
        self.target_pub.publish(out)

    def _inside_any_bbox(self, u: float, v: float) -> bool:
        for d in self.latest_cam.detections:
            half_x = d.bbox.size_x / 2.0 + self.pad
            half_y = d.bbox.size_y / 2.0 + self.pad
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            if (cx - half_x) <= u <= (cx + half_x) and (cy - half_y) <= v <= (cy + half_y):
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
