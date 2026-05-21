import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
from sklearn.cluster import DBSCAN

class LidarDetectorNode(Node):
    def __init__(self):
        super().__init__('lidar_detector_node')

        # Parameters
        self.declare_parameter('pointcloud_topic', '/unilidar/cloud')
        self.declare_parameter('z_min_filter', -0.5)  # Ground removal threshold (Z axis min)
        self.declare_parameter('z_max_filter', 2.0)   # Height removal threshold (Z axis max)
        self.declare_parameter('cluster_eps', 0.5)    # DBSCAN epsilon
        self.declare_parameter('cluster_min_samples', 10) # DBSCAN min samples
        self.declare_parameter('downsample_rate', 5)  # Use every Nth point to speed up
        self.declare_parameter('max_range', 15.0)     # Drop clusters farther than this (m)

        cloud_topic = self.get_parameter('pointcloud_topic').value

        # Subscriptions and Publishers
        # LiDAR drivers publish point clouds with best-effort (sensor_data) QoS;
        # subscribe with the matching profile or no messages arrive.
        self.subscription = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar/detected_objects', 10)
        # Centroid poses consumed by the LiDAR-camera fusion node.
        # Frame == msg.header.frame_id (typically unilidar_lidar).
        self.detections_pub = self.create_publisher(PoseArray, '/lidar/detections', 10)

        self.get_logger().info(f'Lidar Object Detection Node started. Listening to {cloud_topic}')

    def cloud_callback(self, msg):
        # 1. Read points as an (N, 3) float array.
        # read_points_numpy returns a 2D ndarray; the older read_points returns
        # a structured 1D array, so fall back by stacking fields.
        try:
            points = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
        except AttributeError:
            arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.column_stack([arr["x"], arr["y"], arr["z"]])

        if points.ndim != 2 or points.shape[0] == 0:
            return

        # 2. Preprocess: Downsample and Filter Ground (Z-axis thresholding)
        z_min = self.get_parameter('z_min_filter').value
        z_max = self.get_parameter('z_max_filter').value
        downsample_rate = self.get_parameter('downsample_rate').value

        # Filter out points too low (ground) or too high
        z_mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
        filtered_points = points[z_mask]

        # Downsample for faster DBSCAN
        filtered_points = filtered_points[::downsample_rate]

        if len(filtered_points) < 10:
            return

        # 3. Clustering using DBSCAN
        eps = self.get_parameter('cluster_eps').value
        min_samples = self.get_parameter('cluster_min_samples').value
        
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_points)
        labels = clustering.labels_

        # 4. Extract Bounding Boxes
        unique_labels = set(labels)
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header = msg.header

        max_range = self.get_parameter('max_range').value

        marker_id = 0
        for label in unique_labels:
            if label == -1:
                # -1 means noise in DBSCAN
                continue

            cluster_points = filtered_points[labels == label]

            # Simple bounding box
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            cx = float((min_pt[0] + max_pt[0]) / 2.0)
            cy = float((min_pt[1] + max_pt[1]) / 2.0)
            cz = float((min_pt[2] + max_pt[2]) / 2.0)

            if (cx * cx + cy * cy) > max_range * max_range:
                continue

            # Create a Marker
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'objects'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(float(max_pt[0] - min_pt[0]), 0.1)
            marker.scale.y = max(float(max_pt[1] - min_pt[1]), 0.1)
            marker.scale.z = max(float(max_pt[2] - min_pt[2]), 0.1)

            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 sec

            marker_array.markers.append(marker)

            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = cz
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.detections_pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
