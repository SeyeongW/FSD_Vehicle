import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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

        cloud_topic = self.get_parameter('pointcloud_topic').value
        
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar/detected_objects', 10)
        
        self.get_logger().info(f'Lidar Object Detection Node started. Listening to {cloud_topic}')

    def cloud_callback(self, msg):
        # 1. Read points from PointCloud2 message
        # Extract x, y, z
        cloud_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(cloud_gen))

        if len(points) == 0:
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
        
        marker_id = 0
        for label in unique_labels:
            if label == -1:
                # -1 means noise in DBSCAN
                continue
                
            cluster_points = filtered_points[labels == label]
            
            # Simple bounding box
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            
            # Create a Marker
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'objects'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Center of the bounding box
            marker.pose.position.x = float((min_pt[0] + max_pt[0]) / 2.0)
            marker.pose.position.y = float((min_pt[1] + max_pt[1]) / 2.0)
            marker.pose.position.z = float((min_pt[2] + max_pt[2]) / 2.0)
            
            # Orientation (default)
            marker.pose.orientation.w = 1.0
            
            # Scale of the bounding box
            marker.scale.x = float(max_pt[0] - min_pt[0])
            marker.scale.y = float(max_pt[1] - min_pt[1])
            marker.scale.z = float(max_pt[2] - min_pt[2])
            
            # Ensure minimum size to be visible
            marker.scale.x = max(marker.scale.x, 0.1)
            marker.scale.y = max(marker.scale.y, 0.1)
            marker.scale.z = max(marker.scale.z, 0.1)
            
            # Color
            marker.color.a = 0.5  # Transparency
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 sec
            
            marker_array.markers.append(marker)
            marker_id += 1

        # Delete unused markers by publishing DELETEALL first or using lifetime
        # Here we use a short lifetime (0.5s) to automatically clear old markers.
        
        self.marker_pub.publish(marker_array)

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
