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

        # Background subtraction (for a STATIONARY lidar): learn voxels that are
        # repeatedly occupied (walls/floor) and keep only "new" points.
        # The L1 has a non-repetitive scan (each frame hits different points),
        # so we accumulate hits over time and expire by "last seen" rather than
        # decaying every frame -- otherwise the map never densifies.
        self.declare_parameter('use_background_subtraction', True)
        self.declare_parameter('bg_voxel_size', 0.2)    # voxel edge (m)
        self.declare_parameter('bg_threshold', 4)       # hits -> counts as background
        self.declare_parameter('bg_cap', 30)            # max occupancy score
        self.declare_parameter('bg_expire_frames', 60)  # drop voxel if unseen this long

        cloud_topic = self.get_parameter('pointcloud_topic').value

        # Background model: voxel (ix,iy,iz) -> [score, last_seen_frame]
        self.bg = {}
        self.frame_count = 0

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
        # Foreground (non-background) points for a clean RViz view.
        self.foreground_pub = self.create_publisher(PointCloud2, '/lidar/foreground', 10)
        # Accumulated static structure (background voxel centers) -- a dense,
        # stable, SLAM-like map that updates as the scene changes.
        self.map_pub = self.create_publisher(PointCloud2, '/lidar/map', 10)

        self.get_logger().info(f'Lidar Object Detection Node started. Listening to {cloud_topic}')

    def cloud_callback(self, msg):
        # 1. Read points as an (N, 3) float array.
        # read_points_numpy can't handle clouds with mixed-datatype fields
        # (the L1 has float xyz plus other-typed intensity/ring/time), so use
        # read_points and pull x/y/z by name. Modern sensor_msgs_py returns a
        # structured ndarray; older versions yield tuples.
        arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if hasattr(arr, "dtype") and arr.dtype.names:
            points = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32)
        else:
            points = np.array([[p[0], p[1], p[2]] for p in arr], dtype=np.float32)

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

        # Background subtraction: keep only points not part of the static scene.
        if self.get_parameter('use_background_subtraction').value:
            filtered_points = self._background_filter(filtered_points)
            self._publish_map(msg.header)

        # Publish the foreground cloud for a clean RViz view (even if empty).
        fg_msg = pc2.create_cloud_xyz32(msg.header,
                                        filtered_points.tolist() if len(filtered_points) else [])
        self.foreground_pub.publish(fg_msg)

        # 3. Clustering using DBSCAN
        min_samples = self.get_parameter('cluster_min_samples').value
        if len(filtered_points) < min_samples:
            # Nothing to cluster -> publish empty detections so downstream clears.
            empty = PoseArray()
            empty.header = msg.header
            self.detections_pub.publish(empty)
            return

        eps = self.get_parameter('cluster_eps').value
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

    def _background_filter(self, pts):
        """Keep points whose voxel is NOT consistently occupied (i.e. not the
        static scene). Updates the occupancy model each call.

        Static voxels accumulate score up to bg_cap and become background once
        score >= bg_threshold; voxels not seen this frame decay so the model
        adapts and a mover that stops eventually merges into the background.
        """
        if len(pts) == 0:
            return pts

        vsize = float(self.get_parameter('bg_voxel_size').value)
        thresh = int(self.get_parameter('bg_threshold').value)
        cap = int(self.get_parameter('bg_cap').value)
        expire = int(self.get_parameter('bg_expire_frames').value)

        self.frame_count += 1
        now = self.frame_count

        keys = np.floor(pts / vsize).astype(np.int32)
        key_tuples = [tuple(k) for k in keys]
        occupied = set(key_tuples)

        # Accumulate hits (cap) and stamp last-seen for occupied voxels.
        for v in occupied:
            score = self.bg[v][0] if v in self.bg else 0
            self.bg[v] = [min(score + 1, cap), now]
        # Expire voxels not seen for a while (keeps the map dynamic).
        for v in list(self.bg.keys()):
            if now - self.bg[v][1] > expire:
                del self.bg[v]

        # Foreground = voxel hit too few times to be background yet.
        mask = np.array([self.bg.get(v, [0])[0] < thresh for v in key_tuples], dtype=bool)
        return pts[mask]

    def _publish_map(self, header):
        """Publish background voxel centers as a dense, stable map cloud."""
        thresh = int(self.get_parameter('bg_threshold').value)
        vsize = float(self.get_parameter('bg_voxel_size').value)
        centers = [((kx + 0.5) * vsize, (ky + 0.5) * vsize, (kz + 0.5) * vsize)
                   for (kx, ky, kz), (score, _seen) in self.bg.items() if score >= thresh]
        self.map_pub.publish(pc2.create_cloud_xyz32(header, centers))


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
