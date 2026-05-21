from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class PointCloudLidarObjectsNode(Node):
    """Convert 3D LiDAR PointCloud2 into elevated object center candidates.

    역할:
      - 3D LiDAR raw cloud에서 높이/거리 조건을 만족하는 점만 골라 `/waver/lidar_objects` PoseArray를 낸다.
      - 차체 회전 명령이나 `/cmd_vel`은 절대 발행하지 않는다.
      - sklearn이 없어도 voxel adjacency clustering으로 fail-safe 동작한다.
    """

    def __init__(self) -> None:
        super().__init__("pointcloud_lidar_objects_node")
        self.declare_parameter("pointcloud_topic", "/mid360_PointCloud2")
        self.declare_parameter("output_pose_array_topic", "/waver/lidar_objects")
        self.declare_parameter("state_topic", "/waver/lidar_objects_state")
        self.declare_parameter("min_height_m", 0.35)
        self.declare_parameter("max_height_m", 8.0)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 20.0)
        self.declare_parameter("voxel_leaf_size_m", 0.20)
        self.declare_parameter("cluster_eps_m", 0.55)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("remove_robot_body", True)
        self.declare_parameter("self_x_min", -0.6)
        self.declare_parameter("self_x_max", 0.6)
        self.declare_parameter("self_y_min", -0.6)
        self.declare_parameter("self_y_max", 0.6)
        self.declare_parameter("self_z_min", -0.3)
        self.declare_parameter("self_z_max", 0.8)

        self.objects_pub = self.create_publisher(PoseArray, str(self.get_parameter("output_pose_array_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("pointcloud_topic").value), self.cloud_callback, 5)

    def cloud_callback(self, msg: PointCloud2) -> None:
        try:
            points, raw_count = self._filtered_voxel_points(msg)
            clusters = self._cluster(points)
            pose_array = PoseArray()
            pose_array.header = msg.header
            for cluster in clusters:
                if len(cluster) < int(self.get_parameter("cluster_min_points").value):
                    continue
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                cz = sum(p[2] for p in cluster) / len(cluster)
                pose = Pose()
                pose.position.x = float(cx)
                pose.position.y = float(cy)
                pose.position.z = float(cz)
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
            self.objects_pub.publish(pose_array)
            state = "OBJECTS_OK" if pose_array.poses else "NO_OBJECTS"
            self.state_pub.publish(String(data=f"{state} frame={msg.header.frame_id} raw={raw_count} voxels={len(points)} clusters={len(pose_array.poses)}"))
        except Exception as exc:
            self.state_pub.publish(String(data=f"OBJECTS_FAILED fail_safe_no_cmd_vel error={exc}"))
            self.get_logger().warn(f"PointCloud object extraction failed: {exc}")

    def _filtered_voxel_points(self, msg: PointCloud2) -> tuple[list[tuple[float, float, float]], int]:
        leaf = max(float(self.get_parameter("voxel_leaf_size_m").value), 0.05)
        max_points = int(self.get_parameter("max_points").value)
        voxel: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        raw_count = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            raw_count += 1
            if raw_count > max_points:
                break
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            if self._is_self_point(x, y, z):
                continue
            depth = x
            if depth < float(self.get_parameter("min_depth_m").value) or depth > float(self.get_parameter("max_depth_m").value):
                continue
            if z < float(self.get_parameter("min_height_m").value) or z > float(self.get_parameter("max_height_m").value):
                continue
            key = (round(x / leaf), round(y / leaf), round(z / leaf))
            voxel.setdefault(key, (x, y, z))
        return list(voxel.values()), raw_count

    def _is_self_point(self, x: float, y: float, z: float) -> bool:
        if not bool(self.get_parameter("remove_robot_body").value):
            return False
        return (
            float(self.get_parameter("self_x_min").value) <= x <= float(self.get_parameter("self_x_max").value)
            and float(self.get_parameter("self_y_min").value) <= y <= float(self.get_parameter("self_y_max").value)
            and float(self.get_parameter("self_z_min").value) <= z <= float(self.get_parameter("self_z_max").value)
        )

    def _cluster(self, points: list[tuple[float, float, float]]) -> list[list[tuple[float, float, float]]]:
        if not points:
            return []
        eps = max(float(self.get_parameter("cluster_eps_m").value), 0.05)
        leaf = max(float(self.get_parameter("voxel_leaf_size_m").value), 0.05)
        cell_size = max(eps, leaf)
        cells: dict[tuple[int, int, int], list[int]] = {}
        for i, point in enumerate(points):
            key = tuple(math.floor(v / cell_size) for v in point)
            cells.setdefault(key, []).append(i)
        visited: set[int] = set()
        clusters: list[list[tuple[float, float, float]]] = []
        for i in range(len(points)):
            if i in visited:
                continue
            visited.add(i)
            cluster_indices = []
            queue: deque[int] = deque([i])
            while queue:
                idx = queue.popleft()
                cluster_indices.append(idx)
                for neighbor in self._neighbors(idx, points, cells, cell_size, eps):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            clusters.append([points[idx] for idx in cluster_indices])
        return clusters

    @staticmethod
    def _neighbors(index: int, points: list[tuple[float, float, float]], cells: dict, cell_size: float, eps: float) -> list[int]:
        point = points[index]
        key = tuple(math.floor(v / cell_size) for v in point)
        result: list[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for candidate in cells.get((key[0] + dx, key[1] + dy, key[2] + dz), []):
                        if candidate == index:
                            continue
                        other = points[candidate]
                        if math.dist(point, other) <= eps:
                            result.append(candidate)
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PointCloudLidarObjectsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
