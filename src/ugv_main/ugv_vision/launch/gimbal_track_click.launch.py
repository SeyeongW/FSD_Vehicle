"""Click-to-lock LiDAR + camera + gimbal tracking.

Open the OpenCV window (on a monitor attached to the Pi), click a detected
person to lock onto them; the LiDAR-camera fusion node resolves that target's
3D position and the gimbal aims at it.

This launch opens the camera directly inside yolo_track_select_node
(cv2.VideoCapture), so no separate camera driver is needed. Bring up the
Unitree L1 LiDAR driver and the lidar<->camera static TF separately:

  ros2 launch unitree_lidar_ros2 launch.py
  ros2 run tf2_ros static_transform_publisher X Y Z YAW PITCH ROLL \
      unilidar_lidar camera_optical_frame
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ugv_vision'),
        'config', 'gimbal_track_click.yaml')

    serial_port = LaunchConfiguration('serial_port')
    camera_index = LaunchConfiguration('camera_index')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='SIYI gimbal serial port'),
        DeclareLaunchArgument('camera_index', default_value='0',
                              description='cv2.VideoCapture camera index'),

        Node(
            package='ugv_lidar_detection',
            executable='lidar_detector_node',
            name='lidar_detector_node',
            output='screen',
            parameters=[{
                'pointcloud_topic': '/unilidar/cloud',
                'z_min_filter': -0.3,
                'z_max_filter': 2.5,
                'cluster_eps': 0.6,
                'cluster_min_samples': 10,
                'downsample_rate': 2,
                'max_range': 15.0,
            }],
        ),
        Node(
            package='ugv_vision',
            executable='yolo_track_select_node',
            name='yolo_track_select_node',
            output='screen',
            parameters=[config, {'camera_index': camera_index}],
        ),
        Node(
            package='ugv_vision',
            executable='lidar_camera_fusion_node',
            name='lidar_camera_fusion_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='ugv_vision',
            executable='gimbal_controller_node',
            name='gimbal_controller_node',
            output='screen',
            parameters=[config, {'serial_port': serial_port}],
        ),
    ])
