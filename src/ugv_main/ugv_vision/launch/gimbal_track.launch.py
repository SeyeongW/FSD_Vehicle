"""LiDAR + camera + gimbal tracking pipeline.

Pulls the Livox/Unitree LiDAR clustering node, the Hailo YOLO node, the
fusion node, and the SIYI gimbal controller into a single bring-up.

Excludes the LiDAR driver and camera driver themselves; bring those up via
their own launch files (livox_ros_driver2 and ugv_vision/camera.launch.py)
before launching this one.
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
        'config', 'gimbal_track.yaml')

    serial_port = LaunchConfiguration('serial_port')
    force_cpu = LaunchConfiguration('force_cpu')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='SIYI gimbal serial port'),
        DeclareLaunchArgument('force_cpu', default_value='false',
                              description='Bypass Hailo and use ultralytics CPU'),

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
            executable='yolo_hailo_node',
            name='yolo_hailo_node',
            output='screen',
            parameters=[config, {'force_cpu': force_cpu}],
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
