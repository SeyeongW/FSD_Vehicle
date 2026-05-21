"""LiDAR-first tracking: detect moving objects, click one, gimbal follows it.

Flow:
  Unitree L1 -> lidar_detector (cluster) -> lidar_motion_tracker (flag moving)
            -> RViz shows moving objects in red; click one with "Publish Point"
            -> cluster_select locks/follows it -> /target/position
            -> gimbal_controller aims the SIYI A8 mini at it
            -> /v2v/* topics broadcast the moving objects / chosen target

Bring up the LiDAR driver separately first:
  ros2 launch unitree_lidar_ros2 launch.py

The gimbal serial port and the LiDAR USB port must differ
(check `ls -l /dev/serial/by-id/`).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    output_frame = LaunchConfiguration('output_frame')
    # Station LiDAR pose in the shared map (measure once; x y z yaw pitch roll).
    station_xyz = [LaunchConfiguration(k) for k in ('station_x', 'station_y', 'station_z')]
    station_rpy = [LaunchConfiguration(k) for k in ('station_yaw', 'station_pitch', 'station_roll')]

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB1',
                              description='SIYI A8 mini gimbal serial port'),
        DeclareLaunchArgument('output_frame', default_value='map',
                              description='Shared world frame for V2V target poses'),
        DeclareLaunchArgument('station_x', default_value='0.0'),
        DeclareLaunchArgument('station_y', default_value='0.0'),
        DeclareLaunchArgument('station_z', default_value='0.0'),
        DeclareLaunchArgument('station_yaw', default_value='0.0'),
        DeclareLaunchArgument('station_pitch', default_value='0.0'),
        DeclareLaunchArgument('station_roll', default_value='0.0'),

        # Where the (stationary) control LiDAR sits in the shared map.
        # Replace the defaults with the measured pose via launch args.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_lidar_static_tf',
            arguments=station_xyz + station_rpy + ['map', 'unilidar_lidar'],
        ),

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
            package='ugv_lidar_detection',
            executable='lidar_motion_tracker_node',
            name='lidar_motion_tracker_node',
            output='screen',
            parameters=[{
                'detections_topic': '/lidar/detections',
                'assoc_gate': 1.0,
                'speed_thresh': 0.3,
                'min_hits': 3,
                'max_misses': 5,
                'vel_alpha': 0.5,
                'marker_lifetime': 0.4,
            }],
        ),
        Node(
            package='ugv_lidar_detection',
            executable='cluster_select_node',
            name='cluster_select_node',
            output='screen',
            parameters=[{
                'click_gate': 1.5,
                'lost_timeout': 1.0,
                'output_frame': output_frame,
            }],
        ),
        Node(
            package='ugv_vision',
            executable='gimbal_controller_node',
            name='gimbal_controller_node',
            output='screen',
            parameters=[{
                'target_topic': '/target/position',
                'serial_port': serial_port,
                'baud': 115200,
                'kp_yaw': 2.5,
                'kp_pitch': 2.5,
                'deadband_deg': 1.5,
                'max_speed': 60,
                'control_rate_hz': 20.0,
                'target_timeout_sec': 0.5,
            }],
        ),
    ])
