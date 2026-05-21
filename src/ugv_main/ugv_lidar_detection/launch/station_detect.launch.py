"""Control-station detection (no gimbal, no camera).

Pure LiDAR monitoring: detect moving objects, highlight them in RViz, click
one, and broadcast its coordinate for the vehicle. No gimbal is attached yet,
so gimbal_controller_node is omitted.

Flow:
  Unitree L1 -> lidar_detector (cluster) -> lidar_motion_tracker (flag moving)
            -> RViz shows moving objects in red; click one with "Publish Point"
            -> cluster_select -> /v2v/selected_target (PoseStamped, map)

Bring up the LiDAR driver separately:
  ros2 launch unitree_lidar_ros2 launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_frame = LaunchConfiguration('output_frame')
    station_xyz = [LaunchConfiguration(k) for k in ('station_x', 'station_y', 'station_z')]
    station_rpy = [LaunchConfiguration(k) for k in ('station_yaw', 'station_pitch', 'station_roll')]

    return LaunchDescription([
        DeclareLaunchArgument('output_frame', default_value='map',
                              description='Shared world frame for V2V target poses'),
        DeclareLaunchArgument('station_x', default_value='0.0'),
        DeclareLaunchArgument('station_y', default_value='0.0'),
        DeclareLaunchArgument('station_z', default_value='0.0'),
        DeclareLaunchArgument('station_yaw', default_value='0.0'),
        DeclareLaunchArgument('station_pitch', default_value='0.0'),
        DeclareLaunchArgument('station_roll', default_value='0.0'),

        # Where the (stationary) control LiDAR sits in the shared map.
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
    ])
