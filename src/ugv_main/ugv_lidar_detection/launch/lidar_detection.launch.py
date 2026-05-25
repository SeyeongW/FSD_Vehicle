import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ugv_lidar_detection',
            executable='lidar_detector_node',
            name='lidar_detector_node',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/unilidar/cloud'},
                {'z_min_filter': -0.3},
                {'z_max_filter': 2.5},
                {'cluster_eps': 0.6},
                {'cluster_min_samples': 10},
                {'downsample_rate': 2}
            ]
        )
    ])
