from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='plane_fit_pkg',
            executable='plane_fit_node',
            name='plane_fit_node',
            output='screen',
            parameters=[{
                'input_topic': '/mid360_PointCloud2',
                'target_mode': 'ground',
                'distance_threshold': 0.05,
                'ransac_iterations': 500,
                'voxel_size': 0.05,
                'max_points': 15000,
                'process_every_n': 5,
                'z_min': -2.0,
                'z_max': 5.0,
                'range_max': 25.0,
            }]
        )
    ])
