"""
patrol_nav.launch.py — Nav2 + 순찰 + LiDAR 클러스터 통합 실행

사용 순서:
  Terminal 1: ros2 launch ugv_gazebo bringup.launch.py
  Terminal 2: ros2 launch ugv_gazebo patrol_nav.launch.py
  Terminal 3: ros2 run pcd_to_scan_pkg pointcloud_to_laserscan_node
  Terminal 4: ros2 run pcd_cluster_pkg bird_input_node

포함 내용:
  - AMCL + Nav2 + RViz2 (nav.launch.py)
  - cluster_node  (LiDAR 새 감지 + cmd_vel 추적)
  - patrol_node   (지그재그 순찰 상태 기계)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')

    # Nav2 + AMCL + RViz2
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_gazebo_dir, 'launch', 'nav', 'nav.launch.py')
        ),
        launch_arguments={
            'use_localization': 'amcl',
            'use_localplan': 'teb',
        }.items()
    )

    # LiDAR 새 감지 + 추적 cmd_vel
    cluster_node = Node(
        package='pcd_cluster_pkg',
        executable='cluster_node',
        name='cluster_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 지그재그 순찰 상태 기계 (Nav2 액션 클라이언트)
    # Nav2가 준비될 시간을 위해 5초 딜레이
    patrol_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='pcd_cluster_pkg',
                executable='patrol_node',
                name='patrol_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        nav_launch,
        cluster_node,
        patrol_node,
    ])
