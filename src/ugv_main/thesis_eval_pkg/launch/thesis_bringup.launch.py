"""
thesis_bringup.launch.py — 졸업논문 단일/다중 LiDAR 비교 실험 런치

hackathon_bringup.launch.py 전체를 그대로 쓰되,
  - lidar_fusion_node(하드 동기화) 대신 ft_fusion_node(고장 허용) 사용
  - mode:=single 이면 온보드 mid360 하나만 융합 입력으로 사용 (나머지 파이프라인 동일)
  - 새 모델 0.5m, 비행 궤적 seed 고정, GT 20Hz
  - fault:=<센서> 이면 새 등장 후 fault_delay초에 그 센서 메시지 차단
  - eval_logger_node로 CSV 기록 (~/ros2_ws/ugv_ws/logs/thesis/)

[실행 예]
  ros2 launch thesis_eval_pkg thesis_bringup.launch.py mode:=multi fault:=mid360 seed:=1
  ros2 topic pub --once /bird_command std_msgs/String "data: bird_in"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ALL_SOURCES = ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'mid360_PointCloud2']
MAX_POINTS = {'lidar1_PointCloud2': 8000, 'lidar2_PointCloud2': 8000, 'mid360_PointCloud2': 15000}
FAULT_TOPIC = {'none': 'none', 'lidar1': 'lidar1_PointCloud2',
               'lidar2': 'lidar2_PointCloud2', 'mid360': 'mid360_PointCloud2'}


def _nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    fault = LaunchConfiguration('fault').perform(context)
    if mode not in ('single', 'multi'):
        raise RuntimeError(f"mode는 single/multi 중 하나여야 함: {mode}")
    if fault not in FAULT_TOPIC:
        raise RuntimeError(f"fault는 {list(FAULT_TOPIC)} 중 하나여야 함: {fault}")

    sources = ['mid360_PointCloud2'] if mode == 'single' else ALL_SOURCES
    fusion = Node(
        package='thesis_eval_pkg', executable='ft_fusion_node', name='ft_fusion_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topics': sources,
            'max_points_per_source': [MAX_POINTS[s] for s in sources],
            'target_frame': 'odom',
            'output_topic': 'fused_pointcloud2',
            'bounds_xy': 11.0,
            'fault_source': FAULT_TOPIC[fault],
            'fault_delay_sec': float(LaunchConfiguration('fault_delay').perform(context)),
        }],
    )
    nodes = [fusion]
    if LaunchConfiguration('log').perform(context) == 'true':
        nodes.append(Node(
            package='thesis_eval_pkg', executable='eval_logger_node', name='eval_logger_node', output='screen',
            parameters=[{
                'use_sim_time': True,
                'out_dir': os.path.expanduser(LaunchConfiguration('out_dir').perform(context)),
                'mode': mode,
                'fault': fault,
                'seed': int(LaunchConfiguration('seed').perform(context)),
                'source_names': sources,
                'fault_delay_sec': float(LaunchConfiguration('fault_delay').perform(context)),
            }],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('thesis_eval_pkg')
    hackathon = os.path.join(get_package_share_directory('ugv_gazebo'), 'launch', 'hackathon_bringup.launch.py')

    args = [
        DeclareLaunchArgument('mode', default_value='multi', description='single | multi'),
        DeclareLaunchArgument('fault', default_value='none', description='none | mid360 | lidar1 | lidar2'),
        DeclareLaunchArgument('fault_delay', default_value='60.0', description='새 등장 후 고장 주입까지 (sim s)'),
        DeclareLaunchArgument('seed', default_value='1', description='새 비행 궤적 seed'),
        DeclareLaunchArgument('log', default_value='true', description='eval_logger_node 실행 여부'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('steering_mode', default_value='pure_pursuit'),
        DeclareLaunchArgument('out_dir', default_value='~/ros2_ws/ugv_ws/logs/thesis', description='CSV 저장 폴더'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hackathon),
        launch_arguments={
            'use_lidar_fusion': 'false',
            'gui': LaunchConfiguration('gui'),
            'steering_mode': LaunchConfiguration('steering_mode'),
            'bird_sdf_path': os.path.join(share, 'models', 'bird_0p5', 'model.sdf'),
            'bird_random_seed': LaunchConfiguration('seed'),
            'bird_status_rate_hz': '20.0',
        }.items(),
    )

    return LaunchDescription(args + [base, OpaqueFunction(function=_nodes)])
