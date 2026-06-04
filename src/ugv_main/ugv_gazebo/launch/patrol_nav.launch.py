"""
patrol_nav.launch.py — 조류 탐지·순찰 통합 런치

[실행 순서]
  Terminal 1: ros2 launch ugv_gazebo bringup.launch.py
  Terminal 2: ros2 launch ugv_gazebo patrol_nav.launch.py [kf_lookahead:=0.15] [log_data:=true]

[포함 노드]
  즉시 시작:
    - pointcloud_to_laserscan_node  (PointCloud2 → /scan)
    - Nav2 + AMCL + RViz2           (nav.launch.py)
    - cluster_node                   (LiDAR 조류 감지 + KF 추적)
    - bird_yolo_node                 (YOLOv8 시각 보조)

  5초 후:
    - patrol_node                    (순찰/추적 상태 기계)

[조류 제어]
  bird_in:  ros2 topic pub --once /bird_command std_msgs/msg/String "{data: 'bird_in'}"
  bird_out: ros2 topic pub --once /bird_command std_msgs/msg/String "{data: 'bird_out'}"
  종료: Ctrl+C
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# YOLOv8 모델 경로 (workspace 루트에 위치)
_WS_ROOT    = os.path.expanduser('~/ros2_ws/ugv_ws')
_YOLO_MODEL = os.path.join(_WS_ROOT, 'yolov8s.pt')

if not os.path.isfile(_YOLO_MODEL):
    _YOLO_MODEL = 'yolov8s.pt'


def generate_launch_description():
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')

    # log_data:=true 이면 CSV 데이터 로거 활성화
    declare_log_data = DeclareLaunchArgument(
        'log_data', default_value='false',
        description='true 로 설정하면 ~/ros2_ws/ugv_ws/logs/ 에 CSV 저장'
    )
    log_data = LaunchConfiguration('log_data')

    # bird_speed: bringup과 동일한 값을 입력 → data_logger 파일명에 포함
    declare_bird_speed = DeclareLaunchArgument(
        'bird_speed', default_value='0.18',
        description='새 비행 속도 (rad/s). bringup과 동일한 값 입력. 파일명 기록용'
    )
    bird_speed = LaunchConfiguration('bird_speed')

    # use_kf:=false → KF 없이 raw centroid 사용 (baseline)
    declare_use_kf = DeclareLaunchArgument(
        'use_kf', default_value='true',
        description='false 로 설정하면 KF 없이 raw DBSCAN centroid 로 추적 (baseline)'
    )
    use_kf = LaunchConfiguration('use_kf')

    # kf_lookahead:=0.0/0.15/0.25 — KF lookahead 비교 실험용 (use_kf=true일 때만 유효)
    declare_kf_lookahead = DeclareLaunchArgument(
        'kf_lookahead', default_value='0.15',
        description='cluster_node KF lookahead 시간 (s). 0.0/0.15/0.25 비교 실험용'
    )
    kf_lookahead = LaunchConfiguration('kf_lookahead')

    # ── 1. PointCloud2 → LaserScan ──────────────────────────────────
    pcd_to_scan_node = Node(
        package='pcd_to_scan_pkg',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── 2. Nav2 + AMCL + RViz2 ──────────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_gazebo_dir, 'launch', 'nav', 'nav.launch.py')
        ),
        launch_arguments={
            'use_localization': 'amcl',
            'use_localplan':    'teb',
        }.items()
    )

    # ── 3. LiDAR 조류 클러스터 추적 ─────────────────────────────────
    cluster_node = Node(
        package='pcd_cluster_pkg',
        executable='cluster_node',
        name='cluster_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'use_kf': use_kf, 'kf_lookahead_sec': kf_lookahead}],
    )

    # ── 4. YOLOv8 시각 보조 (LiDAR 유실 시 폴백) ─────────────────────
    bird_yolo_node = Node(
        package='pcd_cluster_pkg',
        executable='bird_yolo_node',
        name='bird_yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_path':   _YOLO_MODEL,
        }],
    )

    # ── 5. 순찰/추적 상태 기계 (Nav2 준비 대기 5s) ────────────────────
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

    _setup = 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/ugv_ws/install/setup.bash'

    # ── 6. Bird Control 터미널 (10s 후) ──────────────────────────────
    bird_input_terminal = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--title=Bird Control', '--',
                    'bash', '-c',
                    f'{_setup} && ros2 run pcd_cluster_pkg bird_input_node; exec bash',
                ],
                output='screen',
            )
        ]
    )

    # ── 7. Robot Status 터미널 (10s 후) ──────────────────────────────
    status_terminal = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--title=Robot Status', '--',
                    'bash', '-c',
                    f'{_setup} && ros2 run pcd_cluster_pkg robot_status_monitor; exec bash',
                ],
                output='screen',
            )
        ]
    )

    # ── 6. 데이터 로거 ───────────────────────────────────────────────
    data_logger = Node(
        package='pcd_cluster_pkg',
        executable='data_logger_node',
        name='data_logger',
        output='screen',
        condition=IfCondition(log_data),
        parameters=[{'use_kf': use_kf, 'kf_lookahead_sec': kf_lookahead, 'bird_speed': bird_speed}],
    )

    return LaunchDescription([
        declare_log_data,
        declare_bird_speed,
        declare_use_kf,
        declare_kf_lookahead,
        pcd_to_scan_node,
        nav_launch,
        cluster_node,
        bird_yolo_node,
        patrol_node,
        bird_input_terminal,
        status_terminal,
        data_logger,
    ])
