"""
hackathon_bringup.launch.py — 3-LiDAR 융합 + 검출/추적 + PN 추격 + 순찰 통합 런치

[구성]
  - gzserver/gzclient + ugv_world_3lidar.world (20x20)
  - robot_state_publisher + ugv_rover 스폰
  - 고정 LiDAR 2개 static TF (odom 프레임에 직접 부착 — map은 AMCL이 /scan을 받아야
    map->odom을 발행하는데 그 /scan을 이 파이프라인이 만들어야 해서 순환 의존을 피함)
  - lidar_fusion_node        (3-LiDAR -> /fused_pointcloud2, odom 프레임)
  - pointcloud_to_laserscan_node (/fused_pointcloud2 -> /scan, AMCL/장애물회피용)
  - AMCL + map_server        (map_3lidar_20x20, 순찰 웨이포인트 map 프레임 위치 확인용 — Nav2
                              순찰 스택(TEB/코스트맵/BT)은 안 씀, patrol_node가 직접 조향)
  - bird_detector_node       (DBSCAN + KF, odom 프레임 검출/추적)
  - bird_pursuit_node        (PN/Pure Pursuit 추격 제어, steering_mode로 선택)
  - trajectory_viz_node      (새 (x,y) 궤적 vs 로봇 실주행 궤적 -> /bird_track_path, /robot_track_path)
  - patrol_node              (PATROL<->TRACKING 상태기계 + atan2 기반 순찰 직접 조향, 5초 지연)
  - bird_manager.py          (랜덤 방향전환 비행, 8.5초 지연)
  - bird_input_node / robot_status_monitor 터미널 (10초 지연)

[실행]
  export UGV_MODEL=ugv_rover
  ros2 launch ugv_gazebo hackathon_bringup.launch.py

[조류 제어]
  Bird Control 터미널에서 bird_in / bird_out / q
"""

import glob as _glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_script(filename: str, pkg_share: str) -> str:
    candidate = os.path.normpath(
        os.path.join(pkg_share, '..', '..', 'lib', 'ugv_gazebo', filename)
    )
    if os.path.isfile(candidate):
        return candidate
    ws_src = os.path.expanduser('~/ros2_ws/ugv_ws/src')
    matches = _glob.glob(os.path.join(ws_src, '**', filename), recursive=True)
    if matches:
        return matches[0]
    return candidate


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_gazebo')
    launch_file_dir = os.path.join(pkg_share, 'launch', 'bringup')
    world = os.path.join(pkg_share, 'worlds', 'ugv_world_3lidar.world')
    map_yaml = os.path.join(pkg_share, 'maps', 'map_3lidar_20x20.yaml')
    bird_manager_py = _find_script('bird_manager.py', pkg_share)

    # ── 튜닝용 launch argument ──
    declare_steering_mode = DeclareLaunchArgument(
        'steering_mode', default_value='pn',
        description="추격 조향 방식: 'pn'(시선각 변화율) 또는 'pure_pursuit'(방위각 직접 겨냥)"
    )
    steering_mode = LaunchConfiguration('steering_mode')

    declare_pn_gain = DeclareLaunchArgument(
        'pn_gain', default_value='3.5',
        description='PN(비례 항법) 게인. 클수록 반응이 빠르지만 진동 위험'
    )
    pn_gain = LaunchConfiguration('pn_gain')

    declare_pn_bearing_bias_gain = DeclareLaunchArgument(
        'pn_bearing_bias_gain', default_value='0.6',
        description='PN에 섞는 방위각 비례 최소 조향 성분 — LOS_rate가 0이어도 돌기 시작하게 함'
    )
    pn_bearing_bias_gain = LaunchConfiguration('pn_bearing_bias_gain')

    declare_pp_angular_gain = DeclareLaunchArgument(
        'pp_angular_gain', default_value='2.0',
        description='Pure Pursuit 각속도 게인 (방위각 오차에 비례)'
    )
    pp_angular_gain = LaunchConfiguration('pp_angular_gain')

    declare_flight_speed = DeclareLaunchArgument(
        'flight_speed', default_value='1.2',
        description='새 비행 속도 (m/s)'
    )
    flight_speed = LaunchConfiguration('flight_speed')

    # ── Gazebo 환경 ──
    gazebo_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value=''
    )
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gzclient_cmd = ExecuteProcess(cmd=['gzclient'], output='screen')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py'))
    )
    spawn_ugv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_ugv.launch.py'))
    )

    # odom -> fixed_lidar poses (ugv_world_3lidar.world의 <pose>와 동일 좌표계 기준).
    # 'map'이 아니라 'odom'에 붙인다: map->odom은 AMCL이 /scan을 받아야만 발행하는데,
    # /scan은 이 융합 파이프라인(pointcloud_to_laserscan_node)이 만들어야 해서 target_frame을
    # map으로 하면 순환 의존성(닭-달걀 문제)에 빠진다. odom->base_link는 AMCL 없이도
    # skid_steer_drive_controller+robot_state_publisher가 즉시 발행하므로 안전하다.
    # 새 검출/추격은 로봇 기준 로컬 동작이라 전역 map 프레임을 몰라도 되고, Nav2 순찰
    # (map 프레임 웨이포인트)만 AMCL의 map->odom을 필요로 한다 — 서로 독립적.
    static_tf_lidar1 = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_odom_lidar1',
        arguments=['-9.3333', '0', '1.8', '0', '0', '0', 'odom', 'lidar1'],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_lidar2 = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_odom_lidar2',
        arguments=['9.3333', '0', '1.8', '3.14159', '0', '0', 'odom', 'lidar2'],
        parameters=[{'use_sim_time': True}],
    )

    lidar_fusion_node = Node(
        package='lidar_fusion_pkg', executable='lidar_fusion_node', name='lidar_fusion_node',
        output='screen',
        parameters=[{
            'input_topics': ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'mid360_PointCloud2'],
            'target_frame': 'odom',
            'output_topic': 'fused_pointcloud2',
            # 고정 LiDAR 2대는 각 2만, 온보드(mid360)는 4만 — input_topics 순서와 대응
            'max_points_per_source': [20000, 20000, 40000],
            'bounds_xy': 11.0,          # 20x20 방 기준 (기존 15x15용 기본값 8.0에서 조정)
            'use_sim_time': True,
        }],
    )

    pcd_to_scan_node = Node(
        package='pcd_to_scan_pkg', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/fused_pointcloud2',
            'target_frame': 'odom',
        }],
    )

    # 순찰 주행을 patrol_node의 atan2 직접 조향으로 바꾸면서 TEB/코스트맵/BT 등
    # Nav2 순찰 스택 전체가 필요 없어졌다 (정적인 방+미리 아는 웨이포인트라 Nav2의
    # 동적 장애물 회피가 쓸모없고, 오히려 RotationShimController/velocity_smoother
    # 등 여러 레이어가 서로 값을 clamp하며 튜닝이 계속 꼬였음). map 프레임 위치를
    # 얻기 위한 AMCL+map_server만 nav2_bringup의 localization_launch.py로 남긴다.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': os.path.join(pkg_share, 'param', 'amcl_teb.yaml'),
            'use_sim_time': 'True',
            'autostart': 'true',
        }.items()
    )

    bird_detector_node = Node(
        package='pcd_cluster_pkg', executable='bird_detector_node', name='bird_detector_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )
    bird_pursuit_node = Node(
        package='pcd_cluster_pkg', executable='bird_pursuit_node', name='bird_pursuit_node',
        output='screen', parameters=[{
            'use_sim_time': True,
            'steering_mode': steering_mode,
            'pn_gain': pn_gain,
            'pn_bearing_bias_gain': pn_bearing_bias_gain,
            'pp_angular_gain': pp_angular_gain,
        }],
    )

    trajectory_viz_node = Node(
        package='pcd_cluster_pkg', executable='trajectory_viz_node', name='trajectory_viz_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    patrol_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='pcd_cluster_pkg', executable='patrol_node', name='patrol_node',
            output='screen', parameters=[{'use_sim_time': True}],
        )]
    )

    run_bird_manager_cmd = TimerAction(
        period=8.5,
        actions=[ExecuteProcess(
            cmd=['python3', bird_manager_py, '--ros-args', '-p', ['flight_speed:=', flight_speed]],
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        )]
    )

    _setup = 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/ugv_ws/install/setup.bash'
    bird_input_terminal = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['gnome-terminal', '--title=Bird Control', '--', 'bash', '-c',
                 f'{_setup} && ros2 run pcd_cluster_pkg bird_input_node; exec bash'],
            output='screen',
        )]
    )
    status_terminal = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['gnome-terminal', '--title=Robot Status', '--', 'bash', '-c',
                 f'{_setup} && ros2 run pcd_cluster_pkg robot_status_monitor; exec bash'],
            output='screen',
        )]
    )

    ld = LaunchDescription()
    ld.add_action(declare_steering_mode)
    ld.add_action(declare_pn_gain)
    ld.add_action(declare_pn_bearing_bias_gain)
    ld.add_action(declare_pp_angular_gain)
    ld.add_action(declare_flight_speed)
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_cmd)
    ld.add_action(static_tf_lidar1)
    ld.add_action(static_tf_lidar2)
    ld.add_action(lidar_fusion_node)
    ld.add_action(pcd_to_scan_node)
    ld.add_action(localization_launch)
    ld.add_action(bird_detector_node)
    ld.add_action(bird_pursuit_node)
    ld.add_action(trajectory_viz_node)
    ld.add_action(patrol_node)
    ld.add_action(run_bird_manager_cmd)
    ld.add_action(bird_input_terminal)
    ld.add_action(status_terminal)

    return ld
