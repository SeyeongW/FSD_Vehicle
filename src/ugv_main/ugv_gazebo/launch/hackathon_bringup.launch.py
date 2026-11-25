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

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
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
        description="추격 조향 방식: 'pn'(시선각 변화율) / 'pure_pursuit'(방위각 직접 겨냥) / 'smc'(슬라이딩 모드 제어)"
    )
    steering_mode = LaunchConfiguration('steering_mode')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='gzclient/rviz2/제어용 터미널(Bird Control, Robot Status) 실행 여부 — '
                     'headless 배치 실행 시 false (bird 제어는 /bird_command 토픽으로 직접 발행)'
    )
    gui = LaunchConfiguration('gui')

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

    declare_sta_k1 = DeclareLaunchArgument(
        'sta_k1', default_value='1.5',
        description='SMC(Super-Twisting) 비례항 게인 k1 — sqrt(|bearing|)에 곱해짐'
    )
    sta_k1 = LaunchConfiguration('sta_k1')

    declare_sta_k2 = DeclareLaunchArgument(
        'sta_k2', default_value='0.2',
        description='SMC(Super-Twisting) 적분(스위칭) 게인 k2'
    )
    sta_k2 = LaunchConfiguration('sta_k2')

    declare_sta_eps = DeclareLaunchArgument(
        'sta_eps', default_value='0.05',
        description='SMC(Super-Twisting) sqrt(|s|) 정칙화 항 — s=0 근처 과민반응 완화'
    )
    sta_eps = LaunchConfiguration('sta_eps')

    declare_bearing_deadband = DeclareLaunchArgument(
        'bearing_deadband', default_value='0.05',
        description='SMC(Super-Twisting) 정렬 데드밴드(rad) — 이 이내면 u1을 감쇠시켜 풀어줌'
    )
    bearing_deadband = LaunchConfiguration('bearing_deadband')

    declare_sta_u1_limit = DeclareLaunchArgument(
        'sta_u1_limit', default_value='1.0',
        description='SMC(Super-Twisting) 적분항 u1 자체 상한(rad/s) — 적분항 혼자 과도한 회전 명령을 못 내게 제한'
    )
    sta_u1_limit = LaunchConfiguration('sta_u1_limit')

    declare_sta_kd = DeclareLaunchArgument(
        'sta_kd', default_value='1.0',
        description='SMC(Super-Twisting) 실측 각속도(robot_angular_vel) 감쇠 게인 — 큰 회전 중 관성으로 인한 오버슈트 억제'
    )
    sta_kd = LaunchConfiguration('sta_kd')

    declare_flight_speed = DeclareLaunchArgument(
        'flight_speed', default_value='1.2',
        description='새 비행 속도 (m/s)'
    )
    flight_speed = LaunchConfiguration('flight_speed')

    declare_bird_max_turn_rate_deg = DeclareLaunchArgument(
        'bird_max_turn_rate_deg', default_value='120.0',
        description='새의 최대 선회율 (deg/s) — 웨이포인트 전환 시 반경 0 순간회전 대신 유한한 선회반경 적용'
    )
    bird_max_turn_rate_deg = LaunchConfiguration('bird_max_turn_rate_deg')

    declare_gimbal_model_path = DeclareLaunchArgument(
        'gimbal_model_path', default_value='yolo11l.pt',
        description='벽면 고정 짐벌 카메라 비전 추적에 쓰는 YOLO 가중치'
    )
    gimbal_model_path = LaunchConfiguration('gimbal_model_path')

    # ── 졸업논문 실험(thesis_eval_pkg)용 — 기본값은 기존 동작과 동일 ──
    declare_use_lidar_fusion = DeclareLaunchArgument(
        'use_lidar_fusion', default_value='true',
        description='false면 lidar_fusion_node를 띄우지 않음 (thesis_eval_pkg의 고장허용 융합 노드로 대체할 때)'
    )
    use_lidar_fusion = LaunchConfiguration('use_lidar_fusion')
    declare_bird_sdf_path = DeclareLaunchArgument(
        'bird_sdf_path', default_value='default', description="새 모델 SDF 경로 ('default'=ugv_gazebo/models/bird)"
    )
    bird_sdf_path = LaunchConfiguration('bird_sdf_path')
    declare_bird_random_seed = DeclareLaunchArgument(
        'bird_random_seed', default_value='-1', description='>=0이면 새 비행 궤적 재현'
    )
    bird_random_seed = LaunchConfiguration('bird_random_seed')
    declare_bird_status_rate_hz = DeclareLaunchArgument(
        'bird_status_rate_hz', default_value='1.0', description='새 GT(/bird/nearest_pose) 발행 주기'
    )
    bird_status_rate_hz = LaunchConfiguration('bird_status_rate_hz')

    # ── numpy/BLAS 스레드 제한 ──
    # OPENBLAS_NUM_THREADS 등이 안 잡혀 있으면 OpenBLAS가 작은 행렬곱/배열 연산에도
    # 코어 수(28)만큼 스레드를 뿌린다 — pointcloud_to_laserscan_node에서 CPU 2000%,
    # 스레드 27개 풀가동으로 실측 확인된 문제(순수 numpy 재현 스크립트로 검증됨).
    # 이 프로세스들은 어차피 메시지 하나씩 순차 처리라 병렬 BLAS로 얻을 게 없고,
    # 스레드 생성/경합 오버헤드만 커서 1로 고정하는 게 오히려 더 빠르다(재현 스크립트
    # 기준 867→1229 iter/8s).
    blas_threads_env = [
        SetEnvironmentVariable(name='OPENBLAS_NUM_THREADS', value='1'),
        SetEnvironmentVariable(name='OMP_NUM_THREADS', value='1'),
        SetEnvironmentVariable(name='MKL_NUM_THREADS', value='1'),
        SetEnvironmentVariable(name='NUMEXPR_NUM_THREADS', value='1'),
    ]

    # ── Gazebo 환경 ──
    # gimbal_camera_pkg의 짐벌 모델(벽면 고정 카메라, world 파일에 include됨)이
    # model:// URI(메시)와 커스텀 조인트 컨트롤러 플러그인을 못 찾지 않도록
    # 그 패키지의 models/lib 경로도 함께 추가한다.
    gimbal_pkg_share = get_package_share_directory('gimbal_camera_pkg')
    gimbal_pkg_lib = os.path.join(get_package_prefix('gimbal_camera_pkg'), 'lib', 'gimbal_camera_pkg')

    gazebo_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value=''
    )
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=(
            os.path.join(pkg_share, 'models') + ':' +
            os.path.join(gimbal_pkg_share, 'models') + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '')
        )
    )
    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=gimbal_pkg_lib + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')
    )

    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gzclient_cmd = ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(gui))

    # 사용자가 RViz2에서 File -> Save Config As...로 저장해둔 디스플레이 구성을
    # 매번 자동으로 불러온다 (Add로 매번 새로 추가하는 번거로움을 없앰).
    rviz_config = os.path.join(pkg_share, 'rviz', 'hackathon_view.rviz')
    rviz_cmd = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(gui),
    )

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
            # 고정 LiDAR 2대는 각 8천, 온보드(mid360)는 1.5만 — input_topics 순서와 대응.
            # lidar_fusion_node의 publish() 비용이 메시지 크기(포인트 수)에 거의
            # 비례해서(프로파일링 실측: mid360 4만 포인트일 때 pub()만 66~100ms) CPU를
            # 과도하게 먹던 문제 완화 목적으로 기존 20000/20000/40000에서 하향.
            'max_points_per_source': [8000, 8000, 15000],
            'bounds_xy': 11.0,          # 20x20 방 기준 (기존 15x15용 기본값 8.0에서 조정)
            'use_sim_time': True,
        }],
        condition=IfCondition(use_lidar_fusion),
    )

    pcd_to_scan_node = Node(
        package='pcd_to_scan_pkg', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node', output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/fused_pointcloud2',
            'target_frame': 'odom',
            # skid_steer_drive_controller는 odom->base_footprint를 발행하는데
            # (odom->base_link 아님), SDF에 base_footprint와 base_link를 잇는
            # 조인트가 없어서 scan_frame 기본값('base_link')으로는 TF 조회가
            # 영원히 실패했다("Lookup would require extrapolation into the
            # future" — 사실 미래가 아니라 애초에 그 프레임 자체가 안 옴).
            # 실제 발행되는 프레임에 맞춰 base_footprint로 지정.
            'scan_frame': 'base_footprint',
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
            'sta_k1': sta_k1,
            'sta_k2': sta_k2,
            'sta_eps': sta_eps,
            'bearing_deadband': bearing_deadband,
            'sta_u1_limit': sta_u1_limit,
            'sta_kd': sta_kd,
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
            cmd=['python3', bird_manager_py, '--ros-args',
                 '-p', ['flight_speed:=', flight_speed],
                 '-p', ['bird_max_turn_rate_deg:=', bird_max_turn_rate_deg],
                 '-p', ['bird_sdf_path:=', bird_sdf_path],
                 '-p', ['random_seed:=', bird_random_seed],
                 '-p', ['status_rate_hz:=', bird_status_rate_hz]],
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
        )]
    )

    # 벽면 고정 짐벌 카메라: /bird_target(LiDAR)으로 탐색 -> YOLO로 정밀 추적.
    # camera_x/y/z, mount_yaw_offset 기본값이 world 파일의 camera_tower/
    # gimbal_classic include와 일치해야 한다 (둘 다 바꾸면 같이 바꿀 것).
    gimbal_tracker_py = os.path.join(
        get_package_prefix('gimbal_camera_pkg'), 'lib', 'gimbal_camera_pkg', 'gimbal_bird_tracker.py')
    run_gimbal_tracker_cmd = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['python3', gimbal_tracker_py, '--ros-args',
                 '-p', ['model_path:=', gimbal_model_path]],
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
        )],
        condition=IfCondition(gui),
    )
    status_terminal = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['gnome-terminal', '--title=Robot Status', '--', 'bash', '-c',
                 f'{_setup} && ros2 run pcd_cluster_pkg robot_status_monitor; exec bash'],
            output='screen',
        )],
        condition=IfCondition(gui),
    )

    ld = LaunchDescription()
    for env_action in blas_threads_env:
        ld.add_action(env_action)
    ld.add_action(declare_steering_mode)
    ld.add_action(declare_gui)
    ld.add_action(declare_pn_gain)
    ld.add_action(declare_pn_bearing_bias_gain)
    ld.add_action(declare_pp_angular_gain)
    ld.add_action(declare_sta_k1)
    ld.add_action(declare_sta_k2)
    ld.add_action(declare_sta_eps)
    ld.add_action(declare_bearing_deadband)
    ld.add_action(declare_sta_u1_limit)
    ld.add_action(declare_sta_kd)
    ld.add_action(declare_flight_speed)
    ld.add_action(declare_bird_max_turn_rate_deg)
    ld.add_action(declare_gimbal_model_path)
    ld.add_action(declare_use_lidar_fusion)
    ld.add_action(declare_bird_sdf_path)
    ld.add_action(declare_bird_random_seed)
    ld.add_action(declare_bird_status_rate_hz)
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gazebo_plugin_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(rviz_cmd)
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
    ld.add_action(run_gimbal_tracker_cmd)
    ld.add_action(bird_input_terminal)
    ld.add_action(status_terminal)

    return ld
