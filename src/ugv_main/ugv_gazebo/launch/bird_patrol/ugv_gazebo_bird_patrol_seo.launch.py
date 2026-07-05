from __future__ import annotations

import os
import json
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_prefix(package_name: str) -> str:
    try:
        return get_package_prefix(package_name)
    except Exception:
        return ""


def _resolve_world(world_arg: str, ugv_share: str) -> str:
    expanded = os.path.expanduser(os.path.expandvars(world_arg))
    if os.path.isabs(expanded):
        return expanded
    candidate = os.path.join(ugv_share, "worlds", expanded)
    if os.path.exists(candidate):
        return candidate
    return os.path.join(ugv_share, "worlds", "ugv_world.world")


def _launch_setup(context, *args, **kwargs):
    ugv_share = get_package_share_directory("ugv_gazebo")
    ugv_description_share = get_package_share_directory("ugv_description")
    ugv_description_parent = os.path.dirname(ugv_description_share)
    gazebo_ros_prefix = _package_prefix("gazebo_ros")
    gazebo_ros_lib = os.path.join(gazebo_ros_prefix, "lib") if gazebo_ros_prefix else ""

    ws_root = os.path.expanduser("~/ros2_ws5/FSD_Vehicle")
    livox_prefix = _package_prefix("ros2_livox_simulation")
    livox_plugin_candidates = [
        os.path.join(livox_prefix, "lib") if livox_prefix else "",
        os.path.join(ws_root, "install", "ros2_livox_simulation", "lib"),
        os.path.join(ws_root, "build", "ros2_livox_simulation"),
    ]
    gazebo_plugin_path = ":".join(
        p for p in [gazebo_ros_lib, *livox_plugin_candidates, os.environ.get("GAZEBO_PLUGIN_PATH", "")] if p
    )
    gazebo_model_path = ":".join(
        p
        for p in [
            os.path.join(ugv_share, "models"),
            ugv_description_parent,
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ]
        if p
    )
    gazebo_resource_path = ":".join(
        p
        for p in [
            os.path.join(ugv_share, "worlds"),
            os.path.join(ugv_share, "models"),
            "/usr/share/gazebo-11",
            "/usr/share/gazebo",
            os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        ]
        if p
    )

    world = _resolve_world(LaunchConfiguration("world").perform(context), ugv_share)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_sdf = os.path.join(ugv_share, "models", robot_model, "model.sdf")
    robot_urdf = os.path.join(ugv_share, "urdf", f"{robot_model}.urdf")
    config_dir = os.path.join(ugv_share, "param", "bird_patrol")
    mission_params = os.path.join(config_dir, "ugv_gazebo_bird_patrol_seo.yaml")
    tracking_params = os.path.join(config_dir, "seo_tracking_gazebo.yaml")
    logger_params = os.path.join(config_dir, "experiment_logging.yaml")
    inside_waypoints = os.path.join(config_dir, "patrol_waypoints_inside_15m.yaml")
    paper_waypoints = os.path.join(config_dir, "patrol_waypoints_square_8m_paper.yaml")
    legacy_waypoints = os.path.join(config_dir, "patrol_waypoints_square_4m_7m.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    enable_fake_bird_classifier = LaunchConfiguration("enable_fake_bird_classifier")
    enable_trial_logger = LaunchConfiguration("enable_trial_logger")
    enable_dataset_logger = LaunchConfiguration("enable_dataset_logger")
    enable_spatial_response_logger = LaunchConfiguration("enable_spatial_response_logger")
    enable_spatial_debug_viz = LaunchConfiguration("enable_spatial_debug_viz")
    enable_rosbag_record = LaunchConfiguration("enable_rosbag_record")
    enable_rviz = LaunchConfiguration("enable_rviz")
    use_gui = LaunchConfiguration("use_gui")
    rviz_config_arg = LaunchConfiguration("rviz_config").perform(context)
    detector_mode_value = LaunchConfiguration("detector_mode").perform(context).strip().lower()
    classifier_mode_value = LaunchConfiguration("classifier_mode").perform(context).strip().lower()
    random_seed_value = LaunchConfiguration("random_seed").perform(context)
    trial_id_value = LaunchConfiguration("trial_id").perform(context)
    run_id_arg = LaunchConfiguration("run_id").perform(context).strip()
    run_dir_arg = os.path.expanduser(os.path.expandvars(LaunchConfiguration("run_dir").perform(context).strip()))
    active_birds_value = LaunchConfiguration("active_birds").perform(context)
    output_root = os.path.expanduser(os.path.expandvars(LaunchConfiguration("output_root").perform(context)))
    run_id = run_id_arg or f"{trial_id_value}_{time.strftime('%Y%m%d_%H%M%S')}"
    run_dir = run_dir_arg or os.path.join(output_root, run_id)
    os.makedirs(os.path.join(run_dir, "bags"), exist_ok=True)
    use_inside_waypoints = LaunchConfiguration("use_inside_15m_waypoints").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    use_legacy_waypoints = LaunchConfiguration("use_legacy_4m_7m_waypoints").perform(context).strip().lower() in {"1", "true", "yes", "on"}
    waypoint_file = inside_waypoints if use_inside_waypoints else (legacy_waypoints if use_legacy_waypoints else paper_waypoints)
    inspection_goal_offset = float(LaunchConfiguration("inspection_goal_offset_distance_m").perform(context))
    sim_nav_max_linear_speed = float(LaunchConfiguration("sim_nav_max_linear_speed").perform(context))
    sim_nav_max_angular_speed = float(LaunchConfiguration("sim_nav_max_angular_speed").perform(context))
    rviz_config = os.path.expanduser(os.path.expandvars(rviz_config_arg))
    if not os.path.isabs(rviz_config):
        rviz_config = os.path.join(ugv_share, "rviz", rviz_config)
    use_gt_fallback = detector_mode_value in {"ground_truth", "fused"}
    launch_args_yaml = json.dumps(
        {
            "world": LaunchConfiguration("world").perform(context),
            "robot_model": robot_model,
            "experiment_profile": LaunchConfiguration("experiment_profile").perform(context),
            "detector_mode": detector_mode_value,
            "classifier_mode": classifier_mode_value,
            "random_seed": random_seed_value,
            "trial_id": trial_id_value,
            "run_id": run_id,
            "active_birds": active_birds_value,
            "bird_release_interval_sec": LaunchConfiguration("bird_release_interval_sec").perform(context),
            "bird_stable_demo_spawn": LaunchConfiguration("bird_stable_demo_spawn").perform(context),
            "bird_removal_detection_hold_sec": LaunchConfiguration("bird_removal_detection_hold_sec").perform(context),
            "detection_hold_grace_sec": LaunchConfiguration("detection_hold_grace_sec").perform(context),
            "bird_removal_goal_count": LaunchConfiguration("bird_removal_goal_count").perform(context),
            "use_inside_15m_waypoints": use_inside_waypoints,
            "waypoint_file": waypoint_file,
            "inspection_goal_offset_distance_m": inspection_goal_offset,
            "sim_nav_max_linear_speed": sim_nav_max_linear_speed,
            "sim_nav_max_angular_speed": sim_nav_max_angular_speed,
        }
    )

    common_params = [{"use_sim_time": use_sim_time}]
    mission_common = [
        mission_params,
        {
            "waypoint_file": waypoint_file,
            "inspection_goal_offset_distance_m": inspection_goal_offset,
            "goal_offset_distance_m": inspection_goal_offset,
            "max_linear_speed": sim_nav_max_linear_speed,
            "max_angular_speed": sim_nav_max_angular_speed,
        },
        *common_params,
    ]
    tracking_common = [
        tracking_params,
        {
            "detector_mode": detector_mode_value,
            "use_gazebo_bird_pose_fallback": use_gt_fallback,
            "center_tolerance_rad": 0.15,
            "alignment_mode": "logical",
            "publish_joint_trajectory": False,
            "target_topic": "/waver/camera_aim_target_pose",
            "base_frame": "base_footprint",
            "angular_gain": 1.6,
            "max_angular_speed": 0.55,
            "min_turn_speed": 0.24,
            "allowed_target_x_min_m": -5.0,
            "allowed_target_x_max_m": 5.0,
            "allowed_target_y_min_m": -5.0,
            "allowed_target_y_max_m": 5.0,
        },
        *common_params,
    ]

    return [
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", gazebo_resource_path),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
        ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                world,
                "-s",
                "libgazebo_ros_init.so",
                "-s",
                "libgazebo_ros_factory.so",
            ],
            output="screen",
        ),
        ExecuteProcess(cmd=["gzclient"], output="screen", condition=IfCondition(use_gui)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            arguments=[robot_urdf],
            parameters=common_params,
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_ugv_rover",
                    arguments=[
                        "-entity",
                        robot_model,
                        "-file",
                        robot_sdf,
                        "-x",
                        LaunchConfiguration("robot_spawn_x"),
                        "-y",
                        LaunchConfiguration("robot_spawn_y"),
                        "-z",
                        LaunchConfiguration("robot_spawn_z"),
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="ugv_gazebo",
                    executable="bird_manager.py",
                    name="bird_manager",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "active_birds": active_birds_value,
                            "publish_waver_detection_topics": False,
                            "z_min_m": 3.1,
                            "z_max_m": 3.4,
                            "min_speed_mps": 0.08,
                            "max_speed_mps": 0.22,
                            "random_seed": int(random_seed_value),
                            "sequential_release_birds": LaunchConfiguration("sequential_release_birds"),
                            "release_interval_sec": LaunchConfiguration("bird_release_interval_sec"),
                            "enable_remove_after_detection": LaunchConfiguration("enable_bird_removal_after_detection"),
                            "remove_after_detected_sec": LaunchConfiguration("bird_removal_detection_hold_sec"),
                            "detection_hold_grace_sec": LaunchConfiguration("detection_hold_grace_sec"),
                            "require_sound_done_for_removal": LaunchConfiguration("require_sound_done_for_removal"),
                            "max_removed_birds": LaunchConfiguration("bird_removal_goal_count"),
                            "enable_flee_after_sound": True,
                            "flee_distance_m": LaunchConfiguration("bird_flee_distance_m"),
                            "return_distance_m": LaunchConfiguration("bird_return_distance_m"),
                            "flee_speed_mps": LaunchConfiguration("bird_flee_speed_mps"),
                            "allowed_lidar_x_min_m": -5.0,
                            "allowed_lidar_x_max_m": 5.0,
                            "allowed_lidar_y_min_m": -5.0,
                            "allowed_lidar_y_max_m": 5.0,
                            "stable_demo_spawn": LaunchConfiguration("bird_stable_demo_spawn"),
                            "random_spawn_inside_lidar_area": True,
                        }
                    ],
                )
            ],
        ),
        TimerAction(
            period=4.5,
            actions=[
                Node(
                    package="waver_seo_tracking",
                    executable="scan_alias_node",
                    name="scan_alias_node",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time, "input_topic": "/scan", "slam_topic": "/scan_slam", "safety_topic": "/scan_safety"}],
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="seo_cluster_tracker_node",
                    name="seo_cluster_tracker_node",
                    output="screen",
                    parameters=tracking_common,
                ),
                Node(
                    package="waver_patrol",
                    executable="target_goal_manager_node",
                    name="target_goal_manager_node",
                    output="screen",
                    parameters=mission_common,
                ),
                Node(
                    package="waver_patrol",
                    executable="mission_patrol_manager_node",
                    name="mission_patrol_manager_node",
                    output="screen",
                    parameters=mission_common,
                ),
                Node(
                    package="waver_patrol",
                    executable="simple_nav2_cmd_sim_node",
                    name="simple_nav2_cmd_sim_node",
                    output="screen",
                    parameters=mission_common,
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="seo_observation_body_tracker_node",
                    name="seo_observation_body_tracker_node",
                    output="screen",
                    parameters=tracking_common,
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="mission_state_cmd_selector_node",
                    name="mission_state_cmd_selector_node",
                    output="screen",
                    parameters=tracking_common,
                ),
                Node(
                    package="waver_patrol",
                    executable="safety_cmd_mux_node",
                    name="safety_cmd_mux_node",
                    output="screen",
                    parameters=mission_common,
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="seo_camera_tilt_joint_node",
                    name="seo_camera_tilt_joint_node",
                    output="screen",
                    parameters=tracking_common,
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="seo_bird_yolo_node",
                    name="seo_bird_yolo_node",
                    output="screen",
                    parameters=tracking_common,
                    condition=IfCondition(enable_fake_bird_classifier),
                ),
                Node(
                    package="waver_patrol",
                    executable="sound_alert_stub",
                    name="sound_alert_stub",
                    output="screen",
                    parameters=mission_common,
                ),
                Node(
                    package="waver_seo_tracking",
                    executable="remote_ui_patrol_adapter_node",
                    name="remote_ui_patrol_adapter_node",
                    output="screen",
                    parameters=tracking_common,
                ),
                Node(
                    package="waver_experiment_logger",
                    executable="seo_mechanism_trial_logger_node",
                    name="seo_mechanism_trial_logger_node",
                    output="screen",
                    parameters=[
                        logger_params,
                        {"output_root": output_root, "trial_id": trial_id_value, "run_id": run_id, "run_dir": run_dir},
                        *common_params,
                    ],
                    condition=IfCondition(enable_trial_logger),
                ),
                Node(
                    package="waver_experiment_logger",
                    executable="gazebo_bird_dataset_logger_node",
                    name="gazebo_bird_dataset_logger_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "output_root": output_root,
                            "trial_id": trial_id_value,
                            "experiment_id": LaunchConfiguration("experiment_id"),
                            "run_id": run_id,
                            "run_dir": run_dir,
                            "detector_mode": detector_mode_value,
                            "classifier_mode": classifier_mode_value,
                            "random_seed": int(random_seed_value),
                            "save_images": LaunchConfiguration("save_images"),
                            "save_every_nth_image": LaunchConfiguration("save_every_nth_image"),
                            "image_topic": LaunchConfiguration("camera_image_topic"),
                            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                            "write_coco": LaunchConfiguration("write_coco"),
                            "write_yolo": LaunchConfiguration("write_yolo"),
                            "bird_removal_state_topic": "/waver/gazebo_bird_removal_state",
                            "launch_args_yaml": launch_args_yaml,
                        }
                    ],
                    condition=IfCondition(enable_dataset_logger),
                ),
                Node(
                    package="waver_experiment_logger",
                    executable="waver_spatial_response_logger_node",
                    name="waver_spatial_response_logger_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "output_root": output_root,
                            "trial_id": trial_id_value,
                            "run_id": run_id,
                            "run_dir": run_dir,
                            "detector_mode": detector_mode_value,
                            "configured_offset_m": inspection_goal_offset,
                            "sample_period_sec": 1.0,
                        }
                    ],
                    condition=IfCondition(enable_spatial_response_logger),
                ),
                Node(
                    package="waver_experiment_logger",
                    executable="waver_spatial_debug_viz_node",
                    name="waver_spatial_debug_viz_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "frame_id": "odom",
                            "configured_offset_m": inspection_goal_offset,
                        }
                    ],
                    condition=IfCondition(enable_spatial_debug_viz),
                ),
                Node(
                    package="waver_patrol",
                    executable="gazebo_trial_logger_node",
                    name="gazebo_trial_logger_node",
                    output="screen",
                    parameters=[
                        logger_params,
                        {"output_root": output_root, "trial_id": 1},
                        *common_params,
                    ],
                    condition=IfCondition(enable_trial_logger),
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                "ros2",
                                "bag",
                                "record",
                                "-o",
                                os.path.join(run_dir, "bags", "trial_rosbag"),
                                "/clock",
                                "/odom",
                                "/cmd_vel",
                                "/bird/nearest_pose",
                                "/mid360_PointCloud2",
                                "/waver/elevated_dynamic_targets",
                                "/waver/lidar_target_pose_odom",
                                "/waver/lidar_tracking_state",
                                "/waver/lidar_filter_response",
                                "/waver/object_mission_goal",
                                "/waver/object_mission_goal_debug",
                                "/waver/active_nav_goal",
                                "/waver/active_nav_goal_meta",
                                "/waver/sim_nav2_debug",
                                "/waver/spatial_debug_markers",
                                "/waver/gazebo_bird_kinematics",
                                "/waver/mission_state",
                                "/waver/mission_event",
                                "/filtered_points",
                                "/cluster_markers",
                                LaunchConfiguration("camera_image_topic"),
                                LaunchConfiguration("camera_info_topic"),
                            ],
                            output="screen",
                            condition=IfCondition(enable_rosbag_record),
                        )
                    ],
                ),
                Node(
                    package="ugv_tools",
                    executable="waver_remote_panel",
                    name="waver_remote_panel",
                    output="screen",
                    parameters=[
                        mission_params,
                        *common_params,
                        {"demo_script": LaunchConfiguration("remote_panel_demo_script")},
                    ],
                    condition=IfCondition(start_remote_panel),
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="waver_spatial_response_rviz",
                    arguments=["-d", rviz_config],
                    output="screen",
                    parameters=common_params,
                    condition=IfCondition(enable_rviz),
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("start_remote_panel", default_value="true"),
            DeclareLaunchArgument("remote_panel_demo_script", default_value=""),
            DeclareLaunchArgument("enable_fake_bird_classifier", default_value="true"),
            DeclareLaunchArgument("enable_trial_logger", default_value="true"),
            DeclareLaunchArgument("enable_dataset_logger", default_value="true"),
            DeclareLaunchArgument("enable_spatial_response_logger", default_value="true"),
            DeclareLaunchArgument("enable_spatial_debug_viz", default_value="true"),
            DeclareLaunchArgument("enable_rosbag_record", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value="waver_spatial_response_debug.rviz"),
            DeclareLaunchArgument("inspection_goal_offset_distance_m", default_value="2.0"),
            DeclareLaunchArgument("sim_nav_max_linear_speed", default_value="0.35"),
            DeclareLaunchArgument("sim_nav_max_angular_speed", default_value="0.55"),
            DeclareLaunchArgument("experiment_profile", default_value="spatial_lidar_response_smoke"),
            DeclareLaunchArgument("detector_mode", default_value="lidar"),
            DeclareLaunchArgument("classifier_mode", default_value="fake_gazebo"),
            DeclareLaunchArgument("random_seed", default_value="530"),
            DeclareLaunchArgument("active_birds", default_value="bird_1,bird_2,bird_3,bird_4,bird_5"),
            DeclareLaunchArgument("sequential_release_birds", default_value="true"),
            DeclareLaunchArgument("bird_release_interval_sec", default_value="7.0"),
            DeclareLaunchArgument("bird_stable_demo_spawn", default_value="false"),
            DeclareLaunchArgument("enable_bird_removal_after_detection", default_value="true"),
            DeclareLaunchArgument("bird_removal_detection_hold_sec", default_value="5.0"),
            DeclareLaunchArgument("detection_hold_grace_sec", default_value="3.0"),
            DeclareLaunchArgument("require_sound_done_for_removal", default_value="true"),
            DeclareLaunchArgument("bird_flee_distance_m", default_value="10.0"),
            DeclareLaunchArgument("bird_return_distance_m", default_value="10.0"),
            DeclareLaunchArgument("bird_flee_speed_mps", default_value="1.5"),
            DeclareLaunchArgument("bird_removal_goal_count", default_value="5"),
            DeclareLaunchArgument("trial_id", default_value="gazebo_seo_bird_patrol"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("run_dir", default_value=""),
            DeclareLaunchArgument("experiment_id", default_value="waver_gazebo_bird_patrol"),
            DeclareLaunchArgument("output_root", default_value=os.path.expanduser("~/ros2_ws5/FSD_Vehicle/experiment_results/gazebo_bird_patrol")),
            DeclareLaunchArgument("save_images", default_value="true"),
            DeclareLaunchArgument("save_every_nth_image", default_value="5"),
            DeclareLaunchArgument("write_coco", default_value="true"),
            DeclareLaunchArgument("write_yolo", default_value="true"),
            DeclareLaunchArgument("use_inside_15m_waypoints", default_value="false"),
            DeclareLaunchArgument("use_legacy_4m_7m_waypoints", default_value="false"),
            DeclareLaunchArgument("camera_image_topic", default_value="/pt_camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/pt_camera/camera_info"),
            DeclareLaunchArgument("world", default_value="ugv_world.world"),
            DeclareLaunchArgument("robot_model", default_value="ugv_rover"),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
