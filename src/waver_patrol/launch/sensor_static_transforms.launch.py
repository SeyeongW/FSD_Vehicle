from __future__ import annotations

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_transform(context, *args, **kwargs):
    path = os.path.expanduser(str(LaunchConfiguration("camera_lidar_extrinsic").perform(context)))
    if not os.path.exists(path):
        raise RuntimeError(f"camera-LiDAR extrinsic file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    camera_frame = str(data.get("camera_frame", "camera_color_optical_frame"))
    lidar_frame = str(data.get("lidar_frame", "livox"))
    xyz = list(data.get("translation_xyz", [0.0, 0.0, 0.0]))
    rpy = list(data.get("rotation_rpy", [0.0, 0.0, 0.0]))
    if len(xyz) != 3 or len(rpy) != 3:
        raise RuntimeError(f"invalid extrinsic vector lengths in {path}")
    calibrated = bool(data.get("calibrated", False))
    roll, pitch, yaw = (str(float(v)) for v in rpy)
    args = [str(float(xyz[0])), str(float(xyz[1])), str(float(xyz[2])), yaw, pitch, roll, lidar_frame, camera_frame]
    actions = [
        LogInfo(msg=f"[sensor_static_transforms] camera_lidar_extrinsic={path} calibrated={calibrated}"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_lidar_static_transform_publisher",
            output="screen",
            arguments=args,
        ),
    ]
    if not calibrated:
        actions.append(
            LogInfo(
                msg="[sensor_static_transforms] calibrated=false: production fusion must publish FUSION_INVALID_CALIBRATION_NOT_VERIFIED"
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    default_path = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/config/sensors/camera_lidar_extrinsic.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_lidar_extrinsic", default_value=default_path),
            OpaqueFunction(function=_load_transform),
        ]
    )
