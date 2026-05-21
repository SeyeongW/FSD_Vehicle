from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping


DEFAULT_CONFIG: dict[str, Any] = {
    "serial": {
        "preferred_ports": ["/dev/ttyTHS0", "/dev/serial0"],
        "glob_ports": ["/dev/ttyUSB*", "/dev/ttyACM*"],
        "baudrate": 115200,
        "command_rate_hz": 20,
        "reconnect_interval_s": 1.0,
        "write_timeout_s": 0.05,
    },
    "http": {"enabled": False, "base_ip": "192.168.4.1", "timeout_s": 0.3},
    "rover": {
        "max_left_right": 0.5,
        "default_speed": 0.16,
        "max_demo_speed": 0.28,
        "max_patrol_speed": 0.20,
        "max_mapping_speed": 0.16,
        "turn_gain": 0.65,
        "stop_repeat": 5,
    },
    "motor": {"invert_left": False, "invert_right": False, "swap_left_right": False},
    "keyboard": {"input_timeout_s": 0.3, "manual_override_timeout_s": 1.0},
    "safety": {
        "hard_stop_distance_m": 0.45,
        "slow_down_distance_m": 1.2,
        "clear_distance_m": 1.8,
        "ttc_hard_stop_s": 1.5,
        "ttc_slow_down_s": 3.0,
        "max_cmd_delta_per_tick": 0.04,
        "require_neutral_before_reverse": True,
        "max_roll_deg": 18,
        "max_pitch_deg": 18,
        "sensor_stale_s": 0.5,
        "scan_stale_s": 0.5,
        "camera_stale_s": 1.0,
        "pose_stale_s": 1.0,
        "tf_stale_s": 1.0,
        "loop_deadline_s": 0.1,
        "low_battery_v": 9.6,
        "critical_battery_v": 9.0,
        "max_cpu_temp_c": 80,
        "max_gpu_temp_c": 80,
        "require_manual_reset_after_estop": True,
        "stop_on_unknown_state": True,
    },
    "localization": {
        "require_map_for_patrol": True,
        "covariance_warn_xy": 0.5,
        "covariance_stop_xy": 1.5,
        "relocalization_attempts": 3,
        "allow_patrol_without_good_pose": False,
    },
    "patrol": {
        "enabled": True,
        "waypoint_file": "waypoints/patrol_outdoor_demo.yaml",
        "loop_count": -1,
        "default_dwell_s": 2.0,
        "retry_per_waypoint": 2,
        "skip_blocked_waypoint": True,
        "max_skips_before_return_home": 3,
        "return_home_on_low_battery": True,
        "pause_on_dynamic_obstacle": True,
        "max_wait_for_obstacle_clear_s": 10.0,
        "max_recovery_attempts": 3,
    },
    "nav2": {
        "enabled": True,
        "cmd_vel_topic": "/cmd_vel",
        "scan_topic": "/scan",
        "map_topic": "/map",
        "odom_topic": "/odom",
        "base_frame": "base_link",
        "global_frame": "map",
        "use_waypoint_follower": True,
        "use_collision_monitor": True,
        "use_velocity_smoother": True,
        "use_keepout_filter": True,
        "use_speed_filter": True,
        "planner": "nav2_default",
        "local_planner_candidates": ["teb", "dwa"],
        "goal_tolerance_xy": 0.35,
        "goal_tolerance_yaw_deg": 15,
        "cancel_on_safety_warning": True,
        "cancel_on_pose_uncertain": True,
    },
    "mapping": {
        "algorithm_2d_preference": ["cartographer", "gmapping", "slam_toolbox"],
        "algorithm_3d_preference": ["rtabmap"],
        "map_save_dir": "maps",
        "dynamic_obstacle_filter": True,
        "save_map_on_exit": False,
    },
    "perception": {
        "lidar_required_for_patrol": True,
        "camera_required_for_patrol": False,
        "min_valid_scan_points": 80,
        "enable_camera_object_detection": False,
        "enable_traversability_estimation": False,
    },
}


def deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_yaml(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return {}
    yaml_path = Path(path).expanduser()
    if not yaml_path.exists() and not yaml_path.is_absolute():
        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = Path(get_package_share_directory("waver_patrol")) / yaml_path
            if share_path.exists():
                yaml_path = share_path
        except Exception:
            pass
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("PyYAML is required to load Waver config files") from exc
    with yaml_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config file must contain a mapping: {path}")
    return data


@dataclass(frozen=True)
class WaverConfig:
    data: dict[str, Any] = field(default_factory=lambda: dict(DEFAULT_CONFIG))

    @classmethod
    def from_file(cls, path: str | Path | None = None) -> "WaverConfig":
        return cls(deep_merge(DEFAULT_CONFIG, load_yaml(path)))

    def section(self, name: str) -> dict[str, Any]:
        value = self.data.get(name, {})
        return dict(value) if isinstance(value, Mapping) else {}

    def get(self, dotted_key: str, default: Any = None) -> Any:
        current: Any = self.data
        for part in dotted_key.split("."):
            if not isinstance(current, Mapping) or part not in current:
                return default
            current = current[part]
        return current
