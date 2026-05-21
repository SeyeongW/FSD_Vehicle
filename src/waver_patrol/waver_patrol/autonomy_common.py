from __future__ import annotations

import math
from pathlib import Path
from typing import Any


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def finite_or_zero(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def yaw_deg_to_rad(yaw_deg: float) -> float:
    return math.radians(finite_or_zero(yaw_deg))


def stop_twist():
    from geometry_msgs.msg import Twist

    return Twist()


def make_twist(linear: float, angular: float):
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.linear.x = finite_or_zero(linear)
    msg.angular.z = finite_or_zero(angular)
    return msg


def twist_is_finite(msg: Any) -> bool:
    return math.isfinite(float(msg.linear.x)) and math.isfinite(float(msg.angular.z))


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    import yaml

    yaml_path = Path(path).expanduser()
    if not yaml_path.exists() and not yaml_path.is_absolute():
        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = Path(get_package_share_directory("waver_patrol")) / yaml_path
            if share_path.exists():
                yaml_path = share_path
        except Exception:
            pass
    with yaml_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {yaml_path}")
    return data


def now_seconds(node: Any) -> float:
    return node.get_clock().now().nanoseconds * 1e-9
