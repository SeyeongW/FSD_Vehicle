from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any

import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


def now_stamp_sec() -> float:
    return time.time()


def is_finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def is_finite_pose(pose: PoseStamped | Pose) -> bool:
    p = pose.pose.position if isinstance(pose, PoseStamped) else pose.position
    return is_finite_number(p.x) and is_finite_number(p.y) and is_finite_number(p.z)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance_xy(a: PoseStamped | Point, b: PoseStamped | Point) -> float:
    pa = a.pose.position if isinstance(a, PoseStamped) else a
    pb = b.pose.position if isinstance(b, PoseStamped) else b
    return math.hypot(pa.x - pb.x, pa.y - pb.y)


def pose_stamped(frame_id: str, x: float, y: float, yaw: float, z: float = 0.0) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation = yaw_to_quaternion(float(yaw))
    return msg


def expand_path(path: str, package_share: str | None = None) -> str:
    expanded = os.path.expanduser(os.path.expandvars(path))
    if os.path.isabs(expanded):
        return expanded
    if package_share:
        candidate = os.path.join(package_share, expanded)
        if os.path.exists(candidate):
            return candidate
    return os.path.abspath(expanded)


@dataclass
class Waypoint:
    name: str
    pose: PoseStamped
    dwell_s: float = 0.0


@dataclass
class MissionRoute:
    route_name: str
    frame_id: str
    loop: bool
    home: PoseStamped
    charge: PoseStamped
    waypoints: list[Waypoint]
    failure_policy: dict[str, Any]
    experiment_tags: list[str]


def load_route_yaml(path: str, package_share: str | None = None) -> MissionRoute:
    resolved = expand_path(path, package_share)
    with open(resolved, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    frame_id = str(data.get("frame_id", "map"))
    route_name = str(data.get("route_name", "waver_route"))
    loop = bool(data.get("loop", data.get("patrol_loop", True)))

    def read_pose(block: dict[str, Any], fallback_name: str) -> PoseStamped:
        return pose_stamped(
            frame_id,
            float(block.get("x", 0.0)),
            float(block.get("y", 0.0)),
            math.radians(float(block.get("yaw_deg", 0.0))),
            float(block.get("z", 0.0)),
        )

    home = read_pose(data.get("home", {}), "home")
    charge = read_pose(data.get("charge", data.get("home", {})), "charge")
    waypoints: list[Waypoint] = []
    for index, item in enumerate(data.get("waypoints", [])):
        if not isinstance(item, dict):
            continue
        wp = Waypoint(
            name=str(item.get("name", f"wp_{index}")),
            pose=read_pose(item, f"wp_{index}"),
            dwell_s=float(item.get("dwell_s", data.get("default_dwell_s", 0.0))),
        )
        waypoints.append(wp)
    return MissionRoute(
        route_name=route_name,
        frame_id=frame_id,
        loop=loop,
        home=home,
        charge=charge,
        waypoints=waypoints,
        failure_policy=dict(data.get("failure_policy", {})),
        experiment_tags=[str(x) for x in data.get("experiment_tags", [])],
    )


def offset_goal_from_target(target: PoseStamped, offset_distance_m: float, yaw_policy: str = "FACE_TARGET") -> PoseStamped:
    """Create a 2D Nav2 goal near, not on top of, the object target.

    The target's x/y is assumed to be in a global frame. The robot stops
    `offset_distance_m` before the object along the ray from the map origin.
    For real deployments this should be replaced with a robot-current-pose ray.
    """
    x = float(target.pose.position.x)
    y = float(target.pose.position.y)
    yaw = math.atan2(y, x) if abs(x) + abs(y) > 1e-6 else 0.0
    distance = math.hypot(x, y)
    stop_distance = max(0.0, distance - max(0.0, offset_distance_m))
    scale = stop_distance / distance if distance > 1e-6 else 0.0
    goal = PoseStamped()
    goal.header = target.header
    goal.pose.position.x = x * scale
    goal.pose.position.y = y * scale
    goal.pose.position.z = 0.0
    if yaw_policy == "KEEP_CURRENT_YAW":
        goal.pose.orientation = target.pose.orientation
    elif yaw_policy == "FACE_FORWARD":
        goal.pose.orientation = yaw_to_quaternion(0.0)
    else:
        goal.pose.orientation = yaw_to_quaternion(yaw)
    return goal
