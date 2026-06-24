from __future__ import annotations

import importlib.util

import pytest


ROS_MODULES = ("rclpy", "geometry_msgs", "sensor_msgs", "std_msgs", "nav_msgs")


def ros_available(modules: tuple[str, ...] = ROS_MODULES) -> bool:
    return all(importlib.util.find_spec(module) is not None for module in modules)


def require_ros(*modules: str) -> None:
    for module in modules or ROS_MODULES:
        pytest.importorskip(module)
