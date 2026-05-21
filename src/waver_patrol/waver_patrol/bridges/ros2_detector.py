from __future__ import annotations


def ros2_available() -> bool:
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False
