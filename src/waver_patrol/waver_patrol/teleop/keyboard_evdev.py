from __future__ import annotations


def evdev_available() -> bool:
    try:
        import evdev  # noqa: F401
        return True
    except ImportError:
        return False
