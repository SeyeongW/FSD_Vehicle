from __future__ import annotations


def should_insert_into_static_map(dynamic_obstacle: bool) -> bool:
    return not dynamic_obstacle
