from __future__ import annotations


def choose_mapping_algorithm(preference: list[str], available: set[str]) -> str | None:
    for name in preference:
        if name in available:
            return name
    return None
