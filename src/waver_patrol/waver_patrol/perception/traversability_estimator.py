from __future__ import annotations


def terrain_speed_scale(label: str) -> float:
    table = {
        "mud": 0.35,
        "sand": 0.45,
        "wet_grass": 0.45,
        "slope": 0.5,
        "curb": 0.0,
        "ditch": 0.0,
        "puddle": 0.3,
    }
    return table.get(label, 1.0)
