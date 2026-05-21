from __future__ import annotations


def map_quality_warning(scan_match_score: float | None, min_score: float = 0.45) -> str:
    if scan_match_score is None:
        return "UNKNOWN"
    return "WARN" if scan_match_score < min_score else "OK"
