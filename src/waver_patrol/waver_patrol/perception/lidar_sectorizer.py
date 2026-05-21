from __future__ import annotations

from waver_patrol.safety.collision_guard import CollisionGuard, ScanLike, SectorState


def sectorize(scan: ScanLike) -> dict[str, SectorState]:
    return CollisionGuard().sectorize(scan)
