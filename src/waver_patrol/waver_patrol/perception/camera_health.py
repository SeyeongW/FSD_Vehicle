from __future__ import annotations

from dataclasses import dataclass

from waver_patrol.utils import monotonic


@dataclass
class CameraHealth:
    stale_s: float = 1.0
    last_frame: float | None = None
    dropped_frames: int = 0

    def mark_frame(self) -> None:
        self.last_frame = monotonic()

    def status(self, now: float | None = None) -> str:
        current = monotonic() if now is None else now
        if self.last_frame is None or current - self.last_frame > self.stale_s:
            return "DEGRADED"
        return "OK"
