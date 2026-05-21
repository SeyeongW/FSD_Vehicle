from __future__ import annotations

import json
from pathlib import Path

from waver_patrol.patrol.patrol_events import PatrolEvent


class PatrolLogger:
    def __init__(self, path: str | Path = "logs/patrol_events.jsonl"):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: PatrolEvent) -> None:
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event.__dict__, sort_keys=True) + "\n")
