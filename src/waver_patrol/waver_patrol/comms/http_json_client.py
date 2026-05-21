from __future__ import annotations

import json
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HttpClientConfig:
    base_ip: str = "192.168.4.1"
    timeout_s: float = 0.3


class HttpJsonClient:
    def __init__(self, config: HttpClientConfig | None = None):
        self.config = config or HttpClientConfig()

    def send_json(self, payload: dict[str, Any]) -> None:
        url = f"http://{self.config.base_ip}/js?json={urllib.parse.quote(json.dumps(payload))}"
        with urllib.request.urlopen(url, timeout=self.config.timeout_s) as response:
            response.read()

    def send_stop(self, repeat: int = 5) -> None:
        for _ in range(repeat):
            self.send_json({"T": 1, "L": 0, "R": 0})
