from __future__ import annotations

import glob
from pathlib import Path


def detect_ports(
    preferred_ports: list[str] | None = None,
    glob_ports: list[str] | None = None,
) -> list[str]:
    preferred_ports = preferred_ports or ["/dev/ttyTHS0", "/dev/serial0"]
    glob_ports = glob_ports or ["/dev/ttyUSB*", "/dev/ttyACM*"]
    candidates: list[str] = []
    for port in preferred_ports:
        if Path(port).exists():
            candidates.append(port)
    for pattern in glob_ports:
        for port in sorted(glob.glob(pattern)):
            if port not in candidates:
                candidates.append(port)
    return candidates
