from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ThermalGuard:
    max_cpu_temp_c: float = 80.0
    max_gpu_temp_c: float = 80.0

    def evaluate(self, cpu_temp_c: float | None, gpu_temp_c: float | None = None) -> str:
        if cpu_temp_c is not None and cpu_temp_c >= self.max_cpu_temp_c:
            return "CPU_CRITICAL"
        if gpu_temp_c is not None and gpu_temp_c >= self.max_gpu_temp_c:
            return "GPU_CRITICAL"
        return "OK"

    @staticmethod
    def read_linux_thermal(path: str | Path) -> float | None:
        try:
            value = Path(path).read_text(encoding="utf-8").strip()
            return float(value) / 1000.0
        except (OSError, ValueError):
            return None
