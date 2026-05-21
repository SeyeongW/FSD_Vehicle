from __future__ import annotations

import subprocess
from dataclasses import dataclass


@dataclass
class ManagedLaunch:
    command: list[str]
    process: subprocess.Popen[str] | None = None

    def start(self) -> None:
        if self.process is None or self.process.poll() is not None:
            self.process = subprocess.Popen(self.command, text=True)

    def stop(self) -> None:
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
