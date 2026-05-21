from __future__ import annotations

import fcntl
import os
from pathlib import Path


class ProcessLock:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.fd: int | None = None

    def acquire(self) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fd = os.open(str(self.path), os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.ftruncate(self.fd, 0)
            os.write(self.fd, str(os.getpid()).encode("ascii"))
            return True
        except BlockingIOError:
            return False

    def release(self) -> None:
        if self.fd is None:
            return
        fcntl.flock(self.fd, fcntl.LOCK_UN)
        os.close(self.fd)
        self.fd = None

    def __enter__(self) -> "ProcessLock":
        if not self.acquire():
            raise RuntimeError(f"Another Waver process owns lock {self.path}")
        return self

    def __exit__(self, *_args: object) -> None:
        self.release()
