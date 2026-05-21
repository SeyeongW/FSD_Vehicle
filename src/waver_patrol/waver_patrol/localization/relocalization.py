from __future__ import annotations


class Relocalization:
    def __init__(self, attempts: int = 3):
        self.attempts = attempts
        self.used = 0

    def try_once(self) -> bool:
        self.used += 1
        return self.used <= self.attempts
