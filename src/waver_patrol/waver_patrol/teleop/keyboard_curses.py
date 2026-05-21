from __future__ import annotations

import curses
import time
from dataclasses import dataclass

from waver_patrol.safety.command import WheelCommand


MOVE_KEYS = {
    "w": (1, 0),
    "i": (1, 0),
    "s": (-1, 0),
    ",": (-1, 0),
    "a": (0, 1),
    "j": (0, 1),
    "d": (0, -1),
    "l": (0, -1),
    "q": (1, 1),
    "u": (1, 1),
    "e": (1, -1),
    "o": (1, -1),
    "z": (-1, 1),
    "m": (-1, 1),
    "c": (-1, -1),
    ".": (-1, -1),
}


@dataclass
class KeyboardTeleopState:
    speed: float = 0.16
    turn_gain: float = 0.65
    speed_step: float = 0.02
    timeout_s: float = 0.3
    last_input: float = 0.0
    paused: bool = False

    def command_for_key(self, key: str, now: float | None = None) -> tuple[WheelCommand, str]:
        now = time.monotonic() if now is None else now
        self.last_input = now
        lower = key.lower()
        if lower in {" ", "k", "\x1b"}:
            return WheelCommand.stop(source="manual", reason="operator stop"), "STOP"
        if key == "+":
            self.speed = min(0.28, self.speed + self.speed_step)
            return WheelCommand.stop(source="manual", reason="speed change"), "SPEED_UP"
        if key == "-":
            self.speed = max(0.04, self.speed - self.speed_step)
            return WheelCommand.stop(source="manual", reason="speed change"), "SPEED_DOWN"
        if key == "[":
            self.turn_gain = max(0.2, self.turn_gain - 0.05)
            return WheelCommand.stop(source="manual", reason="turn gain change"), "TURN_DOWN"
        if key == "]":
            self.turn_gain = min(1.0, self.turn_gain + 0.05)
            return WheelCommand.stop(source="manual", reason="turn gain change"), "TURN_UP"
        if lower == "p":
            self.paused = not self.paused
            return WheelCommand.stop(source="manual", reason="patrol pause toggle"), "PATROL_TOGGLE"
        if lower in MOVE_KEYS:
            forward, turn = MOVE_KEYS[lower]
            left = self.speed * forward - self.speed * self.turn_gain * turn
            right = self.speed * forward + self.speed * self.turn_gain * turn
            return WheelCommand(left, right, source="manual", reason=f"key:{key}"), "MOVE"
        return WheelCommand.stop(source="manual", reason="unknown key"), "UNKNOWN"

    def timeout_command(self, now: float | None = None) -> WheelCommand | None:
        now = time.monotonic() if now is None else now
        if self.last_input and now - self.last_input > self.timeout_s:
            return WheelCommand.stop(source="manual", reason="keyboard timeout")
        return None


def run_curses(send_command, state: KeyboardTeleopState | None = None) -> None:
    state = state or KeyboardTeleopState()

    def _main(stdscr) -> None:
        stdscr.nodelay(True)
        stdscr.keypad(True)
        curses.curs_set(0)
        while True:
            ch = stdscr.getch()
            if ch != -1:
                key = chr(ch) if ch < 256 else ""
                command, event = state.command_for_key(key)
                send_command(command)
                if key in {"\x1b", "\x03"}:
                    break
                stdscr.erase()
                stdscr.addstr(0, 0, f"Waver teleop speed={state.speed:.2f} turn={state.turn_gain:.2f} {event}")
                stdscr.refresh()
            timeout = state.timeout_command()
            if timeout is not None:
                send_command(timeout)
            time.sleep(0.03)

    curses.wrapper(_main)
