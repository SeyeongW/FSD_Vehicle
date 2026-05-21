# Failure Modes

Waver defaults to fail-stop. Unknown command sources, invalid numeric commands, stale command input,
scan stale, pose/tf stale, critical battery, critical thermal state, or duplicate command processes
should result in stop or emergency stop.

Recovery order is: stop, wait for clear path, cancel goal and replan, backup only if rear is clear,
skip waypoint if policy allows it, return home if low battery or repeated skips, emergency stop if
recovery repeats beyond limit.
