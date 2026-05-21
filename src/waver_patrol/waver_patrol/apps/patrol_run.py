from __future__ import annotations

import argparse
import time

from waver_patrol.patrol.patrol_manager import PatrolManager
from waver_patrol.patrol.waypoint_store import WaypointStore


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoints", nargs="?", default="waypoints/patrol_outdoor_demo.yaml")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args(argv)
    spec = WaypointStore.load(args.waypoints)
    manager = PatrolManager(spec)
    print(f"Route {spec.route_name}: {manager.next_waypoint_names(12)}")
    if args.live:
        print("Patrol manager is armed as a supervised placeholder; Nav2 action dispatch is enabled in the bridge layer.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Patrol manager stopped; send stop_robot before touching the robot.")
    elif not args.preview:
        print("Use ros2 launch waver_patrol waver_patrol.launch.py for live supervised patrol.")
