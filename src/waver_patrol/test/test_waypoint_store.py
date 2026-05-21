from waver_patrol.patrol.waypoint_store import WaypointStore


def test_waypoint_load(tmp_path):
    path = tmp_path / "route.yaml"
    path.write_text(
        """
route_name: test
frame_id: map
loop_count: 1
mode: loop
start_policy: nearest_first
home: {name: home, x: 0.0, y: 0.0, yaw_deg: 0.0}
waypoints:
  - {name: a, x: 1.0, y: 0.0, yaw_deg: 0.0}
""",
        encoding="utf-8",
    )
    spec = WaypointStore.load(path)
    assert spec.route_name == "test"
    assert spec.waypoints[0].name == "a"
