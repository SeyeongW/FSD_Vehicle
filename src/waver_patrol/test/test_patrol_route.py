from waver_patrol.patrol.patrol_route import PatrolRoute
from waver_patrol.patrol.waypoint_store import PatrolSpec, Waypoint


def _spec(mode="loop", loop_count=2):
    return PatrolSpec(
        "route",
        "map",
        loop_count,
        mode,
        "nearest_first",
        Waypoint("home", 0, 0),
        [Waypoint("a", 1, 0), Waypoint("b", 2, 0), Waypoint("c", 3, 0)],
    )


def test_loop_order():
    route = PatrolRoute(_spec(loop_count=2))
    assert [wp.name for wp in route.iter_loop()] == ["a", "b", "c", "a", "b", "c"]


def test_pingpong_order():
    route = PatrolRoute(_spec(mode="pingpong", loop_count=1))
    assert [wp.name for wp in route.iter_loop()] == ["a", "b", "c", "b"]
