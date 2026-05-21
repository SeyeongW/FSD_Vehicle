"""Track LiDAR cluster centroids over time and flag the moving ones.

Input : /lidar/detections   (PoseArray of cluster centroids, lidar frame)
Output:
  /lidar/tracked_markers     (MarkerArray)  -- RViz viz: moving=red + velocity
                                                arrow, static=dim gray, id label
  /lidar/moving_objects      (PoseArray)    -- moving centroids (selection input)
  /v2v/moving_objects         (PoseArray)   -- same set, for sharing

"Moving" is decided from NET displacement over a time window, not the
frame-to-frame velocity. DBSCAN centroids of static objects jitter by tens of
cm per frame on sparse clouds, which naive instantaneous velocity reads as
several m/s. A real mover accumulates displacement in one direction; jitter
oscillates around a fixed point, so its net displacement over ~1 s stays near
zero and is correctly rejected.

Association across frames is greedy nearest-neighbour within `assoc_gate` m.

Frame note: positions are in the LiDAR frame. For vehicle-to-vehicle use they
must be transformed into a shared map frame (requires localization).
"""
from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class Track:
    _next_id = 0

    def __init__(self, pos, t, window):
        self.id = Track._next_id
        Track._next_id += 1
        self.x, self.y, self.z = pos
        self.hits = 1
        self.misses = 0
        self.window = window
        self.history = deque()           # (x, y, t)
        self.history.append((pos[0], pos[1], t))

    def update(self, pos, t):
        self.x, self.y, self.z = pos
        self.hits += 1
        self.misses = 0
        self.history.append((pos[0], pos[1], t))
        # Drop samples older than the motion window.
        while len(self.history) > 2 and (t - self.history[0][2]) > self.window:
            self.history.popleft()

    def _window_endpoints(self):
        if len(self.history) < 2:
            return None
        x0, y0, t0 = self.history[0]
        x1, y1, t1 = self.history[-1]
        dt = t1 - t0
        if dt <= 1e-3:
            return None
        return x0, y0, x1, y1, dt

    def net_displacement(self):
        e = self._window_endpoints()
        if e is None:
            return 0.0
        x0, y0, x1, y1, _dt = e
        return math.hypot(x1 - x0, y1 - y0)

    def velocity(self):
        """Average velocity vector (vx, vy) over the window, m/s."""
        e = self._window_endpoints()
        if e is None:
            return 0.0, 0.0
        x0, y0, x1, y1, dt = e
        return (x1 - x0) / dt, (y1 - y0) / dt


class LidarMotionTrackerNode(Node):
    def __init__(self):
        super().__init__('lidar_motion_tracker_node')

        self.declare_parameter('detections_topic', '/lidar/detections')
        self.declare_parameter('assoc_gate', 0.7)        # max match distance (m)
        self.declare_parameter('motion_window', 1.0)     # seconds of history to judge motion
        self.declare_parameter('min_displacement', 0.4)  # net move over window to be "moving" (m)
        self.declare_parameter('speed_thresh', 0.3)      # AND windowed speed over this (m/s)
        self.declare_parameter('min_hits', 4)            # frames before trusting a track
        self.declare_parameter('max_misses', 5)          # drop track after N missed frames
        self.declare_parameter('marker_lifetime', 0.4)   # seconds

        self.gate = float(self.get_parameter('assoc_gate').value)
        self.window = float(self.get_parameter('motion_window').value)
        self.min_disp = float(self.get_parameter('min_displacement').value)
        self.speed_thresh = float(self.get_parameter('speed_thresh').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.max_misses = int(self.get_parameter('max_misses').value)
        self.lifetime = float(self.get_parameter('marker_lifetime').value)

        self.tracks: list[Track] = []

        self.create_subscription(PoseArray,
                                 self.get_parameter('detections_topic').value,
                                 self.detections_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar/tracked_markers', 10)
        self.moving_pub = self.create_publisher(PoseArray, '/lidar/moving_objects', 10)
        self.v2v_pub = self.create_publisher(PoseArray, '/v2v/moving_objects', 10)

        self.get_logger().info('Motion tracker started (net-displacement based).')

    def detections_cb(self, msg: PoseArray):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        dets = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self._associate(dets, t)
        self._publish(msg.header)

    def _associate(self, dets, t):
        unmatched = set(range(len(dets)))
        for tr in self.tracks:
            best_i, best_d = None, self.gate
            for i in unmatched:
                d = math.hypot(dets[i][0] - tr.x, dets[i][1] - tr.y)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i is not None:
                tr.update(dets[best_i], t)
                unmatched.discard(best_i)
            else:
                tr.misses += 1

        for i in unmatched:
            self.tracks.append(Track(dets[i], t, self.window))

        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]

    def _is_moving(self, tr: Track) -> bool:
        if tr.hits < self.min_hits:
            return False
        if tr.net_displacement() < self.min_disp:
            return False
        vx, vy = tr.velocity()
        return math.hypot(vx, vy) > self.speed_thresh

    def _publish(self, header):
        markers = MarkerArray()
        moving = PoseArray()
        moving.header = header

        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for tr in self.tracks:
            if tr.hits < self.min_hits:
                continue
            is_moving = self._is_moving(tr)

            box = Marker()
            box.header = header
            box.ns = 'moving' if is_moving else 'static'
            box.id = tr.id
            box.type = Marker.SPHERE
            box.action = Marker.ADD
            box.pose.position.x = tr.x
            box.pose.position.y = tr.y
            box.pose.position.z = tr.z
            box.pose.orientation.w = 1.0
            box.scale.x = box.scale.y = box.scale.z = 0.5
            box.color = (ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9) if is_moving
                         else ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.35))
            box.lifetime.nanosec = int(self.lifetime * 1e9)
            markers.markers.append(box)

            if is_moving:
                vx, vy = tr.velocity()
                speed = math.hypot(vx, vy)

                arrow = Marker()
                arrow.header = header
                arrow.ns = 'velocity'
                arrow.id = tr.id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.points = [Point(x=tr.x, y=tr.y, z=tr.z),
                                Point(x=tr.x + vx, y=tr.y + vy, z=tr.z)]
                arrow.scale.x = 0.08
                arrow.scale.y = 0.16
                arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
                arrow.lifetime.nanosec = int(self.lifetime * 1e9)
                markers.markers.append(arrow)

                txt = Marker()
                txt.header = header
                txt.ns = 'label'
                txt.id = tr.id
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = tr.x
                txt.pose.position.y = tr.y
                txt.pose.position.z = tr.z + 0.6
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.3
                txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                txt.text = f'#{tr.id} {speed:.1f}m/s'
                txt.lifetime.nanosec = int(self.lifetime * 1e9)
                markers.markers.append(txt)

                pose = Pose()
                pose.position.x = tr.x
                pose.position.y = tr.y
                pose.position.z = tr.z
                pose.orientation.w = 1.0
                moving.poses.append(pose)

        self.marker_pub.publish(markers)
        self.moving_pub.publish(moving)
        self.v2v_pub.publish(moving)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMotionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
