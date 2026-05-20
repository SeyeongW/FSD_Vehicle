"""Track LiDAR cluster centroids over time and flag the moving ones.

Input : /lidar/detections   (PoseArray of cluster centroids, lidar frame)
Output:
  /lidar/tracked_markers     (MarkerArray)  -- RViz viz: moving=red + velocity
                                                arrow, static=dim gray, id text
  /lidar/moving_objects      (PoseArray)    -- moving centroids (selection input)
  /v2v/moving_objects         (PoseArray)   -- same set, intended for sharing
                                                with other vehicles (see frame note)

"Moving" = a track whose smoothed speed exceeds `speed_thresh` and that has
been seen for at least `min_hits` frames. Association across frames is greedy
nearest-neighbour within `assoc_gate` metres; DBSCAN labels are not stable so
we track explicitly.

Frame note: positions are in the LiDAR frame. For real vehicle-to-vehicle use
they must be transformed into a shared map frame (requires localization).
"""
from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class Track:
    _next_id = 0

    def __init__(self, pos, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        self.x, self.y, self.z = pos
        self.vx = self.vy = self.vz = 0.0
        self.hits = 1
        self.misses = 0
        self.stamp = stamp

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    def predict(self, dt):
        return (self.x + self.vx * dt, self.y + self.vy * dt, self.z)


class LidarMotionTrackerNode(Node):
    def __init__(self):
        super().__init__('lidar_motion_tracker_node')

        self.declare_parameter('detections_topic', '/lidar/detections')
        self.declare_parameter('assoc_gate', 1.0)       # max match distance (m)
        self.declare_parameter('speed_thresh', 0.3)     # moving if speed > this (m/s)
        self.declare_parameter('min_hits', 3)           # frames before trusting a track
        self.declare_parameter('max_misses', 5)         # drop track after N missed frames
        self.declare_parameter('vel_alpha', 0.5)        # velocity EMA smoothing
        self.declare_parameter('marker_lifetime', 0.4)  # seconds

        self.gate = float(self.get_parameter('assoc_gate').value)
        self.speed_thresh = float(self.get_parameter('speed_thresh').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.max_misses = int(self.get_parameter('max_misses').value)
        self.alpha = float(self.get_parameter('vel_alpha').value)
        self.lifetime = float(self.get_parameter('marker_lifetime').value)

        self.tracks: list[Track] = []
        self.last_stamp = None

        self.create_subscription(PoseArray,
                                 self.get_parameter('detections_topic').value,
                                 self.detections_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar/tracked_markers', 10)
        self.moving_pub = self.create_publisher(PoseArray, '/lidar/moving_objects', 10)
        self.v2v_pub = self.create_publisher(PoseArray, '/v2v/moving_objects', 10)

        self.get_logger().info('Motion tracker started.')

    def detections_cb(self, msg: PoseArray):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        dt = 0.0 if self.last_stamp is None else max(t - self.last_stamp, 1e-3)
        self.last_stamp = t

        dets = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self._associate(dets, dt)

        self._publish(msg.header)

    def _associate(self, dets, dt):
        unmatched = set(range(len(dets)))
        # Greedy NN: for each track (predicted), grab nearest free detection in gate.
        for tr in self.tracks:
            px, py, pz = tr.predict(dt)
            best_i, best_d = None, self.gate
            for i in unmatched:
                dx = dets[i][0] - px
                dy = dets[i][1] - py
                d = math.hypot(dx, dy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_i is not None:
                nx, ny, nz = dets[best_i]
                if dt > 0:
                    vx = (nx - tr.x) / dt
                    vy = (ny - tr.y) / dt
                    vz = (nz - tr.z) / dt
                    tr.vx = self.alpha * vx + (1 - self.alpha) * tr.vx
                    tr.vy = self.alpha * vy + (1 - self.alpha) * tr.vy
                    tr.vz = self.alpha * vz + (1 - self.alpha) * tr.vz
                tr.x, tr.y, tr.z = nx, ny, nz
                tr.hits += 1
                tr.misses = 0
                unmatched.discard(best_i)
            else:
                tr.misses += 1

        # Spawn tracks for leftover detections.
        for i in unmatched:
            self.tracks.append(Track(dets[i], self.last_stamp))

        # Cull dead tracks.
        self.tracks = [tr for tr in self.tracks if tr.misses <= self.max_misses]

    def _is_moving(self, tr: Track) -> bool:
        return tr.hits >= self.min_hits and tr.speed > self.speed_thresh

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
                         else ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4))
            box.lifetime.nanosec = int(self.lifetime * 1e9)
            markers.markers.append(box)

            if is_moving:
                arrow = Marker()
                arrow.header = header
                arrow.ns = 'velocity'
                arrow.id = tr.id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point(x=tr.x, y=tr.y, z=tr.z)
                end = Point(x=tr.x + tr.vx, y=tr.y + tr.vy, z=tr.z + tr.vz)
                arrow.points = [start, end]
                arrow.scale.x = 0.08   # shaft diameter
                arrow.scale.y = 0.16   # head diameter
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
                txt.text = f'#{tr.id} {tr.speed:.1f}m/s'
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
