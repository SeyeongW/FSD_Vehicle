#!/usr/bin/env python3

import math
import os
import random
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Float32, String


def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def vec_len(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_norm(a):
    l = vec_len(a)
    if l < 1e-9:
        return (0.0, 0.0, 0.0)
    return (a[0] / l, a[1] / l, a[2] / l)


def vec_limit(a, max_len):
    l = vec_len(a)
    if l < 1e-9 or l <= max_len:
        return a
    s = max_len / l
    return (a[0] * s, a[1] * s, a[2] * s)


def horizontal_len(x, y):
    return math.sqrt(x * x + y * y)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


@dataclass
class BirdConfig:
    name: str
    max_speed: float
    min_speed: float
    arrival_threshold: float


@dataclass
class BirdRuntime:
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0

    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    initialized: bool = False


class BirdManager(Node):
    def __init__(self):
        super().__init__('bird_manager')

        self.declare_parameter(
            'active_birds',
            os.environ.get('BIRD_MANAGER_ACTIVE_BIRDS', 'bird_single'),
        )
        self.declare_parameter('publish_waver_detection_topics', True)
        self.declare_parameter('z_min_m', float(os.environ.get('BIRD_MANAGER_Z_MIN_M', '3.1')))
        self.declare_parameter('z_max_m', float(os.environ.get('BIRD_MANAGER_Z_MAX_M', '3.4')))
        self.declare_parameter('min_speed_mps', float(os.environ.get('BIRD_MANAGER_MIN_SPEED_MPS', '0.05')))
        self.declare_parameter('max_speed_mps', float(os.environ.get('BIRD_MANAGER_MAX_SPEED_MPS', '0.18')))
        self.declare_parameter('min_xy_radius_m', 0.0)
        self.publish_waver_detection_topics = bool(
            self.get_parameter('publish_waver_detection_topics').value
        )

        # 15x15 map, leave room from the 2m boundary wall.
        self.x_min = -6.5
        self.x_max = 6.5
        self.y_min = -6.5
        self.y_max = 6.5

        self.z_min = float(self.get_parameter('z_min_m').value)
        self.z_max = float(self.get_parameter('z_max_m').value)
        if self.z_max < self.z_min:
            self.z_min, self.z_max = self.z_max, self.z_min
        self.min_xy_radius = max(0.0, float(self.get_parameter('min_xy_radius_m').value))

        self.dt = 0.08
        self.busy = False
        self.current_states = {}
        self.pending_gets = 0

        self.model_yaw_offset = math.pi / 2.0

        self.max_pitch_rad = math.radians(12.0)
        self.max_roll_rad = math.radians(18.0)
        self.max_yaw_rate = math.radians(60.0)
        self.max_pitch_rate = math.radians(30.0)
        self.max_roll_rate = math.radians(45.0)
        self.bank_from_turn_gain = 0.45

        self.slowdown_radius_single = 4.0
        self.goal_damping_gain = 0.18
        self.vertical_damping_gain = 0.20

        self.state_fail_count = {}
        self.service_log_time = 0.0
        self.services_ready = False
        self.set_service_name = ''
        self.get_service_name = ''
        self.set_cli = None
        self.get_cli = None
        self.tracking_start_wall = None
        self.tracking_start_text = 'waiting'
        self.last_metric_samples = {}
        self.obstacle_metrics = {}
        self.last_obstacle_status_time = 0.0

        active_names = [
            name.strip()
            for name in str(self.get_parameter('active_birds').value).split(',')
            if name.strip()
        ]
        if not active_names:
            active_names = ['bird_single']

        min_speed = max(0.0, float(self.get_parameter('min_speed_mps').value))
        max_speed = max(min_speed, float(self.get_parameter('max_speed_mps').value))
        self.birds = [
            BirdConfig(name, max_speed, min_speed, 1.0)
            for name in active_names
        ]

        self.runtime = {bird.name: BirdRuntime() for bird in self.birds}

        self.bird_pose_pub = self.create_publisher(PoseStamped, '/bird/nearest_pose', 10)
        self.bird_visible_pub = self.create_publisher(Bool, '/bird/visible', 10)
        self.dynamic_targets_pub = None
        self.dynamic_obstacle_state_pub = None
        self.height_filter_debug_pub = None
        self.target_class_pub = None
        self.target_confidence_pub = None
        self.bird_confirmed_pub = None
        self.classification_state_pub = None
        if self.publish_waver_detection_topics:
            self.dynamic_targets_pub = self.create_publisher(
                PoseArray, '/waver/elevated_dynamic_targets', 10
            )
            self.dynamic_obstacle_state_pub = self.create_publisher(
                String, '/waver/dynamic_obstacle_state', 10
            )
            self.height_filter_debug_pub = self.create_publisher(
                String, '/waver/height_filter_debug', 10
            )
            self.target_class_pub = self.create_publisher(String, '/waver/target_class', 10)
            self.target_confidence_pub = self.create_publisher(
                Float32, '/waver/target_confidence', 10
            )
            self.bird_confirmed_pub = self.create_publisher(Bool, '/waver/bird_confirmed', 10)
            self.classification_state_pub = self.create_publisher(
                String, '/waver/classification_state', 10
            )

        for bird in self.birds:
            self.pick_new_target(bird.name)
        self.service_timer = self.create_timer(1.0, self.connect_services_if_ready)
        self.timer = self.create_timer(self.dt, self.update_all)
        self.get_logger().info(
            f'bird_manager started z_range={self.z_min:.2f}-{self.z_max:.2f}m '
            f'speed_range={min_speed:.2f}-{max_speed:.2f}m/s'
        )

    def find_service_name(self, preferred, service_type):
        services = self.get_service_names_and_types()

        for name, types in services:
            if service_type in types and name in preferred:
                return name

        for name, types in services:
            if service_type in types:
                return name

        return ''

    def connect_services_if_ready(self):
        if self.services_ready:
            return

        set_name = self.find_service_name(
            preferred=['/gazebo/set_entity_state', '/set_entity_state'],
            service_type='gazebo_msgs/srv/SetEntityState'
        )
        get_name = self.find_service_name(
            preferred=['/gazebo/get_entity_state', '/get_entity_state'],
            service_type='gazebo_msgs/srv/GetEntityState'
        )

        if not set_name or not get_name:
            now = time.monotonic()
            if now - self.service_log_time > 5.0:
                self.service_log_time = now
                self.get_logger().warn(
                    'Waiting for Gazebo entity state services. '
                    'Check that libgazebo_ros_state.so is loaded if this persists.'
                )
            return

        if self.set_cli is None or self.set_service_name != set_name:
            self.set_service_name = set_name
            self.set_cli = self.create_client(SetEntityState, self.set_service_name)
        if self.get_cli is None or self.get_service_name != get_name:
            self.get_service_name = get_name
            self.get_cli = self.create_client(GetEntityState, self.get_service_name)

        if not self.set_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().info(f'{self.set_service_name} waiting...')
            return
        if not self.get_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().info(f'{self.get_service_name} waiting...')
            return

        self.services_ready = True
        self.service_timer.cancel()
        self.get_logger().info(
            f'Gazebo entity state services ready: {self.get_service_name}, {self.set_service_name}'
        )

    def pick_random_target(self):
        for _ in range(40):
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            if horizontal_len(x, y) >= self.min_xy_radius:
                return (x, y, random.uniform(self.z_min, self.z_max))
        angle = random.uniform(-math.pi, math.pi)
        radius = min(
            self.min_xy_radius,
            min(abs(self.x_min), abs(self.x_max), abs(self.y_min), abs(self.y_max)),
        )
        return (
            radius * math.cos(angle),
            radius * math.sin(angle),
            random.uniform(self.z_min, self.z_max),
        )

    def pick_new_target(self, name):
        tx, ty, tz = self.pick_random_target()
        rt = self.runtime[name]
        rt.target_x = tx
        rt.target_y = ty
        rt.target_z = tz

    def update_all(self):
        if self.busy:
            return
        if not self.services_ready or self.get_cli is None or self.set_cli is None:
            return

        self.busy = True
        self.current_states = {}
        self.pending_gets = len(self.birds)

        for bird in self.birds:
            req = GetEntityState.Request()
            req.name = bird.name
            req.reference_frame = 'world'
            future = self.get_cli.call_async(req)
            future.add_done_callback(lambda fut, b=bird: self.on_got_state(fut, b))

    def on_got_state(self, future, bird):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'[{bird.name}] get_entity_state exception: {e}')
            self.pending_gets -= 1
            self.finish_get_phase_if_ready()
            return

        if not res.success:
            c = self.state_fail_count.get(bird.name, 0) + 1
            self.state_fail_count[bird.name] = c
            if c % 20 == 1:
                self.get_logger().warn(f'[{bird.name}] get_entity_state failed')
            self.pending_gets -= 1
            self.finish_get_phase_if_ready()
            return

        self.state_fail_count[bird.name] = 0
        self.current_states[bird.name] = res.state
        self.pending_gets -= 1
        self.finish_get_phase_if_ready()

    def finish_get_phase_if_ready(self):
        if self.pending_gets > 0:
            return

        if not self.current_states:
            self.publish_detection(None)
            self.publish_dynamic_obstacles()
            self.busy = False
            return

        world_info = self.build_world_info()

        for bird in self.birds:
            if bird.name not in self.current_states:
                continue
            self.move_one_bird(bird, world_info)

        self.publish_dynamic_obstacles()
        self.publish_nearest_bird()
        self.busy = False

    def build_world_info(self):
        info = {}
        for bird in self.birds:
            state = self.current_states.get(bird.name)
            if state is None:
                continue

            rt = self.runtime[bird.name]

            x = state.pose.position.x
            y = state.pose.position.y
            z = state.pose.position.z

            if not rt.initialized:
                yaw_guess = math.atan2(state.twist.linear.y, state.twist.linear.x) if horizontal_len(
                    state.twist.linear.x, state.twist.linear.y
                ) > 1e-6 else 0.0
                rt.yaw = yaw_guess + self.model_yaw_offset
                rt.pitch = 0.0
                rt.roll = 0.0
                rt.vx = state.twist.linear.x
                rt.vy = state.twist.linear.y
                rt.vz = state.twist.linear.z
                rt.initialized = True

            info[bird.name] = {
                'pos': (x, y, z),
                'vel': (rt.vx, rt.vy, rt.vz),
            }
        return info

    def compute_single_seek_velocity(self, bird, pos, vel):
        rt = self.runtime[bird.name]
        x, y, z = pos
        vx, vy, vz = vel

        tx = rt.target_x
        ty = rt.target_y
        tz = rt.target_z

        dist_to_target = math.sqrt((tx - x) ** 2 + (ty - y) ** 2 + (tz - z) ** 2)
        if dist_to_target < bird.arrival_threshold:
            self.pick_new_target(bird.name)
            tx = rt.target_x
            ty = rt.target_y
            tz = rt.target_z
            dist_to_target = math.sqrt((tx - x) ** 2 + (ty - y) ** 2 + (tz - z) ** 2)

        to_target = (tx - x, ty - y, tz - z)
        dir_to_target = vec_norm(to_target)

        slowdown_radius = self.slowdown_radius_single
        if dist_to_target < slowdown_radius:
            target_speed = bird.min_speed + (bird.max_speed - bird.min_speed) * (dist_to_target / slowdown_radius)
            target_speed = clamp(target_speed, bird.min_speed, bird.max_speed)
        else:
            target_speed = bird.max_speed

        desired_vel = vec_mul(dir_to_target, target_speed)

        if dist_to_target < slowdown_radius:
            desired_vel = vec_sub(desired_vel, vec_mul((vx, vy, vz), self.goal_damping_gain))

        desired_vel = (
            desired_vel[0],
            desired_vel[1],
            desired_vel[2] - vz * self.vertical_damping_gain
        )

        return vec_limit(desired_vel, bird.max_speed)

    def move_one_bird(self, bird, world_info):
        state = self.current_states[bird.name]
        rt = self.runtime[bird.name]

        x = state.pose.position.x
        y = state.pose.position.y
        z = state.pose.position.z
        pos = (x, y, z)
        vel = (rt.vx, rt.vy, rt.vz)

        desired_vel = self.compute_single_seek_velocity(bird, pos, vel)

        desired_vel = self.apply_boundary_soft_push(pos, desired_vel)
        desired_vel = vec_limit(desired_vel, bird.max_speed)

        speed = vec_len(desired_vel)
        if speed < 1e-6:
            return

        blend = 0.12
        rt.vx = (1.0 - blend) * rt.vx + blend * desired_vel[0]
        rt.vy = (1.0 - blend) * rt.vy + blend * desired_vel[1]
        rt.vz = (1.0 - blend) * rt.vz + blend * desired_vel[2]

        current_speed = math.sqrt(rt.vx * rt.vx + rt.vy * rt.vy + rt.vz * rt.vz)
        if current_speed > bird.max_speed:
            s = bird.max_speed / current_speed
            rt.vx *= s
            rt.vy *= s
            rt.vz *= s

        nx = clamp(x + rt.vx * self.dt, self.x_min, self.x_max)
        ny = clamp(y + rt.vy * self.dt, self.y_min, self.y_max)
        nz = clamp(z + rt.vz * self.dt, self.z_min, self.z_max)

        horizontal_speed = horizontal_len(rt.vx, rt.vy)
        desired_yaw_base = math.atan2(rt.vy, rt.vx)
        desired_yaw = desired_yaw_base + self.model_yaw_offset

        desired_pitch = math.atan2(rt.vz, max(horizontal_speed, 1e-6))
        desired_pitch = clamp(desired_pitch, -self.max_pitch_rad, self.max_pitch_rad)

        yaw_error = wrap_angle(desired_yaw - rt.yaw)
        max_yaw_step = self.max_yaw_rate * self.dt
        yaw_step = clamp(yaw_error, -max_yaw_step, max_yaw_step)
        rt.yaw = wrap_angle(rt.yaw + yaw_step)

        pitch_error = desired_pitch - rt.pitch
        max_pitch_step = self.max_pitch_rate * self.dt
        pitch_step = clamp(pitch_error, -max_pitch_step, max_pitch_step)
        rt.pitch = clamp(rt.pitch + pitch_step, -self.max_pitch_rad, self.max_pitch_rad)

        target_roll = clamp(
            -yaw_step / max(self.dt, 1e-6) * self.bank_from_turn_gain * 0.20,
            -self.max_roll_rad,
            self.max_roll_rad
        )
        roll_error = target_roll - rt.roll
        max_roll_step = self.max_roll_rate * self.dt
        roll_step = clamp(roll_error, -max_roll_step, max_roll_step)
        rt.roll = clamp(rt.roll + roll_step, -self.max_roll_rad, self.max_roll_rad)

        qx, qy, qz, qw = euler_to_quaternion(rt.roll, -rt.pitch, rt.yaw)

        new_state = EntityState()
        new_state.name = bird.name
        new_state.reference_frame = 'world'

        new_state.pose.position.x = nx
        new_state.pose.position.y = ny
        new_state.pose.position.z = nz

        new_state.pose.orientation.x = qx
        new_state.pose.orientation.y = qy
        new_state.pose.orientation.z = qz
        new_state.pose.orientation.w = qw

        new_state.twist.linear.x = rt.vx
        new_state.twist.linear.y = rt.vy
        new_state.twist.linear.z = rt.vz
        new_state.twist.angular.x = 0.0
        new_state.twist.angular.y = 0.0
        new_state.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = new_state
        self.current_states[bird.name] = new_state

        future = self.set_cli.call_async(req)
        future.add_done_callback(lambda fut, name=bird.name: self.on_set_done(fut, name))

    def apply_boundary_soft_push(self, pos, vel):
        x, y, z = pos
        vx, vy, vz = vel

        margin = 1.0
        push_gain = 1.2

        if x < self.x_min + margin:
            vx += (self.x_min + margin - x) * push_gain
        elif x > self.x_max - margin:
            vx -= (x - (self.x_max - margin)) * push_gain

        if y < self.y_min + margin:
            vy += (self.y_min + margin - y) * push_gain
        elif y > self.y_max - margin:
            vy -= (y - (self.y_max - margin)) * push_gain

        if z < self.z_min + 1.0:
            vz += (self.z_min + 1.0 - z) * 1.5
        elif z > self.z_max - 1.0:
            vz -= (z - (self.z_max - 1.0)) * 1.5

        radius = horizontal_len(x, y)
        if self.min_xy_radius > 0.0 and radius < self.min_xy_radius:
            if radius < 1e-6:
                vx += self.min_xy_radius * 1.4
            else:
                push = (self.min_xy_radius - radius) * 1.4
                vx += (x / radius) * push
                vy += (y / radius) * push

        return (vx, vy, vz)

    def publish_detection(self, pos):
        # 역할: Gazebo trial 종료 순간 context가 닫혀도 테스트용 bird manager가
        # traceback을 남기지 않도록 publish 경로를 방어한다.
        if not rclpy_ok():
            return
        visible_msg = Bool()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        try:
            if pos is None:
                visible_msg.data = False
                self.bird_visible_pub.publish(visible_msg)
                self.bird_pose_pub.publish(pose_msg)
                return

            visible_msg.data = True
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            pose_msg.pose.orientation.w = 1.0

            self.bird_visible_pub.publish(visible_msg)
            self.bird_pose_pub.publish(pose_msg)
        except Exception:
            return

    def start_tracking_if_needed(self):
        if self.tracking_start_wall is not None:
            return
        self.tracking_start_wall = time.time()
        self.tracking_start_text = time.strftime(
            '%Y-%m-%d %H:%M:%S %Z',
            time.localtime(self.tracking_start_wall),
        )
        self.get_logger().info(f'dynamic obstacle tracking started at {self.tracking_start_text}')

    def update_obstacle_metrics(self, now):
        for bird in self.birds:
            state = self.current_states.get(bird.name)
            if state is None:
                continue
            x = float(state.pose.position.x)
            y = float(state.pose.position.y)
            z = float(state.pose.position.z)
            rt = self.runtime[bird.name]
            prev = self.last_metric_samples.get(bird.name)

            if prev is None:
                dt = 0.0
                dx = 0.0
                dy = 0.0
                dz = 0.0
                speed = vec_len((rt.vx, rt.vy, rt.vz))
            else:
                prev_t, prev_x, prev_y, prev_z = prev
                dt = max(1e-6, now - prev_t)
                dx = x - prev_x
                dy = y - prev_y
                dz = z - prev_z
                speed = vec_len((dx, dy, dz)) / dt

            if horizontal_len(dx, dy) > 1e-4:
                direction_deg = math.degrees(math.atan2(dy, dx))
            elif horizontal_len(rt.vx, rt.vy) > 1e-4:
                direction_deg = math.degrees(math.atan2(rt.vy, rt.vx))
            else:
                direction_deg = 0.0

            self.obstacle_metrics[bird.name] = {
                'x': x,
                'y': y,
                'z': z,
                'dx': dx,
                'dy': dy,
                'dz': dz,
                'sample_dt': dt,
                'speed': speed,
                'direction_deg': direction_deg,
            }
            self.last_metric_samples[bird.name] = (now, x, y, z)

    def build_obstacle_status_text(self):
        names = [
            bird.name for bird in self.birds
            if bird.name in self.current_states and bird.name in self.obstacle_metrics
        ]
        if not names:
            return f'start={self.tracking_start_text} | count=0 | no moving dynamic obstacle'

        lines = [f'start={self.tracking_start_text} | count={len(names)} | interval=1.0s']
        for index, name in enumerate(names, start=1):
            metric = self.obstacle_metrics[name]
            lines.append(
                f'{index}:{name} '
                f'xyz=({metric["x"]:+.2f},{metric["y"]:+.2f},{metric["z"]:+.2f}) '
                f'd1s=({metric["dx"]:+.2f},{metric["dy"]:+.2f},{metric["dz"]:+.2f}) '
                f'speed={metric["speed"]:.2f}m/s '
                f'dir={metric["direction_deg"]:+.0f}deg'
            )
        return '\n'.join(lines)

    def publish_dynamic_obstacles(self):
        if not rclpy_ok():
            return
        if not self.publish_waver_detection_topics or self.dynamic_targets_pub is None:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        for bird in self.birds:
            state = self.current_states.get(bird.name)
            if state is None:
                continue
            msg.poses.append(state.pose)

        try:
            self.dynamic_targets_pub.publish(msg)
        except Exception:
            return

        now = time.monotonic()
        has_obstacles = bool(msg.poses)
        if has_obstacles:
            self.start_tracking_if_needed()

        if now - self.last_obstacle_status_time < 1.0 and self.last_obstacle_status_time > 0.0:
            return
        self.last_obstacle_status_time = now
        if has_obstacles:
            self.update_obstacle_metrics(now)

        status_text = self.build_obstacle_status_text()
        status_msg = String()
        status_msg.data = status_text
        height_msg = String()
        height_msg.data = status_text
        class_msg = String()
        class_msg.data = (
            f'bird tracking active: {len(msg.poses)} dynamic obstacle(s)'
            if has_obstacles
            else 'waiting for dynamic obstacle'
        )
        target_msg = String()
        target_msg.data = 'bird' if has_obstacles else 'unknown'
        confidence_msg = Float32()
        confidence_msg.data = 0.95 if has_obstacles else 0.0
        confirmed_msg = Bool()
        confirmed_msg.data = has_obstacles

        try:
            self.dynamic_obstacle_state_pub.publish(status_msg)
            self.height_filter_debug_pub.publish(height_msg)
            self.classification_state_pub.publish(class_msg)
            self.target_class_pub.publish(target_msg)
            self.target_confidence_pub.publish(confidence_msg)
            self.bird_confirmed_pub.publish(confirmed_msg)
        except Exception:
            return

    def publish_nearest_bird(self):
        if not self.current_states:
            self.publish_detection(None)
            return

        best_name = None
        best_z = float('inf')

        for bird in self.birds:
            if bird.name not in self.current_states:
                continue
            state = self.current_states[bird.name]
            z = state.pose.position.z
            if z < best_z:
                best_z = z
                best_name = bird.name

        if best_name is None:
            self.publish_detection(None)
            return

        state = self.current_states[best_name]
        pos = (
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z
        )
        self.publish_detection(pos)

    def on_set_done(self, future, name):
        try:
            res = future.result()
            if hasattr(res, 'success') and not res.success:
                self.get_logger().warn(f'[{name}] set_entity_state failed')
        except Exception as e:
            self.get_logger().error(f'[{name}] set_entity_state exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BirdManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 역할: launch 종료나 Gazebo trial cleanup 때 생기는 정상 shutdown을
        # traceback/exit code 1로 남기지 않아 반복 실험 로그를 깨끗하게 유지한다.
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass

        if rclpy_ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
