#!/usr/bin/env python3

import math
import random
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


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
    is_swarm: bool
    max_speed: float
    min_speed: float
    arrival_threshold: float
    neighbor_radius: float
    separation_radius: float


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

    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0

    initialized: bool = False


class BirdManager(Node):
    def __init__(self):
        super().__init__('bird_manager')

        # 50x50 범위 (0,0 중심)
        self.x_min = -35.0
        self.x_max = 35.0
        self.y_min = -35.0
        self.y_max = 35.0

        # 낮춘 고도 범위
        self.z_min = 5.0
        self.z_max = 8.0

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

        self.slowdown_radius_single = 26.0
        self.slowdown_radius_swarm = 18.0
        self.goal_damping_gain = 0.18
        self.vertical_damping_gain = 0.20

        # 0,0 중심 swarm center
        self.swarm_center_x = 0.0
        self.swarm_center_y = 0.0
        self.swarm_center_z = 6.5
        self.swarm_target_x = 0.0
        self.swarm_target_y = 0.0
        self.swarm_target_z = 6.5

        # swarm 중심 이동 속도도 절반
        self.swarm_center_speed = 2.0
        self.swarm_center_arrival_threshold = 8.0

        self.weight_seek = 1.7
        self.weight_separation = 2.6
        self.weight_alignment = 1.2
        self.weight_cohesion = 0.8
        self.weight_center_follow = 1.6

        self.state_fail_count = {}

        self.birds = [
            BirdConfig('bird_single', False, 3.0, 0.75, 7.0, 0.0, 0.0),
            BirdConfig('bird_swarm_1', True, 2.5, 0.75, 4.5, 22.0, 7.0),
            BirdConfig('bird_swarm_2', True, 2.5, 0.75, 4.5, 22.0, 7.0),
            BirdConfig('bird_swarm_3', True, 2.5, 0.75, 4.5, 22.0, 7.0),
            BirdConfig('bird_swarm_4', True, 2.5, 0.75, 4.5, 22.0, 7.0),
            BirdConfig('bird_swarm_5', True, 2.5, 0.75, 4.5, 22.0, 7.0),
        ]

        self.runtime = {bird.name: BirdRuntime() for bird in self.birds}

        swarm_offsets = [
            (-6.0, 0.0, 0.0),
            (6.0, 0.0, 0.0),
            (-3.0, -5.5, 1.0),
            (3.0, -5.5, -1.0),
            (0.0, 6.0, 0.6),
        ]

        idx = 0
        for bird in self.birds:
            if bird.is_swarm:
                ox, oy, oz = swarm_offsets[idx]
                self.runtime[bird.name].offset_x = ox
                self.runtime[bird.name].offset_y = oy
                self.runtime[bird.name].offset_z = oz
                idx += 1

        self.set_service_name = self.find_service_name(
            preferred=['/gazebo/set_entity_state', '/set_entity_state'],
            service_type='gazebo_msgs/srv/SetEntityState'
        )
        self.get_service_name = self.find_service_name(
            preferred=['/gazebo/get_entity_state', '/get_entity_state'],
            service_type='gazebo_msgs/srv/GetEntityState'
        )

        self.set_cli = self.create_client(SetEntityState, self.set_service_name)
        self.get_cli = self.create_client(GetEntityState, self.get_service_name)

        while not self.set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.set_service_name} waiting...')
        while not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.get_service_name} waiting...')

        self.bird_pose_pub = self.create_publisher(PoseStamped, '/bird/nearest_pose', 10)
        self.bird_visible_pub = self.create_publisher(Bool, '/bird/visible', 10)

        self.pick_new_target('bird_single')
        self.pick_new_swarm_target()

        self.timer = self.create_timer(self.dt, self.update_all)
        self.get_logger().info('bird_manager started')

    def find_service_name(self, preferred, service_type):
        for _ in range(50):
            services = self.get_service_names_and_types()

            for name, types in services:
                if service_type in types and name in preferred:
                    return name

            for name, types in services:
                if service_type in types:
                    return name

            rclpy.spin_once(self, timeout_sec=0.2)

        raise RuntimeError(
            f'Could not find service type {service_type}. '
            f'Available services: {self.get_service_names_and_types()}'
        )

    def pick_random_target(self):
        return (
            random.uniform(self.x_min, self.x_max),
            random.uniform(self.y_min, self.y_max),
            random.uniform(self.z_min, self.z_max),
        )

    def pick_new_target(self, name):
        tx, ty, tz = self.pick_random_target()
        rt = self.runtime[name]
        rt.target_x = tx
        rt.target_y = ty
        rt.target_z = tz

    def pick_new_swarm_target(self):
        self.swarm_target_x, self.swarm_target_y, self.swarm_target_z = self.pick_random_target()

    def update_swarm_center(self):
        dx = self.swarm_target_x - self.swarm_center_x
        dy = self.swarm_target_y - self.swarm_center_y
        dz = self.swarm_target_z - self.swarm_center_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.swarm_center_arrival_threshold:
            self.pick_new_swarm_target()
            dx = self.swarm_target_x - self.swarm_center_x
            dy = self.swarm_target_y - self.swarm_center_y
            dz = self.swarm_target_z - self.swarm_center_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            return

        if dist < 30.0:
            target_speed = max(1.5, self.swarm_center_speed * (dist / 30.0))
        else:
            target_speed = self.swarm_center_speed

        step = min(target_speed * self.dt, dist)

        self.swarm_center_x = clamp(self.swarm_center_x + dx / dist * step, self.x_min, self.x_max)
        self.swarm_center_y = clamp(self.swarm_center_y + dy / dist * step, self.y_min, self.y_max)
        self.swarm_center_z = clamp(self.swarm_center_z + dz / dist * step, self.z_min, self.z_max)

    def update_all(self):
        if self.busy:
            return

        self.update_swarm_center()

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
            self.busy = False
            return

        world_info = self.build_world_info()

        for bird in self.birds:
            if bird.name not in self.current_states:
                continue
            self.move_one_bird(bird, world_info)

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

    def compute_swarm_velocity(self, bird, world_info):
        rt = self.runtime[bird.name]
        pos = world_info[bird.name]['pos']
        vel = world_info[bird.name]['vel']

        formation_target = (
            self.swarm_center_x + rt.offset_x,
            self.swarm_center_y + rt.offset_y,
            self.swarm_center_z + rt.offset_z
        )
        to_form = vec_sub(formation_target, pos)
        dist_to_form = vec_len(to_form)

        if dist_to_form < self.slowdown_radius_swarm:
            seek_speed = bird.min_speed + (bird.max_speed - bird.min_speed) * (dist_to_form / self.slowdown_radius_swarm)
            seek_speed = clamp(seek_speed, bird.min_speed, bird.max_speed)
        else:
            seek_speed = bird.max_speed

        seek = vec_mul(vec_norm(to_form), seek_speed)

        sep = (0.0, 0.0, 0.0)
        align_sum = (0.0, 0.0, 0.0)
        coh_sum = (0.0, 0.0, 0.0)
        count_align = 0
        count_coh = 0

        for other in self.birds:
            if other.name == bird.name or not other.is_swarm:
                continue
            if other.name not in world_info:
                continue

            other_pos = world_info[other.name]['pos']
            other_vel = world_info[other.name]['vel']

            diff = vec_sub(pos, other_pos)
            d = vec_len(diff)
            if d < 1e-6:
                continue

            if d < bird.separation_radius:
                push = vec_mul(vec_norm(diff), (bird.separation_radius - d) / bird.separation_radius)
                sep = vec_add(sep, push)

            if d < bird.neighbor_radius:
                align_sum = vec_add(align_sum, other_vel)
                coh_sum = vec_add(coh_sum, other_pos)
                count_align += 1
                count_coh += 1

        alignment = (0.0, 0.0, 0.0)
        cohesion = (0.0, 0.0, 0.0)

        if count_align > 0:
            avg_vel = vec_mul(align_sum, 1.0 / count_align)
            alignment = vec_sub(avg_vel, vel)

        if count_coh > 0:
            avg_pos = vec_mul(coh_sum, 1.0 / count_coh)
            cohesion = vec_sub(avg_pos, pos)

        center_follow = vec_sub(
            (self.swarm_center_x, self.swarm_center_y, self.swarm_center_z),
            pos
        )

        desired = (0.0, 0.0, 0.0)
        desired = vec_add(desired, vec_mul(seek, self.weight_seek))
        desired = vec_add(desired, vec_mul(vec_norm(sep), bird.max_speed * self.weight_separation))
        desired = vec_add(desired, vec_mul(vec_norm(alignment), bird.max_speed * self.weight_alignment))
        desired = vec_add(desired, vec_mul(vec_norm(cohesion), bird.max_speed * self.weight_cohesion))
        desired = vec_add(desired, vec_mul(vec_norm(center_follow), bird.max_speed * self.weight_center_follow * 0.5))

        if dist_to_form < self.slowdown_radius_swarm:
            desired = vec_sub(desired, vec_mul(vel, self.goal_damping_gain))

        desired = (
            desired[0],
            desired[1],
            desired[2] - vel[2] * self.vertical_damping_gain
        )

        limited = vec_limit(desired, bird.max_speed)
        speed = vec_len(limited)

        if speed < bird.min_speed:
            n = vec_norm(limited if speed > 1e-6 else to_form)
            limited = vec_mul(n, bird.min_speed)

        return limited

    def move_one_bird(self, bird, world_info):
        state = self.current_states[bird.name]
        rt = self.runtime[bird.name]

        x = state.pose.position.x
        y = state.pose.position.y
        z = state.pose.position.z
        pos = (x, y, z)
        vel = (rt.vx, rt.vy, rt.vz)

        if bird.is_swarm:
            desired_vel = self.compute_swarm_velocity(bird, world_info)
        else:
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

        future = self.set_cli.call_async(req)
        future.add_done_callback(lambda fut, name=bird.name: self.on_set_done(fut, name))

    def apply_boundary_soft_push(self, pos, vel):
        x, y, z = pos
        vx, vy, vz = vel

        # 50x50 범위에 맞춰 margin 축소
        margin = 5.0
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

        return (vx, vy, vz)

    def publish_detection(self, pos):
        visible_msg = Bool()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy_ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
