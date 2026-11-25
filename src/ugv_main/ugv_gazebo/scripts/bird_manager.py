#!/usr/bin/env python3

import math
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory


def _euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # qx
        cr * sp * cy + sr * cp * sy,  # qy
        cr * cp * sy - sr * sp * cy,  # qz
        cr * cp * cy + sr * sp * sy,  # qw
    )


class BirdManager(Node):

    BIRD_NAME = 'bird_single'

    # 랜덤 방향전환 비행 파라미터
    FLIGHT_SPEED     = 1.2   # m/s — 목표점을 향한 이동 속도
    WAYPOINT_RADIUS  = 0.5   # m  — 이 거리 이내로 도달하면 새 목표점 재선정
    MODEL_YAW_OFF    = math.pi / 2.0   # 모델 기본 방향 오프셋

    # 최대 선회율(deg/s) — 실제 새는 웨이포인트를 반경 0으로 순간 전환하지 못하고
    # 유한한 선회반경을 그린다. 이 상한이 없으면(기존 구현) 웨이포인트 도달마다
    # 속도벡터가 그 자리에서 즉시 홱 꺾여서, PN 항법이 절대 대응 못 하는
    # 비현실적인 반경 0 회전이 매번 발생했다. 120deg/s는 급하게 쫓기는 상황도
    # 표현할 만큼 날카롭지만, 여전히 물리적으로 유한한 값.
    MAX_TURN_RATE_DEG = 120.0

    # 맵 바운더리 (20x20 방, 내부 여유 포함) — 목표점은 이 범위 안에서만 랜덤 선정
    X_MIN, X_MAX = -9.0, 9.0
    Y_MIN, Y_MAX = -9.0, 9.0
    # ground_z_limit(2.1m) 위, 벽 높이(2m)보다 확실히 높은 고도대
    Z_MIN, Z_MAX = 2.5, 5.0

    def __init__(self):
        super().__init__('bird_manager')

        # 비행 속도 / 최대 선회율 파라미터 (launch argument로 주입 가능)
        self.declare_parameter('flight_speed', self.FLIGHT_SPEED)
        self.FLIGHT_SPEED = self.get_parameter('flight_speed').value
        self.declare_parameter('bird_max_turn_rate_deg', self.MAX_TURN_RATE_DEG)
        self.MAX_TURN_RATE = math.radians(self.get_parameter('bird_max_turn_rate_deg').value)
        self.get_logger().info(
            f'[bird_manager] flight_speed={self.FLIGHT_SPEED:.3f} m/s, '
            f'max_turn_rate={math.degrees(self.MAX_TURN_RATE):.1f} deg/s (랜덤 방향전환 비행)'
        )

        # 졸업논문 실험용 파라미터 (기본값은 기존 동작과 동일)
        #   bird_sdf_path : 다른 크기의 새 모델 사용 ('default' = ugv_gazebo/models/bird)
        #   random_seed   : >=0이면 비행 궤적 재현 (단일/다중 LiDAR를 같은 궤적으로 비교)
        #   status_rate_hz: /bird/nearest_pose(GT) 발행 주기 — 위치 오차 평가엔 1Hz가 부족
        self.declare_parameter('bird_sdf_path', 'default')
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('status_rate_hz', 1.0)
        seed = self.get_parameter('random_seed').value
        if seed >= 0:
            random.seed(seed)

        pkg_share = get_package_share_directory('ugv_gazebo')
        sdf_path = self.get_parameter('bird_sdf_path').value
        if sdf_path == 'default':
            sdf_path = os.path.join(pkg_share, 'models', 'bird', 'model.sdf')
        with open(sdf_path, 'r') as f:
            self.bird_sdf = f.read()

        self.bird_spawned = False
        self.pos     = [0.0, 0.0, 0.0]   # 현재 위치 (world)
        self.target  = [0.0, 0.0, 0.0]   # 현재 목표점 (world)
        self.yaw     = 0.0
        self.heading = 0.0   # 현재 수평 이동 방향(rad) — 선회율 상한 적용 대상
        self._anim_timer = None    # 비행 애니메이션 타이머

        # ── 서비스 클라이언트 ──────────────────────────────────────
        self.spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('Waiting for spawn/delete services...')
        self.spawn_cli.wait_for_service(timeout_sec=15.0)
        self.delete_cli.wait_for_service(timeout_sec=15.0)

        # SetEntityState: 클라이언트는 서비스가 아직 안 떠 있어도 즉시 생성 가능하고
        # ROS2가 알아서 나중에 자동으로 연결해준다(DDS discovery). wait_for_service로
        # 한 번 확인해서 실패하면 self.set_cli를 영영 None으로 남기던 예전 방식은
        # 시스템이 느려서 시작 시점에 gzserver의 상태 서비스가 아직 준비 안 됐을 때
        # (오늘 계속 겪은 문제) 그 세션 내내 새가 스폰만 되고 절대 안 움직이는 버그였다.
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        if not self.set_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/set_entity_state 서비스가 아직 안 떠 있음 — 준비되는 대로 자동 연결됨'
            )

        # ── 퍼블리셔 / 서브스크라이버 ─────────────────────────────
        self.bird_pose_pub    = self.create_publisher(PoseStamped, '/bird/nearest_pose', 10)
        self.bird_visible_pub = self.create_publisher(Bool,        '/bird/visible',      10)
        self.create_subscription(String, '/bird_command', self._on_command, 10)

        # 상태 퍼블리시 (기본 1 Hz)
        self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self._publish_status)

        self.get_logger().info(
            'bird_manager ready. '
            '"bird_in" → 스폰 + 랜덤 방향전환 비행 시작  |  "bird_out" → 제거'
        )

    def _random_point(self):
        return [
            random.uniform(self.X_MIN, self.X_MAX),
            random.uniform(self.Y_MIN, self.Y_MAX),
            random.uniform(self.Z_MIN, self.Z_MAX),
        ]

    # ------------------------------------------------------------------ #
    #  명령 처리
    # ------------------------------------------------------------------ #
    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'bird_in':
            if self.bird_spawned:
                self.get_logger().info('[CMD] bird_in — already spawned, ignored.')
            else:
                self._spawn_bird()
        elif cmd == 'bird_out':
            if not self.bird_spawned:
                self.get_logger().info('[CMD] bird_out — no bird to remove.')
            else:
                self._delete_bird()

    # ------------------------------------------------------------------ #
    #  스폰
    # ------------------------------------------------------------------ #
    def _spawn_bird(self):
        self.pos    = self._random_point()
        self.target = self._random_point()
        # 스폰 직후 첫 목표는 선회율 제한 없이 바로 그 방향을 보도록 초기화
        self.heading = math.atan2(
            self.target[1] - self.pos[1], self.target[0] - self.pos[0]
        )

        req = SpawnEntity.Request()
        req.name            = self.BIRD_NAME
        req.xml             = self.bird_sdf
        req.reference_frame = 'world'
        pose = Pose()
        pose.position.x = float(self.pos[0])
        pose.position.y = float(self.pos[1])
        pose.position.z = float(self.pos[2])
        pose.orientation.w = 1.0
        req.initial_pose = pose

        future = self.spawn_cli.call_async(req)
        future.add_done_callback(self._on_spawn_done)
        self.get_logger().info(
            f'[CMD] bird_in — spawning at ({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f}), '
            f'target=({self.target[0]:.2f}, {self.target[1]:.2f}, {self.target[2]:.2f})'
        )

    def _on_spawn_done(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
            return

        if res.success:
            self.bird_spawned = True
            self.get_logger().info('Bird spawned — starting random-direction flight.')
            self._anim_timer = self.create_timer(0.05, self._animate_bird)
        else:
            self.get_logger().error(f'Spawn rejected: {res.status_message}')

    # ------------------------------------------------------------------ #
    #  삭제
    # ------------------------------------------------------------------ #
    def _delete_bird(self):
        if self._anim_timer is not None:
            self._anim_timer.cancel()
            self._anim_timer = None

        req = DeleteEntity.Request()
        req.name = self.BIRD_NAME
        future = self.delete_cli.call_async(req)
        future.add_done_callback(self._on_delete_done)
        self.get_logger().info('[CMD] bird_out — removing bird...')

    def _on_delete_done(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Delete failed: {e}')
            return

        if res.success:
            self.bird_spawned = False
            self.get_logger().info('Bird removed.')
        else:
            self.get_logger().error(f'Delete rejected: {res.status_message}')

    # ------------------------------------------------------------------ #
    #  랜덤 방향전환 비행 애니메이션
    # ------------------------------------------------------------------ #
    def _animate_bird(self):
        if not self.bird_spawned:
            return

        dt = 0.05  # 타이머 주기와 일치

        dx = self.target[0] - self.pos[0]
        dy = self.target[1] - self.pos[1]
        dz = self.target[2] - self.pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.WAYPOINT_RADIUS:
            # 목표점 도달 → 새 목표점 재선정 (랜덤 방향 전환)
            self.target = self._random_point()
            dx = self.target[0] - self.pos[0]
            dy = self.target[1] - self.pos[1]
            dz = self.target[2] - self.pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        step = min(self.FLIGHT_SPEED * dt, dist) if dist > 1e-6 else 0.0

        # 수평 방향은 목표점을 향해 즉시 스냅하지 않고, 현재 헤딩에서
        # 최대 선회율(MAX_TURN_RATE)만큼만 회전시킨다 — 유한한 선회반경 구현.
        # 수직(z) 성분은 기존처럼 목표까지의 비율을 그대로 따라간다(피치 클램프로 이미 제한됨).
        desired_heading = math.atan2(dy, dx) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else self.heading
        heading_err = math.atan2(
            math.sin(desired_heading - self.heading), math.cos(desired_heading - self.heading)
        )
        max_step = self.MAX_TURN_RATE * dt
        self.heading += max(-max_step, min(max_step, heading_err))

        uz = dz / dist if dist > 1e-6 else 0.0
        ux, uy = math.cos(self.heading), math.sin(self.heading)
        norm = math.sqrt(ux * ux + uy * uy + uz * uz)
        if norm > 1e-9:
            ux, uy, uz = ux / norm, uy / norm, uz / norm

        self.pos[0] += ux * step
        self.pos[1] += uy * step
        self.pos[2] += uz * step

        vx, vy, vz = ux * self.FLIGHT_SPEED, uy * self.FLIGHT_SPEED, uz * self.FLIGHT_SPEED

        # 기체 방향 = 이동 방향
        horiz = math.sqrt(vx * vx + vy * vy)
        yaw = self.heading + self.MODEL_YAW_OFF
        pitch = math.atan2(vz, horiz) if horiz > 1e-6 else 0.0
        pitch = max(-0.3, min(0.3, pitch))
        self.yaw = yaw

        qx, qy, qz, qw = _euler_to_quat(0.0, -pitch, yaw)

        state = EntityState()
        state.name            = self.BIRD_NAME
        state.reference_frame = 'world'
        state.pose.position.x = self.pos[0]
        state.pose.position.y = self.pos[1]
        state.pose.position.z = self.pos[2]
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.linear.z = vz

        req = SetEntityState.Request()
        req.state = state
        future = self.set_cli.call_async(req)
        future.add_done_callback(self._on_anim_set_done)

    def _on_anim_set_done(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn(
                    f'SetEntityState failed: {res.status_message}',
                    throttle_duration_sec=5.0
                )
        except Exception as e:
            self.get_logger().error(
                f'SetEntityState exception: {e}',
                throttle_duration_sec=5.0
            )

    # ------------------------------------------------------------------ #
    #  상태 퍼블리시
    # ------------------------------------------------------------------ #
    def _publish_status(self):
        visible_msg = Bool()
        pose_msg    = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        if self.bird_spawned:
            visible_msg.data = True
            pose_msg.pose.position.x = self.pos[0]
            pose_msg.pose.position.y = self.pos[1]
            pose_msg.pose.position.z = self.pos[2]
            pose_msg.pose.orientation.w = 1.0
        else:
            visible_msg.data = False

        self.bird_visible_pub.publish(visible_msg)
        self.bird_pose_pub.publish(pose_msg)


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
