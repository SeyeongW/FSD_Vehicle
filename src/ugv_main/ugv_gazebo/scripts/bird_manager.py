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

    # 원형 비행 파라미터
    CIRCLE_RADIUS  = 5.0    # m
    CIRCLE_SPEED   = 0.18   # rad/s  → 한 바퀴 ≈ 35s
    Z_CENTER       = 3.5    # m  — 최저 3.0m, 최고 4.0m
    Z_AMP          = 0.5    # m  — 고도 진동 진폭 (Z_CENTER ± Z_AMP)
    MODEL_YAW_OFF  = math.pi / 2.0   # 모델 기본 방향 오프셋

    # 맵 바운더리 (내부 여유 포함)
    X_MIN, X_MAX = -6.5, 6.5
    Y_MIN, Y_MAX = -6.5, 6.5

    def __init__(self):
        super().__init__('bird_manager')

        # 비행 속도 파라미터 (launch argument로 주입 가능)
        self.declare_parameter('circle_speed', self.CIRCLE_SPEED)
        self.CIRCLE_SPEED = self.get_parameter('circle_speed').value
        self.get_logger().info(
            f'[bird_manager] circle_speed={self.CIRCLE_SPEED:.3f} rad/s '
            f'(접선속도 {self.CIRCLE_RADIUS * self.CIRCLE_SPEED:.2f} m/s)'
        )

        pkg_share = get_package_share_directory('ugv_gazebo')
        sdf_path = os.path.join(pkg_share, 'models', 'bird', 'model.sdf')
        with open(sdf_path, 'r') as f:
            self.bird_sdf = f.read()

        self.bird_spawned = False
        self.circle_angle = 0.0     # 현재 원 위 각도 (rad)
        self._anim_timer  = None    # 비행 애니메이션 타이머

        # ── 서비스 클라이언트 ──────────────────────────────────────
        self.spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('Waiting for spawn/delete services...')
        self.spawn_cli.wait_for_service(timeout_sec=15.0)
        self.delete_cli.wait_for_service(timeout_sec=15.0)

        # SetEntityState: /set_entity_state 또는 /gazebo/set_entity_state
        self.set_cli = None
        for svc_name in ['/set_entity_state', '/gazebo/set_entity_state']:
            cli = self.create_client(SetEntityState, svc_name)
            if cli.wait_for_service(timeout_sec=3.0):
                self.set_cli = cli
                self.get_logger().info(f'SetEntityState ready: {svc_name}')
                break
        if self.set_cli is None:
            self.get_logger().error('SetEntityState service not found — bird will not move!')

        # ── 퍼블리셔 / 서브스크라이버 ─────────────────────────────
        self.bird_pose_pub    = self.create_publisher(PoseStamped, '/bird/nearest_pose', 10)
        self.bird_visible_pub = self.create_publisher(Bool,        '/bird/visible',      10)
        self.create_subscription(String, '/bird_command', self._on_command, 10)

        # 상태 퍼블리시 1 Hz
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            'bird_manager ready. '
            '"bird_in" → 스폰 + 원형 비행 시작  |  "bird_out" → 제거'
        )

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
        # 랜덤 진입 각도
        self.circle_angle = random.uniform(0.0, 2.0 * math.pi)
        x = self.CIRCLE_RADIUS * math.cos(self.circle_angle)
        y = self.CIRCLE_RADIUS * math.sin(self.circle_angle)
        z = self.Z_CENTER

        req = SpawnEntity.Request()
        req.name            = self.BIRD_NAME
        req.xml             = self.bird_sdf
        req.reference_frame = 'world'
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        req.initial_pose = pose

        future = self.spawn_cli.call_async(req)
        future.add_done_callback(self._on_spawn_done)
        self.get_logger().info(
            f'[CMD] bird_in — spawning at ({x:.2f}, {y:.2f}, {z:.2f}), '
            f'angle={math.degrees(self.circle_angle):.1f}°'
        )

    def _on_spawn_done(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
            return

        if res.success:
            self.bird_spawned = True
            self.get_logger().info('Bird spawned — starting circular flight.')
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
    #  원형 비행 애니메이션
    # ------------------------------------------------------------------ #
    def _animate_bird(self):
        if not self.bird_spawned or self.set_cli is None:
            return

        dt = 0.05  # 타이머 주기와 일치

        self.circle_angle += self.CIRCLE_SPEED * dt
        if self.circle_angle > 2.0 * math.pi:
            self.circle_angle -= 2.0 * math.pi

        θ = self.circle_angle
        x = self.CIRCLE_RADIUS * math.cos(θ)
        y = self.CIRCLE_RADIUS * math.sin(θ)
        z = self.Z_CENTER + self.Z_AMP * math.sin(θ * 2.0)

        # 속도 방향(CCW): (-sin θ, cos θ)
        # 기체 yaw = θ + π/2 + 모델오프셋
        yaw = θ + math.pi / 2.0 + self.MODEL_YAW_OFF
        pitch = math.atan2(
            self.Z_AMP * 2.0 * math.cos(θ * 2.0) * self.CIRCLE_SPEED,
            self.CIRCLE_RADIUS * self.CIRCLE_SPEED
        )
        pitch = max(-0.2, min(0.2, pitch))

        qx, qy, qz, qw = _euler_to_quat(0.0, -pitch, yaw)

        state = EntityState()
        state.name            = self.BIRD_NAME
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw

        # 속도 정보 (Gazebo 물리 엔진 보조용)
        vx = -self.CIRCLE_RADIUS * self.CIRCLE_SPEED * math.sin(θ)
        vy =  self.CIRCLE_RADIUS * self.CIRCLE_SPEED * math.cos(θ)
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.linear.z = self.Z_AMP * 2.0 * self.CIRCLE_SPEED * math.cos(θ * 2.0)

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
        θ = self.circle_angle
        visible_msg = Bool()
        pose_msg    = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        if self.bird_spawned:
            visible_msg.data = True
            pose_msg.pose.position.x = self.CIRCLE_RADIUS * math.cos(θ)
            pose_msg.pose.position.y = self.CIRCLE_RADIUS * math.sin(θ)
            pose_msg.pose.position.z = self.Z_CENTER + self.Z_AMP * math.sin(θ * 2.0)
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
