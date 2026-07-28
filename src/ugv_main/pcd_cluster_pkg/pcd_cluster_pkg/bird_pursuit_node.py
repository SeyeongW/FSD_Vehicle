#!/usr/bin/env python3
"""
bird_pursuit_node.py — PN(비례 항법) / Pure Pursuit 조류 추격 제어

cluster_node.py의 Pure Pursuit+P 추격 로직(track_target)을 대체.
표적의 위치/속도(bird_detector_node가 odom 프레임으로 발행)를 받아 /cmd_vel을
발행한다. steering_mode 파라미터로 두 조향 법칙 중 하나를 고른다 (해커톤
정량평가에서 PN vs Pure Pursuit 비교가 필요해서 둘 다 남겨둠):

  - 'pn' (기존 기본값이었음): 시선각(LOS) 변화율에 비례해 조향 (요격 코스 유지)
      LOS_rate = (R × V) / |R|^2   (R=상대위치, V=상대속도, 2D 외적/거리제곱)
      angular_cmd = pn_gain * LOS_rate
  - 'pure_pursuit' (기본값): 표적의 현재 방위각(bearing)을 직접 겨냥하는 단순 P제어
      angular_cmd = pp_angular_gain * bearing

새의 고도(z)는 조향/속도 계산 어디에도 안 쓴다 — 로봇은 지상차량이라 항상
새의 (x, y) 지상 투영점만 본다.

로봇 위치도 /odom(odom 프레임)에서 받는다 — bird_target과 같은 프레임이라 그대로
빼서 상대위치를 구할 수 있다. (전역 map 프레임의 AMCL 보정과는 무관한, 로봇 기준
로컬 추격 동작이라 map 프레임을 쓸 필요가 없음.)

patrol_node가 /tracking_active=True로 설정했을 때만 cmd_vel을 발행한다
(False일 때는 Nav2가 cmd_vel을 소유하므로 이 노드는 절대 끼어들지 않음).
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class BirdPursuitNode(Node):
    def __init__(self):
        super().__init__('bird_pursuit_node')

        self.create_subscription(PointStamped, '/bird_target', self._target_cb, 10)
        self.create_subscription(Vector3Stamped, '/bird_target_velocity', self._target_vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Bool, '/tracking_active', self._tracking_active_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── 조향 법칙 선택 ──
        self.declare_parameter('steering_mode', 'pure_pursuit')  # 'pn' or 'pure_pursuit'
        self.steering_mode = self.get_parameter('steering_mode').value

        # ── PN 게인 ──
        self.declare_parameter('pn_gain', 3.5)
        self.pn_gain = self.get_parameter('pn_gain').value

        # 순수 PN은 시선각 변화율(LOS_rate)이 우연히 0이면(새가 거의 일직선으로
        # 멀어지거나, 로봇이 정지해 있어 상대속도가 새 속도뿐인 경우) 조향값도
        # 0이 되어 스스로 돌 이유를 못 찾는다 — 방위각이 거의 반대(180도)라 전진도
        # 0으로 눌리는 상황과 겹치면 완전 정지 상태에 빠진다. 방위각 자체에
        # 비례하는 최소한의 성분(bias)을 살짝 섞어서 이 교착을 막는다.
        self.declare_parameter('pn_bearing_bias_gain', 0.6)
        self.pn_bearing_bias_gain = self.get_parameter('pn_bearing_bias_gain').value

        # ── Pure Pursuit 게인 (방위각 오차에 비례) ──
        self.declare_parameter('pp_angular_gain', 2.0)
        self.pp_angular_gain = self.get_parameter('pp_angular_gain').value

        # ── 전진 속도 (거리 기반 비례 제어, cluster_node.py 추격 파라미터 재사용) ──
        self.chase_target_dist = 2.0
        self.linear_gain = 0.5
        # 새 기본 비행속도(1.2m/s)보다 로봇 최고속도가 느리면 추적이 완벽해도
        # 물리적으로 못 따라잡아서 상향 (기존 0.5m/s는 새보다 느렸음)
        self.max_linear_speed = 1.8
        self.max_angular_speed = 2.0

        # ── 전방 장애물 회피 (cluster_node.py와 동일) ──
        self.obstacle_stop_dist = 0.5
        self.obstacle_slow_dist = 1.2
        self.obstacle_half_angle = 0.40

        # ── cmd_vel 스무딩 ──
        self.smooth_alpha = 0.4
        self.last_cmd_angular_z = 0.0
        self.last_cmd_linear_x = 0.0

        # ── 표적 신선도 체크: 이 시간(초) 이상 갱신 없으면 정지 ──
        self.target_timeout = 0.8

        # ── 상태 ──
        self.tracking_active = False
        self.target_pos = None       # (x,y,z) world
        self.target_vel = (0.0, 0.0, 0.0)
        self.target_stamp = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0

        self.latest_scan = None

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Bird pursuit node started (steering_mode={self.steering_mode}, '
            f'pn_gain={self.pn_gain}, pp_angular_gain={self.pp_angular_gain}).'
        )

    # =========================================================
    # 콜백
    # =========================================================
    def _tracking_active_cb(self, msg: Bool):
        was_active = self.tracking_active
        self.tracking_active = msg.data
        if was_active and not self.tracking_active:
            self.last_cmd_angular_z = 0.0
            self.last_cmd_linear_x = 0.0

    def _target_cb(self, msg: PointStamped):
        self.target_pos = (msg.point.x, msg.point.y, msg.point.z)
        self.target_stamp = time.monotonic()

    def _target_vel_cb(self, msg: Vector3Stamped):
        self.target_vel = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # body frame 전진속도 -> odom frame 속도 벡터 (횡슬립 없다고 가정)
        v_body = msg.twist.twist.linear.x
        self.robot_vx = v_body * math.cos(self.robot_yaw)
        self.robot_vy = v_body * math.sin(self.robot_yaw)

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # =========================================================
    # 제어 루프 (10Hz)
    # =========================================================
    def control_loop(self):
        if not self.tracking_active:
            return

        target_fresh = (
            self.target_pos is not None and
            (time.monotonic() - self.target_stamp) < self.target_timeout
        )
        if not target_fresh:
            self._publish(0.0, 0.0)
            return

        rx, ry = self.target_pos[0] - self.robot_x, self.target_pos[1] - self.robot_y
        vx, vy = self.target_vel[0] - self.robot_vx, self.target_vel[1] - self.robot_vy
        range2 = rx * rx + ry * ry
        dist_2d = math.sqrt(range2)

        if range2 < 1e-4:
            self._publish(0.0, 0.0)
            return

        # ── 표적 방위각(로봇 기준) — 두 조향 방식 다 필요 ──
        cos_yaw, sin_yaw = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        local_x = cos_yaw * rx - sin_yaw * ry
        local_y = sin_yaw * rx + cos_yaw * ry
        bearing = math.atan2(local_y, local_x)

        if self.steering_mode == 'pn':
            # 시선각 변화율(PN 본체) + 방위각 자체에 비례하는 최소 성분(bias) —
            # LOS_rate가 우연히 0이어도 새가 있는 쪽으로 최소한은 돌기 시작하게 함
            los_rate = (rx * vy - ry * vx) / range2
            desired_angular = self.pn_gain * los_rate + self.pn_bearing_bias_gain * bearing
        else:
            # Pure Pursuit: 표적 방위각을 직접 겨냥
            desired_angular = self.pp_angular_gain * bearing
        desired_angular = max(min(desired_angular, self.max_angular_speed), -self.max_angular_speed)

        # ── 전진속도: 거리 비례 + 방위각 오차에 따라 부드럽게 스케일링 ──
        # PN에서 "방위각 게이트를 아예 없앰"으로 했더니, 추적 시작 순간이나
        # LOS_rate가 잠깐 작을 때 로봇이 지금 향하고 있는 방향(새와 무관한 임의
        # 방향, 심하면 정반대)으로 전속력 돌진해서 벽에 부딪히는 사고가 났다.
        # 이분법 게이트(예전 Pure Pursuit용) 대신 cos(bearing/2)로 완만하게
        # 줄이면: 웬만큼 어긋나도 어느 정도는 전진하고(PN이 막혀버리던 문제 없음),
        # 방향이 거의 반대(180도 근처)일 땐 속도가 0에 가깝게 눌려서(돌진 사고 방지).
        align_scale = max(0.0, math.cos(bearing / 2.0))
        if dist_2d > self.chase_target_dist:
            desired_linear = min(self.linear_gain * (dist_2d - self.chase_target_dist), self.max_linear_speed) * align_scale
        else:
            desired_linear = 0.0

        # ── 전방 장애물 회피 ──
        obs_dist = self._forward_obstacle_dist()
        if obs_dist < self.obstacle_stop_dist:
            desired_linear = 0.0
        elif obs_dist < self.obstacle_slow_dist:
            scale = (obs_dist - self.obstacle_stop_dist) / (self.obstacle_slow_dist - self.obstacle_stop_dist)
            desired_linear *= scale

        self._publish(desired_linear, desired_angular)

    def _forward_obstacle_dist(self) -> float:
        if self.latest_scan is None:
            return float('inf')
        scan = self.latest_scan
        min_dist = float('inf')
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) < self.obstacle_half_angle:
                if scan.range_min < r < scan.range_max:
                    min_dist = min(min_dist, r)
            angle += scan.angle_increment
        return min_dist

    def _publish(self, desired_linear: float, desired_angular: float):
        a = self.smooth_alpha
        twist = Twist()
        twist.angular.z = a * desired_angular + (1.0 - a) * self.last_cmd_angular_z
        twist.linear.x = a * desired_linear + (1.0 - a) * self.last_cmd_linear_x

        self.last_cmd_angular_z = twist.angular.z
        self.last_cmd_linear_x = twist.linear.x
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BirdPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
