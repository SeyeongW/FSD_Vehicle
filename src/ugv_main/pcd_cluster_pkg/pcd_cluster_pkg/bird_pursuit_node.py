#!/usr/bin/env python3
"""
bird_pursuit_node.py — PN(비례 항법) / Pure Pursuit / SMC 조류 추격 제어

cluster_node.py의 Pure Pursuit+P 추격 로직(track_target)을 대체.
표적의 위치/속도(bird_detector_node가 odom 프레임으로 발행)를 받아 /cmd_vel을
발행한다. steering_mode 파라미터로 세 조향 법칙 중 하나를 고른다 (해커톤에서
PN vs Pure Pursuit 비교 후 Pure Pursuit 채택 — KCI 논문용으로 SMC를 추가해
3자 비교 중):

  - 'pn': 시선각(LOS) 변화율에 비례해 조향 (요격 코스 유지)
      LOS_rate = (R × V) / |R|^2   (R=상대위치, V=상대속도, 2D 외적/거리제곱)
      angular_cmd = pn_gain * LOS_rate
  - 'pure_pursuit' (기본값): 표적의 현재 방위각(bearing)을 직접 겨냥하는 단순 P제어
      angular_cmd = pp_angular_gain * bearing
  - 'smc': 슬라이딩 모드 제어 — Super-Twisting Algorithm(STA, 2차 SMC).
      슬라이딩 면은 s=bearing(미분 없음, 1차) — PN처럼 LOS_rate를 쓰지 않고
      방위각 자체만 쓴다(노이즈 있는 저속 비동기 탐지에서 미분이 증폭되는
      문제를 피하려는 의도, PN 실패 원인 분석 참고). 표적의 시선각 변화(λ̇)는
      로봇 입장에서 크기를 모르는 유계 외란으로 취급한다.
      기존 경계층(tanh) 근사 대신 STA로 반응:
      angular_cmd = sta_k1 * s / sqrt(|s| + sta_eps) + u1   (sqrt(|s|)*sign(s)의 정칙화 버전)
      u1(t+dt) = u1(t) + sta_k2 * sign(s) * dt      (적분 상태, tracking 시작시 0으로 리셋)
      sqrt(|s|)*sign(s)는 s=0 근처에서 기울기가 무한대라 작은 bearing 오차에도
      과민 반응했다(예: 0.1rad 오차에도 0.316 크기 신호) — s/sqrt(|s|+eps) 형태로
      완화. |s|>>eps에서는 원래 항과 거의 동일하고 s=0 근처만 완만해진다.
      단, 총 출력이 이미 max_angular_speed로 포화됐고 이번 sign(s)가 그 포화를
      더 키우는 방향이면 적분을 멈춘다(conditional integration 안티와인드업) —
      안 그러면 bearing이 이미 정렬됐는데도 예전 부호로 쌓인 u1이 안 풀려서
      로봇이 최대각속도로 계속 돌며 표적을 지나쳐버리는 오버슈트가 났었다.
      경계층 근사는 "정확히 0 대신 φ에 비례하는 정상상태 오차"로 채터링과
      맞바꾸는 타협이었지만, STA는 제어입력 자체가 연속(sign()의 불연속성이
      한 단계 위 적분항에만 있음)이라 채터링 없이도 s=0으로 유한시간에
      정확히 수렴하는 걸 이론적으로 보장한다 (Levant, 2차 슬라이딩 모드).
      데드밴드: |s|<bearing_deadband(거의 정렬)이면 비례항을 끄고 u1도 0.8배씩
      감쇠시켜 풀어준다 — 작은 오차에서도 u1이 쌓여 회전을 과하게 유지하는
      문제(표적한테 회전할 때 항상 너무 많이 도는 증상) 완화용.
      각속도 감쇠항: -sta_kd * robot_angular_vel(실측, /odom)을 추가로 뺀다.
      위 데드밴드/u1_limit은 적분(u1) windup 쪽 대응이고, 이건 별개 메커니즘 —
      큰 회전일수록 로봇이 관성으로 계속 도는 것 자체를 억제(patrol_node의
      P-D 순찰 제어와 동일 아이디어, 순수 P만 쓰면 오버슈트/진동 발생했었음).
      단 이걸 넣으면 더 이상 "순수 STA"가 아니라 STA+각속도 감쇠 하이브리드가
      된다 — 논문에서 STA를 이론 그대로 검증하려면 이 절충을 명시할 것.

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
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class BirdPursuitNode(Node):

    CONTROL_DT = 0.1  # 제어 루프 주기(초) — STA 적분 상태 업데이트에도 씀

    def __init__(self):
        super().__init__('bird_pursuit_node')

        self.create_subscription(PointStamped, '/bird_target', self._target_cb, 10)
        self.create_subscription(Vector3Stamped, '/bird_target_velocity', self._target_vel_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Bool, '/tracking_active', self._tracking_active_cb, 10)
        # pointcloud_to_laserscan_node가 /scan을 BEST_EFFORT(qos_profile_sensor_data)로
        # 내므로 여기도 맞춰야 한다 — 안 맞으면 QoS 불일치로 메시지를 아예 못 받는다.
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

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

        # ── SMC(STA) 게인 ── k1: sqrt(|s|) 비례항, k2: 적분(스위칭) 게인
        self.declare_parameter('sta_k1', 1.5)
        self.sta_k1 = self.get_parameter('sta_k1').value
        self.declare_parameter('sta_k2', 0.2)
        self.sta_k2 = self.get_parameter('sta_k2').value
        # sqrt(|s|)는 s=0 근처에서 기울기가 무한대라 작은 bearing 오차에도 과민
        # 반응한다(예: 0.1rad 오차에도 0.316 크기 신호) — s/sqrt(|s|+eps) 형태로
        # 완화(표준적인 STA 정칙화). |s|>>eps에서는 sqrt(|s|)*sign(s)와 거의
        # 동일하고, s=0 근처에서만 완만해진다.
        self.declare_parameter('sta_eps', 0.05)
        self.sta_eps = self.get_parameter('sta_eps').value
        # bearing이 이 값(rad) 미만이면 이미 정렬된 것으로 보고 적분(u1)을 감쇠시켜
        # 풀어준다 — 작은 오차에서도 u1이 계속 쌓여 회전을 과하게 유지하는 문제 완화.
        self.declare_parameter('bearing_deadband', 0.05)
        self.bearing_deadband = self.get_parameter('bearing_deadband').value
        # u1 자체의 상한(max_angular_speed=2.0의 절반) — 적분항 혼자서 최대각속도
        # 전부를 채우지 못하게 강제 제한. 데드밴드 감쇠는 |bearing|<deadband일
        # 때만 걸리는데, 빠르게 진동하며 0을 지나칠 땐 그 좁은 구간을 건너뛰어
        # 감쇠가 거의 안 걸릴 수 있다 — 그래도 회전 과다를 막기 위한 안전장치.
        self.declare_parameter('sta_u1_limit', 1.0)
        self.sta_u1_limit = self.get_parameter('sta_u1_limit').value
        self.sta_u1 = 0.0  # 적분 상태 — tracking_active가 꺼지면 리셋(와인드업 방지)
        # 실측 각속도(robot_angular_vel) 감쇠항 — 적분 windup과 별개로, 큰 회전 중
        # 로봇이 관성으로 계속 도는 것 자체를 억제. patrol_node의 P-D 순찰 제어와
        # 동일 아이디어(순수 P만 쓰면 진동/오버슈트 발생했었음).
        self.declare_parameter('sta_kd', 1.0)
        self.sta_kd = self.get_parameter('sta_kd').value

        # ── 전진 속도 (거리 기반 비례 제어, cluster_node.py 추격 파라미터 재사용) ──
        self.chase_target_dist = 0.5
        self.linear_gain = 0.7
        # 새 기본 비행속도(1.2m/s)보다 로봇 최고속도가 느리면 추적이 완벽해도
        # 물리적으로 못 따라잡아서 상향 (기존 0.5m/s는 새보다 느렸음)
        self.max_linear_speed = 2.0
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
        self.robot_angular_vel = 0.0

        self.latest_scan = None

        self.create_timer(self.CONTROL_DT, self.control_loop)

        self.get_logger().info(
            f'Bird pursuit node started (steering_mode={self.steering_mode}, '
            f'pn_gain={self.pn_gain}, pp_angular_gain={self.pp_angular_gain}, '
            f'sta_k1={self.sta_k1}, sta_k2={self.sta_k2}, bearing_deadband={self.bearing_deadband}, '
            f'sta_u1_limit={self.sta_u1_limit}, sta_kd={self.sta_kd}).'
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
            self.sta_u1 = 0.0

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
        self.robot_angular_vel = msg.twist.twist.angular.z

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
        elif self.steering_mode == 'smc':
            # Super-Twisting Algorithm — 슬라이딩 면 s=bearing(미분 없음).
            # sign(0)=0으로 둬서 정렬 완료 시 불필요한 스위칭 방지.
            s = bearing
            sign_s = math.copysign(1.0, s) if s != 0.0 else 0.0
            # 데드밴드: 이미 거의 정렬된(|s|<bearing_deadband) 상태면 비례항을 끄고
            # u1도 감쇠시켜 풀어준다 — 작은 오차에서도 u1이 계속 쌓여 회전을
            # 과하게 유지하는 문제 완화.
            if abs(s) < self.bearing_deadband:
                self.sta_u1 *= 0.8
                s_control = 0.0
                sign_s = 0.0
            else:
                s_control = s
            desired_angular = (
                self.sta_k1
                * (s_control / math.sqrt(abs(s_control) + self.sta_eps))
                + self.sta_u1
                - self.sta_kd * self.robot_angular_vel
            )
            # 안티와인드업(conditional integration): 총 출력이 이미 포화된 상태에서
            # 이번 sign(s)가 그 포화를 더 키우는 방향이면 적분을 멈춘다. bearing이
            # 이미 0을 지나 정렬됐는데도 예전 부호로 쌓인 u1이 안 풀려서 로봇이
            # 계속 최대각속도로 도는(오버슈트) 현상을 막기 위함. sign(s)가 반대로
            # 바뀌면(정렬 후 되돌아와야 할 때) 즉시 다시 적분을 재개해 풀려나온다.
            already_saturated = abs(desired_angular) >= self.max_angular_speed
            same_direction = already_saturated and (math.copysign(1.0, desired_angular) == sign_s)
            if not same_direction:
                self.sta_u1 += self.sta_k2 * sign_s * self.CONTROL_DT
            self.sta_u1 = max(min(self.sta_u1, self.sta_u1_limit), -self.sta_u1_limit)
        else:
            # Pure Pursuit: 표적 방위각을 직접 겨냥
            desired_angular = self.pp_angular_gain * bearing
        desired_angular = max(min(desired_angular, self.max_angular_speed), -self.max_angular_speed)

        # ── 전진속도: 거리 비례 + 방위각 오차에 따라 부드럽게 스케일링 ──
        # PN에서 "방위각 게이트를 아예 없앰"으로 했더니, 추적 시작 순간이나
        # LOS_rate가 잠깐 작을 때 로봇이 지금 향하고 있는 방향(새와 무관한 임의
        # 방향, 심하면 정반대)으로 전속력 돌진해서 벽에 부딪히는 사고가 났다.
        # 이분법 게이트(예전 Pure Pursuit용) 대신 cos(bearing)으로 스케일링하면:
        # 웬만큼 어긋나도 어느 정도는 전진하고(PN이 막혀버리던 문제 없음), 방위각
        # 오차가 90도만 넘어도 전진속도가 0으로 눌린다. (기존 cos(bearing/2)는
        # 180도 근처에서만 0이 되어 새가 급선회할 때 전진속도가 안 죽어 회전반경이
        # 커지는 문제가 있었음 — chase_target_dist를 0.5m로 줄이면서 평소 순항
        # 속도 자체가 높아져 이 문제가 두드러졌다.)
        align_scale = max(0.0, math.cos(bearing))
        # 실측 각속도가 클수록(아직 세게 도는 중) 전진을 추가로 억제 — 최대각속도로
        # 계속 돌면서 동시에 고속 직진하는 조합이 물리적으로 로봇을 넘어뜨렸던
        # 문제(patrol_node의 rotation_calm_scale과 동일 아이디어) 방지.
        rotation_calm_scale = max(0.0, 1.0 - abs(self.robot_angular_vel) / self.max_angular_speed)
        if dist_2d > self.chase_target_dist:
            desired_linear = (
                min(self.linear_gain * (dist_2d - self.chase_target_dist), self.max_linear_speed)
                * align_scale * rotation_calm_scale
            )
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
