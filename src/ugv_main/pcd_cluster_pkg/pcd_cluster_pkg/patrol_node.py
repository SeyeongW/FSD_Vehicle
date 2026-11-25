#!/usr/bin/env python3
"""
patrol_node.py — atan2 기반 직접 순찰 주행 + 새 추적 상태 기계

[상태]
  PATROL  : 웨이포인트를 향해 직접 atan2로 방위각 조향 + 거리비례 전진 (Nav2 안 씀)
  TRACKING: /bird_detected True → 순찰 정지, bird_pursuit_node가 cmd_vel 제어

Nav2(TEB/코스트맵/BT)로 순찰 주행하던 걸 없앴다 — 이 맵은 정적이고 웨이포인트를
미리 다 아는 상황이라 Nav2의 진가(동적 장애물 실시간 회피)가 필요 없는데, 오히려
RotationShimController/velocity_smoother 등 여러 레이어가 서로 값을 다시 clamp하며
튜닝이 계속 꼬였다. bird_pursuit_node와 동일한 Pure Pursuit 패턴(방위각 직접 겨냥
+ 거리비례 전진 + LaserScan 정지/감속 게이트)으로 통일한다. 목표 yaw는 더 이상
요구하지 않는다 — TEB가 최종 yaw까지 맞추려고 후진을 택하던 문제가 애초에 이
요구조건 자체에서 나왔던 것이라, (x, y)만 순서대로 지나가면 되도록 뺐다.

AMCL은 그대로 유지한다 — map 프레임 위치를 알아야 웨이포인트까지 방위각/거리를
계산할 수 있어서다 (launch에서 map_server+amcl만 남기고 나머지 Nav2 스택은 뺐음).

[초기화]
  AMCL 초기 위치를 (0, 0, yaw=0)으로 자동 퍼블리시.
  로봇은 항상 (0,0)에서 +X 방향을 바라보고 스폰됨.

[토픽]
  /amcl_pose          (PoseWithCovarianceStamped, sub) — 로봇 map 프레임 위치
  /odom               (Odometry, sub) — 실측 각속도(감쇠항용)
  /scan               (LaserScan, sub) — 전방 장애물 회피
  /bird_detected      (Bool, sub) — 새 감지 여부
  /tracking_active    (Bool, pub) — bird_pursuit_node cmd_vel 허용 여부
  /initialpose        (PoseWithCovarianceStamped, pub) — AMCL 초기 위치
  /cmd_vel            (Twist, pub) — PATROL 상태일 때만 발행

[조향 PD 제어]
  순수 비례제어(방위각 오차만 보고 조향)만 쓰면 게인이 조금만 높아도 목표를
  넘어서고 반대로 꺾고 또 넘어서는 지속 진동(limit cycle)이 생긴다 — 직선
  구간에서 헤딩이 계속 크게 좌우로 흔들리던 원인이 이것이었다. 여기에 실측
  각속도(/odom)를 감쇠항으로 빼서, 목표에 가까워질수록(=이미 빠르게 돌고 있을
  수록) 제동이 걸려 못 넘어가게 한다.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _lawnmower_waypoints():
    """
    맵 내부(-6.67 ~ 6.67m, 20x20 방 기준 벽과 여유 확보) 지그재그 웨이포인트.
    X축 방향 트랙, 각 트랙은 Y축 방향으로 왕복. (x, y)만 있으면 되고 목표 yaw는
    안 씀 — 다음 웨이포인트로 향하는 방위각이 자연스럽게 헤딩을 정해준다.
    """
    x_tracks = [-6.67, -4.0, -1.33, 1.33, 4.0, 6.67]
    y_low, y_high = -6.67, 6.67
    wps = []
    for i, x in enumerate(x_tracks):
        if i % 2 == 0:
            wps.append((x, y_low))
            wps.append((x, y_high))
        else:
            wps.append((x, y_high))
            wps.append((x, y_low))
    return wps


PATROL   = 'patrol'
TRACKING = 'tracking'


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.waypoints = _lawnmower_waypoints()
        self.wp_idx    = 0
        self.state     = PATROL
        self._started       = False
        self._startup_count = 0
        # TRACKING 중 로봇이 새를 쫓아 경로와 무관한 곳으로 이동했다가 복귀할 때,
        # _turn_aim_point()의 "이전 웨이포인트에서 오는 중"이라는 진입방향 전제가
        # 깨져서 조준점이 엉뚱해진다 — 복귀 첫 구간만 오프셋 없이 실제 웨이포인트를
        # 직접 조준하도록 하는 플래그.
        self._resuming_from_tracking = False

        # ── 순찰 주행 파라미터 (bird_pursuit_node의 Pure Pursuit와 동일 패턴) ──
        # 코너에 딱 붙어야 다음 목표로 넘어가면 정확히 그 점을 찍고 꺾어야 해서
        # 지그재그처럼 보였음. 반경을 넉넉히 키워서 코너에 못 미쳐서부터 다음
        # 웨이포인트를 보고 미리 커브를 그리며 넘어가게 함 (Pure Pursuit의
        # look-ahead와 같은 효과).
        # 예전엔 1.5였는데, x_tracks 간격(2.67m)에 비해 너무 커서 짧은 가로
        # 구간(예: 2.67m 길이)에서 도착판정 반경이 구간 전체를 덮어버려 실제
        # 웨이포인트 근처에 가지도 않고 다음 구간으로 넘어가는 문제가 있었음.
        self.waypoint_tolerance = 0.6
        self.angular_gain = 1.5
        # 실측 각속도(오돔)에 비례해서 빼주는 감쇠항 — 목표에 가까워질수록
        # (=이미 빠르게 돌고 있을수록) 제동이 걸려 오버슈트를 막음
        self.angular_damping = 1.6
        self.max_angular_speed = 1.2
        self.linear_gain = 0.5
        self.max_linear_speed = 1.0
        # 코너를 바깥으로 도는 정도 — 급한 턴(U턴에 가까움)일수록 이 값 전체가,
        # 완만한 턴이면 그 비율만큼만 웨이포인트를 바깥쪽으로 밀어서 조준한다.
        # 예전 1.5m는 x_tracks 간격(2.67m)의 절반 이상이라 짧은 가로 구간에서
        # 조준점이 실제 웨이포인트에서 너무 많이 벗어났었음.
        self.corner_offset = 0.6

        # ── 전방 장애물 회피 (bird_pursuit_node와 동일) ──
        self.obstacle_stop_dist = 0.5
        self.obstacle_slow_dist = 1.2
        # 0.40rad(±23도)이던 걸 1.0rad(±57도)로 넓힘 — 코너 회전 오버슈트 중에는
        # 벽이 정면이 아니라 옆쪽에 있는 순간이 생기는데, 좁은 정면 콘 밖이면
        # 벽이 코앞이어도 정지/감속이 전혀 안 걸려서 충돌했던 문제 완화.
        self.obstacle_half_angle = 1.0

        # ── cmd_vel 스무딩 ──
        # 새 추적(bird_pursuit_node)과 달리 AMCL 위치는 깨끗해서 무겁게 스무딩할
        # 필요가 없고, 오히려 지연을 만들어 방위각 0 근처에서 못 멈추고 넘어갔다
        # 되돌아오는 진동(오버슈트)의 원인이 됐음 — 새 명령을 훨씬 더 즉각 반영
        self.smooth_alpha = 0.8
        self.last_cmd_angular_z = 0.0
        self.last_cmd_linear_x = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_angular_vel = 0.0
        self.have_pose = False
        self.latest_scan = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 5
        )
        self.tracking_active_pub = self.create_publisher(Bool, '/tracking_active', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # pointcloud_to_laserscan_node가 /scan을 BEST_EFFORT(qos_profile_sensor_data)로
        # 내므로 여기도 맞춰야 한다 — 안 맞으면 QoS 불일치로 메시지를 아예 못 받는다.
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.bird_sub = self.create_subscription(
            Bool, '/bird_detected', self._bird_cb, 10
        )
        # TRACKING→PATROL 전환 hysteresis: 연속 N프레임 미감지 후 복귀
        self._bird_lost_count = 0
        self._BIRD_LOST_THRESHOLD = 20  # ~2s (10Hz 기준)
        self._tracking_start_sec = 0.0  # 추적 시작 시각

        # 1초마다 startup 체크 (AMCL 준비 대기)
        self.create_timer(1.0, self._startup_tick)
        # 3초마다 현재 동작 상태 출력
        self.create_timer(3.0, self._status_log)
        # 순찰 제어 루프 (10Hz)
        self.create_timer(0.1, self._patrol_control_loop)

    # ─────────────────────────────────────────────────────────────
    # 시작 시퀀스
    # ─────────────────────────────────────────────────────────────
    def _startup_tick(self):
        if self._started:
            return
        self._startup_count += 1

        if self._startup_count == 5:
            self._publish_initial_pose()
            self.get_logger().info('[LOCALIZER] Initial pose published: (0, 0, yaw=0).')

        if self._startup_count >= 8:
            self._started = True
            self._set_tracking_active(False)
            self._status('[ 시스템 ] 준비 완료 — 순찰을 시작합니다.')

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation = yaw_to_quat(0.0)
        msg.pose.covariance[0]   = 0.25
        msg.pose.covariance[7]   = 0.25
        msg.pose.covariance[35]  = 0.07
        self.initial_pose_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────
    # 콜백
    # ─────────────────────────────────────────────────────────────
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.have_pose = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _odom_cb(self, msg: Odometry):
        self.robot_angular_vel = msg.twist.twist.angular.z

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    # ─────────────────────────────────────────────────────────────
    # 새 감지 콜백 → 상태 전환
    # ─────────────────────────────────────────────────────────────
    def _bird_cb(self, msg: Bool):
        if msg.data:
            self._bird_lost_count = 0
            if self.state == PATROL:
                self._tracking_start_sec = self.get_clock().now().nanoseconds * 1e-9
                self.state = TRACKING
                self._set_tracking_active(True)
                self._publish(0.0, 0.0)
                self._status('━' * 50)
                self._status('[ 조류 탐지 ] 추적 모드로 전환합니다.')
                self._status('━' * 50)
        else:
            if self.state == TRACKING:
                self._bird_lost_count += 1
                if self._bird_lost_count >= self._BIRD_LOST_THRESHOLD:
                    elapsed = (
                        self.get_clock().now().nanoseconds * 1e-9
                        - self._tracking_start_sec
                    )
                    self._bird_lost_count = 0
                    self.state = PATROL
                    self._set_tracking_active(False)
                    # 추적 중 원래 순찰 경로와 무관한 곳으로 이동했을 수 있으므로,
                    # 이전 wp_idx를 그대로 이어가지 않고 현재 위치에서 가장 가까운
                    # 웨이포인트부터 다시 시작한다.
                    if self.have_pose:
                        self.wp_idx = self._nearest_waypoint_idx()
                    self._resuming_from_tracking = True
                    self._status('━' * 50)
                    self._status(f'[ 조류 사라짐 ] 추적 {elapsed:.1f}초 후 순찰로 복귀합니다.')
                    self._status('━' * 50)

    def _nearest_waypoint_idx(self) -> int:
        best_i, best_d = 0, float('inf')
        for i, wp in enumerate(self.waypoints):
            d = math.hypot(wp[0] - self.robot_x, wp[1] - self.robot_y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _set_tracking_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.tracking_active_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────
    # 순찰 주행 (atan2 기반 직접 조향, Nav2 안 씀)
    # ─────────────────────────────────────────────────────────────
    def _turn_aim_point(self, wp_idx):
        """웨이포인트를 그대로 찍지 않고, 진입방향->진출방향 회전의 바깥쪽으로
        살짝 밀어낸 조준점을 계산한다. 이 점을 향해 가면 코너 안쪽을 파고들지
        않고 바깥으로 살짝 돌아 들어와서, 웨이포인트를 기준으로 약간 거리를 둔
        채 선회하며 다음 구간으로 자연스럽게 이어진다."""
        n = len(self.waypoints)
        prev_wp = self.waypoints[(wp_idx - 1) % n]
        curr_wp = self.waypoints[wp_idx]
        next_wp = self.waypoints[(wp_idx + 1) % n]

        in_dx, in_dy = curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]
        out_dx, out_dy = next_wp[0] - curr_wp[0], next_wp[1] - curr_wp[1]
        in_len = math.hypot(in_dx, in_dy) or 1.0
        out_len = math.hypot(out_dx, out_dy) or 1.0
        in_dx, in_dy = in_dx / in_len, in_dy / in_len
        out_dx, out_dy = out_dx / out_len, out_dy / out_len

        avg_dx, avg_dy = in_dx + out_dx, in_dy + out_dy
        avg_len = math.hypot(avg_dx, avg_dy)
        if avg_len < 1e-6:
            return curr_wp  # 거의 완전한 U턴 — 평균 진행방향이 안 정해짐, 그냥 원점 사용

        avg_dx, avg_dy = avg_dx / avg_len, avg_dy / avg_len

        # 진행방향 평균에 수직인 두 방향 중 회전 바깥쪽을 고름
        # (외적 z>0=좌회전이면 바깥쪽은 오른쪽 = 반시계 수직의 반대)
        cross_z = in_dx * out_dy - in_dy * out_dx
        perp_dx, perp_dy = -avg_dy, avg_dx
        if cross_z > 0:
            perp_dx, perp_dy = -perp_dx, -perp_dy

        # 턴이 급할수록(거의 U턴) 많이 밀어내고, 직진에 가까우면 거의 안 밀어냄
        turn_sharpness = (1.0 - (in_dx * out_dx + in_dy * out_dy)) / 2.0
        offset = self.corner_offset * turn_sharpness

        return (curr_wp[0] + perp_dx * offset, curr_wp[1] + perp_dy * offset)

    def _patrol_control_loop(self):
        # /tracking_active는 상태 전환 시점(_bird_cb)에도 발행되지만 그건 단발성이라,
        # 그 순간에 구독을 놓친 노드(예: 나중에 켠 data_logger_node)는 실제 상태를
        # 영원히 모르게 된다. 매 제어 주기(10Hz)마다 현재 상태를 같이 발행해서
        # 늦게 붙는 구독자도 곧바로 정확한 값을 받게 한다.
        if self._started:
            self._set_tracking_active(self.state == TRACKING)

        if self.state != PATROL or not self._started or not self.have_pose:
            return

        # TRACKING 복귀 직후 첫 구간은 "이전 웨이포인트에서 오는 중"이라는
        # _turn_aim_point()의 진입방향 전제가 깨져있으므로 오프셋 없이 실제
        # 웨이포인트를 직접 조준한다.
        if self._resuming_from_tracking:
            aim = self.waypoints[self.wp_idx]
        else:
            aim = self._turn_aim_point(self.wp_idx)
        dx = aim[0] - self.robot_x
        dy = aim[1] - self.robot_y
        dist = math.hypot(dx, dy)

        # 도착 판정은 조준점이 아니라 실제 웨이포인트까지 거리로 한다 — 조준점
        # 기준으로 하면 코너 오프셋만큼 앞당겨져서, 짧은 구간에서는 실제
        # 웨이포인트 근처에 가지도 않고 다음 구간으로 넘어가버렸었다.
        curr_wp = self.waypoints[self.wp_idx]
        wp_dist = math.hypot(curr_wp[0] - self.robot_x, curr_wp[1] - self.robot_y)

        if wp_dist < self.waypoint_tolerance:
            self._status(f'[ 순찰 기동 중 ] 웨이포인트 {self.wp_idx + 1} 도착')
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self._resuming_from_tracking = False
            return

        # 표적 방위각(로봇 기준) — bird_pursuit_node의 Pure Pursuit와 동일 계산
        cos_yaw, sin_yaw = math.cos(-self.robot_yaw), math.sin(-self.robot_yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        bearing = math.atan2(local_y, local_x)

        # PD: 방위각 오차(P) - 실측 각속도(D, 감쇠) — 순수 P만 쓰면 게인이 조금만
        # 높아도 목표를 넘어서고 반대로 꺾이는 지속 진동이 생김
        desired_angular = self.angular_gain * bearing - self.angular_damping * self.robot_angular_vel
        desired_angular = max(min(desired_angular, self.max_angular_speed), -self.max_angular_speed)

        # 정렬 여부를 이분법(정지/전진)으로 게이트하면 트랙 전환처럼 방위각이
        # 크게 바뀌어야 할 때 "완전정지 -> 최고속도 제자리회전 -> 급출발"로
        # 뚝뚝 끊겨서 물결치듯 보였다. cos(bearing/2)로 부드럽게 스케일링해서
        # (bearing=180도에서만 0, cos(bearing)보다 완만하게 떨어짐) 방위각이
        # 커도 어느 정도 전진을 유지한 채 넓게 커브를 그리도록 함.
        align_scale = math.cos(bearing / 2.0)

        # 위 align_scale은 "목표 방위각"만 보고 판단하는데, 제자리 회전 직후
        # 관성으로 실제로는 아직 빠르게 돌고 있는 상태에서 이 값만 믿고 바로
        # 전진을 확 넣으면 실제 헤딩이 목표를 지나쳐버려 엉뚱한 방향으로 직진하게
        # 됨. 실측 각속도(/odom)가 클수록 전진을 추가로 눌러서, 회전이 실제로
        # 잦아든 뒤에야 직진하도록 함.
        rotation_calm_scale = max(0.0, 1.0 - abs(self.robot_angular_vel) / self.max_angular_speed)

        desired_linear = (
            min(self.linear_gain * dist, self.max_linear_speed)
            * align_scale * rotation_calm_scale
        )

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

    def _status(self, text: str):
        self.status_pub.publish(String(data=text))

    def _status_log(self):
        if not self._started:
            return
        if self.state == PATROL:
            wp = self.waypoints[self.wp_idx]
            self._status(
                f'[ 순찰 기동 중 ] 웨이포인트 {self.wp_idx + 1}/{len(self.waypoints)} '
                f'({wp[0]:.1f}, {wp[1]:.1f}) 으로 이동 중'
            )
        elif self.state == TRACKING:
            elapsed = (
                self.get_clock().now().nanoseconds * 1e-9
                - self._tracking_start_sec
            )
            self._status(f'[ 조류 추적 중 ] 추적 경과 {elapsed:.1f}초')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
