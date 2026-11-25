#!/usr/bin/env python3
"""
gimbal_bird_tracker.py — LiDAR 좌표로 탐색, YOLO 비전으로 정밀 추적하는 짐벌 카메라 조준

로봇의 3-LiDAR 검출 파이프라인(bird_detector_node)이 이미 발행하는 /bird_target
(odom 프레임 3D 위치)을 받아서, 벽면에 고정된 이 카메라가 그 방향을 먼저
겨냥한다(탐색). 그러다 YOLO가 실제로 화면에서 새(COCO 'bird')를 보면, 그 순간부터는
화면 중심 대비 픽셀 오차로 미세 보정하며 추적한다(정밀 추적). 즉:

    LiDAR 좌표 있음, 화면엔 아직 안 보임  -> LiDAR 방향으로 계속 겨냥 (SEARCH)
    화면에 실제로 보임                    -> 픽셀 오차로 미세 보정 (TRACK)
    둘 다 없음                            -> 마지막 목표각 유지 (COAST)

[구독]
  /bird_target          (geometry_msgs/PointStamped) - LiDAR 추적 3D 위치 (odom 프레임)
  /gimbal/image_raw     (sensor_msgs/Image)          - 짐벌 카메라 영상 (image_transport라
                                                        SDF <remapping>이 안 먹어서 실제
                                                        나가는 토픽 이름을 그대로 씀)
  /gimbal/joint_states  (sensor_msgs/JointState)     - 현재 yaw/pitch (오차를 더할 기준점)

[발행]
  /gimbal/yaw_cmd           (std_msgs/Float64) - 목표 yaw[rad]   (PID는 시뮬레이터가 처리)
  /gimbal/pitch_cmd         (std_msgs/Float64) - 목표 pitch[rad]
  /gimbal/camera_annotated  (sensor_msgs/Image) - bbox + 조준 상태 오버레이 (디버그용)

[카메라 스펙] (models/gimbal_classic/model.sdf 기준)
  1280x720, horizontal_fov = 2.0 rad

[좌표 변환 — LiDAR 좌표 → 짐벌 목표각]
  이 시스템은 odom 프레임을 world(Gazebo 절대좌표)와 사실상 동일하게 취급한다
  (로봇이 (0,0)에서 스폰되고 정적 TF들이 odom 프레임에 직접 붙는 기존 설계와 동일
  — bird_pursuit_node 등도 같은 가정을 쓴다). 그래서 /bird_target을 그대로
  카메라의 월드 고정 좌표(camera_x/y/z, 벽걸이 위치)에서 뺀 벡터로 방위각/고각을
  계산한다. 짐벌 자체가 마운트 시점에 mount_yaw_offset만큼 돌아가 있으므로
  (ugv_world_3lidar.world의 <include> pose yaw), 월드 방위각에서 그 오프셋을
  빼야 짐벌의 yaw 조인트 기준각이 된다.

[부호 규약 — 시뮬레이터에서 실제로 확인 전]
  yaw_sign/pitch_sign 파라미터로 부호를 뒤집을 수 있게 남겨뒀다. LiDAR 좌표
  방향으로 겨냥했는데 반대로 돌면 해당 파라미터를 -1로 뒤집을 것.

[실행 — hackathon_bringup.launch.py에 자동 포함됨]
  수동 실행이 필요하면:
  ros2 run gimbal_camera_pkg gimbal_bird_tracker.py --ros-args -p model_path:=yolo11l.pt
"""

import math
import queue
import threading

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge


COCO_BIRD_CLASS_ID = 14

CAM_W = 1280
CAM_H = 720
CAM_FOV_H = 2.0
CAM_FOV_V = CAM_FOV_H * CAM_H / CAM_W  # ≈ 1.125 rad

YAW_JOINT = 'cgo3_vertical_arm_joint'
PITCH_JOINT = 'cgo3_camera_joint'

# model.sdf의 pitch 조인트 관절 한계(-135°~+45°)와 동일. yaw는 continuous라
# 랩어라운드만 방지하는 용도로 넉넉하게 ±파이로 클램프.
YAW_MIN, YAW_MAX = -math.pi, math.pi
PITCH_MIN, PITCH_MAX = -2.35619, 0.7854


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GimbalBirdTracker(Node):
    def __init__(self):
        super().__init__('gimbal_bird_tracker')

        self.declare_parameter('model_path', 'yolo11l.pt')
        self.declare_parameter('conf_threshold', 0.20)
        # 화면 중앙 오차를 목표각에 반영하는 비율. 1.0이면 오차를 한 프레임만에
        # 전부 반영(과충전 진동 위험) — 물리 PID 자체는 스텝 응답 테스트로
        # 오버슈트 없이 안정적인 것 확인했으니(0.7rad 스텝도 링잉 없음) 조금
        # 공격적으로 잡아도 됨.
        self.declare_parameter('yaw_gain', 0.8)
        self.declare_parameter('pitch_gain', 0.8)
        # 짐벌 축 방향과 화면/월드 좌우·상하 부호가 실제로 맞는지 확인 전이라
        # 반전용으로 남겨둔 파라미터 (모듈 docstring 참고).
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('pitch_sign', 1.0)
        # 목표각 자체를 한 번 더 저역통과 — 검출/LiDAR 프레임 간 노이즈로
        # 목표각이 들쭉날쭉해지는 걸 완화.
        self.declare_parameter('smooth_alpha', 0.7)
        # 새 미검출 지속 시간이 이보다 길어지면 화면에 LOST로 표시.
        self.declare_parameter('lost_timeout_sec', 1.0)
        # 디지털 줌: horizontal_fov=2.0rad(광각)라 새가 화면에 너무 작게 잡혀서
        # YOLO가 못 알아보는 문제 때문에, 화면 중앙 부분만 잘라내 확대한 뒤
        # 그 잘라낸 이미지로 추론한다 (LiDAR가 이미 대략 중앙에 겨냥해준다는
        # 전제). 1.0=자르지 않음(원본 그대로), 0.5=가로세로 절반만 사용
        # (픽셀 밀도 4배), 값을 더 낮추면 더 확대되지만 그만큼 화면 벗어난
        # 새는 못 잡음.
        self.declare_parameter('zoom_crop_fraction', 0.3)
        # LiDAR 좌표(/bird_target)가 이 시간 이상 안 갱신되면 탐색 소스로 안 씀.
        self.declare_parameter('lidar_timeout_sec', 1.5)
        # 비전(TRACK)→LiDAR(SEARCH) 전환 히스테리시스. 검출 경계 거리에서는 YOLO가
        # 한 프레임씩 들쭉날쭉 놓치는 게 정상인데, 그때마다 바로 SEARCH로
        # 돌아가면 두 소스가 계산한 각도가 서로 달라서 목표각이 왔다갔다 하며
        # 짐벌이 크게 흔들리는 원인이 됐다 (특히 화면 가장자리에 걸리는 pitch
        # 한계 근처에서 더 심함). 이 값을 늘려서 잠깐 놓쳐도 TRACK을 유지하게 함.
        self.declare_parameter('vision_lost_hysteresis_sec', 0.7)
        # 짐벌이 고정된 벽걸이 위치의 월드(odom과 동일 취급) 좌표
        # (ugv_world_3lidar.world의 gimbal_classic include와 일치).
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 9.85)
        self.declare_parameter('camera_z', 1.8)
        # 짐벌이 마운트되며 돌아간 각도 (world 파일의 <include> pose yaw와 일치).
        self.declare_parameter('mount_yaw_offset', 1.5708)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.predict(np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8), verbose=False)
        self.get_logger().info('YOLO model ready.')

        self.bridge = CvBridge()

        # 최신 관절 피드백 (joint_states 콜백에서 갱신) — 오차를 더할 기준점.
        self._yaw_meas = 0.0
        self._pitch_meas = 0.0
        self._have_joint_state = False

        self._yaw_target = 0.0
        self._pitch_target = 0.0
        self._last_detect_time = 0.0
        # 마지막으로 검출된 새의 화면 좌표(전체 프레임 기준) — 줌 크롭을 항상
        # 화면 정중앙이 아니라 이 위치 중심으로 잘라서, 새가 정중앙에서 살짝만
        # 벗어나도 좁아진 크롭 밖으로 나가 검출을 통째로 놓치는 일을 줄인다.
        # (겨냥 목표 자체는 여전히 화면 정중앙 — 크롭 위치만 따라감.)
        self._last_bird_px = None  # (bx, by) or None

        # LiDAR 탐색 소스
        self._lidar_target = None    # (x, y, z) odom 프레임
        self._lidar_target_time = 0.0

        self.create_subscription(Image, '/gimbal/image_raw', self._image_cb, 10)
        self.create_subscription(JointState, '/gimbal/joint_states', self._joint_cb, 10)
        self.create_subscription(PointStamped, '/bird_target', self._bird_target_cb, 10)

        self.yaw_pub = self.create_publisher(Float64, '/gimbal/yaw_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/gimbal/pitch_cmd', 10)
        self.annotated_pub = self.create_publisher(Image, '/gimbal/camera_annotated', 10)

        # 검출 프레임과 무관하게 목표각을 재계산/재발행 (LiDAR 탐색 + keepalive).
        self.create_timer(1.0 / 20.0, self._control_step)

        # YOLO 추론이 카메라 FPS보다 느려도 ROS 콜백이 안 막히도록 별도 스레드.
        self._infer_queue: queue.Queue = queue.Queue(maxsize=1)
        self._infer_thread = threading.Thread(
            target=self._infer_loop, daemon=True, name='yolo_infer'
        )
        self._infer_thread.start()

        self.get_logger().info(
            f'GimbalBirdTracker started. conf={self.get_parameter("conf_threshold").value}, '
            f'fov_h={CAM_FOV_H:.3f}rad, img={CAM_W}x{CAM_H}, model={model_path}'
        )

    # =========================================================
    # ROS 콜백 (블로킹 금지)
    # =========================================================
    def _joint_cb(self, msg: JointState):
        try:
            yaw_idx = msg.name.index(YAW_JOINT)
            pitch_idx = msg.name.index(PITCH_JOINT)
        except ValueError:
            return
        self._yaw_meas = msg.position[yaw_idx]
        self._pitch_meas = msg.position[pitch_idx]
        if not self._have_joint_state:
            # 최초 피드백 수신 시 목표각을 현재 실측값으로 맞춰서 시작 스냅 방지.
            self._yaw_target = self._yaw_meas
            self._pitch_target = self._pitch_meas
            self._have_joint_state = True

    def _bird_target_cb(self, msg: PointStamped):
        self._lidar_target = (msg.point.x, msg.point.y, msg.point.z)
        self._lidar_target_time = self.get_clock().now().nanoseconds * 1e-9

    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        try:
            self._infer_queue.get_nowait()
        except queue.Empty:
            pass
        self._infer_queue.put((cv_img, msg))

    # =========================================================
    # 추론 전용 스레드 — 화면에 새가 실제로 보일 때만 목표각을 미세 보정한다.
    # =========================================================
    def _infer_loop(self):
        while rclpy.ok():
            try:
                cv_img, ros_msg = self._infer_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            self._process_frame(cv_img, ros_msg)

    def _process_frame(self, cv_img: np.ndarray, ros_msg: Image):
        h, w = cv_img.shape[:2]
        cx_center, cy_center = w // 2, h // 2
        conf_threshold = self.get_parameter('conf_threshold').value

        # 디지털 줌: crop_frac 비율만 잘라서 그 위에서 추론 (화면 전체를 그대로
        # 640으로 뭉개는 것보다 새가 차지하는 픽셀 비율이 커져서 작은 새도 더
        # 잘 잡힌다). 크롭 위치는 화면 정중앙 고정이 아니라 마지막으로 검출된
        # 새 위치를 따라간다 — 고정 중앙이면 새가 정중앙에서 조금만 벗어나도
        # (좁게 확대할수록 여유가 더 적어짐) 크롭 밖으로 나가 검출을 통째로
        # 놓쳐버려서, 오히려 "정중앙에 딱 있을 때만 보이고 살짝만 벗어나도
        # 놓치는" 악순환이 생겼다. 목표(겨냥) 자체는 아래에서 여전히 화면
        # 정중앙(cx_center, cy_center) 기준으로 계산하니, 크롭 위치만 따라가는
        # 건 "어디를 찾아볼지"만 바꾸는 것 — 겨냥 로직과는 무관.
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        lost_timeout = self.get_parameter('lost_timeout_sec').value
        if self._last_bird_px is not None and (now_sec - self._last_detect_time) < lost_timeout:
            crop_cx, crop_cy = self._last_bird_px
        else:
            crop_cx, crop_cy = cx_center, cy_center

        crop_frac = max(0.1, min(1.0, self.get_parameter('zoom_crop_fraction').value))
        cw, ch = int(w * crop_frac), int(h * crop_frac)
        x_off = int(max(0, min(w - cw, crop_cx - cw // 2)))
        y_off = int(max(0, min(h - ch, crop_cy - ch // 2)))
        crop = cv_img[y_off:y_off + ch, x_off:x_off + cw]

        results = self.model.track(
            crop,
            conf=conf_threshold,
            classes=[COCO_BIRD_CLASS_ID],
            verbose=False,
            imgsz=640,
            persist=True,
        )

        best_box = None
        best_conf = 0.0
        bird_detected = False

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                if cls_id != COCO_BIRD_CLASS_ID:
                    continue

                bird_detected = True
                # crop 좌표계 -> 원본 프레임 좌표계로 되돌림 (오버레이/오차계산은
                # 전부 원본 프레임 기준이라 여기서 한 번만 보정하면 나머지 코드는
                # 그대로 쓸 수 있다).
                cx1, cy1, cx2, cy2 = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = cx1 + x_off, cy1 + y_off, cx2 + x_off, cy2 + y_off
                if conf > best_conf:
                    best_conf = conf
                    best_box = (x1, y1, x2, y2, conf)

                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 220, 80), 2)
                label = f'bird {conf:.2f}'
                (lw, lh), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(cv_img, (x1, y1 - lh - base - 4), (x1 + lw, y1), (0, 220, 80), cv2.FILLED)
                cv2.putText(cv_img, label, (x1, y1 - base - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

        if best_box is not None:
            self._last_detect_time = self.get_clock().now().nanoseconds * 1e-9
            x1, y1, x2, y2, conf = best_box
            bx = (x1 + x2) / 2.0
            by = (y1 + y2) / 2.0
            self._last_bird_px = (bx, by)

            # 화면 중심 대비 픽셀 오차 -> 각도 오차 (양수 = 새가 왼쪽/위쪽)
            yaw_err = -(bx - cx_center) / (w / 2.0) * (CAM_FOV_H / 2.0)
            pitch_err = (cy_center - by) / (h / 2.0) * (CAM_FOV_V / 2.0)

            pitch_sign = self.get_parameter('pitch_sign').value
            yaw_gain = self.get_parameter('yaw_gain').value
            pitch_gain = self.get_parameter('pitch_gain').value

            base_yaw = self._yaw_meas if self._have_joint_state else self._yaw_target
            base_pitch = self._pitch_meas if self._have_joint_state else self._pitch_target

            # yaw는 여기서 yaw_sign을 안 쓴다 (SEARCH의 yaw_sign과는 다른 의미가
            # 되어버려서 하나로 겸용하면 안 됨). SEARCH 쪽에서 검증한 관계가
            # camera_world_yaw = mount_yaw_offset - joint_angle + pi 였다 —
            # joint_angle 계수가 -1. 즉 "화면 중심 기준으로 원하는 실세계 방향
            # 변화량"(yaw_err)만큼 camera_world_yaw를 바꾸려면 joint는 그
            # "반대 부호"만큼 바뀌어야 한다. bird_yolo_node.py(로봇 몸체 회전용,
            # 이런 축 반전이 없음)의 부호를 그대로 베껴서 여기 처음엔 '+'였는데,
            # 그러면 중앙으로 안 모이고 오히려 계속 밀려나는(양성 피드백) 문제가
            # 있었다 — '-'가 맞다.
            new_yaw = base_yaw - yaw_gain * yaw_err
            new_pitch = base_pitch + pitch_sign * pitch_gain * pitch_err
            new_yaw = max(YAW_MIN, min(YAW_MAX, new_yaw))
            new_pitch = max(PITCH_MIN, min(PITCH_MAX, new_pitch))

            alpha = self.get_parameter('smooth_alpha').value
            self._yaw_target = alpha * new_yaw + (1.0 - alpha) * self._yaw_target
            self._pitch_target = alpha * new_pitch + (1.0 - alpha) * self._pitch_target

            bxi, byi = int(bx), int(by)
            cv2.line(cv_img, (bxi - 8, byi), (bxi + 8, byi), (0, 255, 255), 1)
            cv2.line(cv_img, (bxi, byi - 8), (bxi, byi + 8), (0, 255, 255), 1)
            cv2.putText(
                cv_img, f'yaw_err:{math.degrees(yaw_err):.1f} pitch_err:{math.degrees(pitch_err):.1f}',
                (bxi - 60, byi + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 200, 0), 1, cv2.LINE_AA
            )

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        lost = (now_sec - self._last_detect_time) > self.get_parameter('lost_timeout_sec').value
        mode_text = 'TRACK' if bird_detected else ('LOST' if lost else 'COAST')
        mode_color = (0, 220, 80) if bird_detected else ((100, 100, 255) if lost else (0, 200, 255))

        cv2.line(cv_img, (cx_center - 15, cy_center), (cx_center + 15, cy_center), (200, 200, 200), 1)
        cv2.line(cv_img, (cx_center, cy_center - 15), (cx_center, cy_center + 15), (200, 200, 200), 1)
        # 실제 YOLO 추론에 쓰인 디지털 줌 영역 표시 (디버그용)
        cv2.rectangle(cv_img, (x_off, y_off), (x_off + cw, y_off + ch), (255, 140, 0), 1)
        cv2.putText(cv_img, mode_text, (w - 90, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 1, cv2.LINE_AA)
        cv2.putText(
            cv_img, f'target yaw:{math.degrees(self._yaw_target):+.1f} pitch:{math.degrees(self._pitch_target):+.1f}',
            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
        )

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = ros_msg.header
            self.annotated_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'publish error: {e}')

    # =========================================================
    # 제어 루프 (20Hz) — LiDAR 탐색 / 비전 추적 소스 선택 + 목표각 발행
    # =========================================================
    def _control_step(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        vision_fresh = (now_sec - self._last_detect_time) < self.get_parameter('vision_lost_hysteresis_sec').value
        lidar_fresh = (
            self._lidar_target is not None and
            (now_sec - self._lidar_target_time) < self.get_parameter('lidar_timeout_sec').value
        )

        # 비전이 이번 프레임에 실제로 새를 봤으면(_process_frame이 이미 목표각을
        # 갱신해뒀음) 그걸 그대로 쓴다 — 여기서 다시 계산하지 않음(정밀 추적 우선).
        if not vision_fresh and lidar_fresh:
            self._aim_at_lidar_target()

        self.yaw_pub.publish(Float64(data=self._yaw_target))
        self.pitch_pub.publish(Float64(data=self._pitch_target))

    def _aim_at_lidar_target(self):
        """LiDAR가 알려준 3D 좌표를 향해 겨냥 (아직 화면에 새가 안 보이는 탐색 단계)."""
        cam_x = self.get_parameter('camera_x').value
        cam_y = self.get_parameter('camera_y').value
        cam_z = self.get_parameter('camera_z').value
        mount_yaw_offset = self.get_parameter('mount_yaw_offset').value
        yaw_sign = self.get_parameter('yaw_sign').value
        pitch_sign = self.get_parameter('pitch_sign').value

        tx, ty, tz = self._lidar_target
        dx, dy, dz = tx - cam_x, ty - cam_y, tz - cam_z
        horiz_dist = math.hypot(dx, dy)
        if horiz_dist < 1e-3 and abs(dz) < 1e-3:
            return

        world_bearing = math.atan2(dy, dx)
        world_elev = math.atan2(dz, horiz_dist)

        # world_bearing -> yaw 조인트 각도 변환. 단순히 mount_yaw_offset을 빼는 게
        # 아니라 두 가지를 더 반영해야 한다 (실제로 이거 빠뜨려서 정면 표적이
        # 180도 반대로 겨냥되는 버그였음, model.sdf에서 직접 확인):
        #   1) yaw 조인트 축이 <axis><xyz>0 0 -1</xyz> — 표준 +Z 기준 회전의 반대
        #      부호라 조인트각 부호가 반대로 걸린다.
        #   2) 카메라 센서 자체 pose가 <pose>... 0 0 3.14</pose> — camera_link
        #      기준으로 이미 180도 돌아가 있다.
        # joint=0일 때 실제로 mount_yaw_offset 방향(방 안쪽)을 정확히 보고
        # 있다는 걸 라이브 캡처로 확인했고, 그 조건을 만족하는 유일한 변환은:
        #   camera_world_yaw = mount_yaw_offset - joint_angle + pi
        # 이를 world_bearing = camera_world_yaw로 놓고 풀면:
        new_yaw = yaw_sign * _wrap_pi(mount_yaw_offset + math.pi - world_bearing)
        new_pitch = pitch_sign * world_elev
        new_yaw = max(YAW_MIN, min(YAW_MAX, new_yaw))
        new_pitch = max(PITCH_MIN, min(PITCH_MAX, new_pitch))

        alpha = self.get_parameter('smooth_alpha').value
        self._yaw_target = alpha * new_yaw + (1.0 - alpha) * self._yaw_target
        self._pitch_target = alpha * new_pitch + (1.0 - alpha) * self._pitch_target


def main(args=None):
    rclpy.init(args=args)
    node = GimbalBirdTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
