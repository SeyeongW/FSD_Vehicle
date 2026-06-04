#!/usr/bin/env python3
"""
bird_yolo_node.py — YOLOv8 기반 카메라 조류 검출 + 바운딩 박스 + 방위각 퍼블리시

[구독 토픽]
  /pt_camera/image_raw     (sensor_msgs/Image)         - pt 카메라 원본 영상
  /bird_target             (geometry_msgs/PointStamped) - LiDAR 트랙 3D 위치 (참고용)

[퍼블리시 토픽]
  /pt_camera/image_annotated (sensor_msgs/Image)         - bbox 오버레이된 영상
  /bird_visual_bearing       (geometry_msgs/Vector3Stamped) - YOLO 방위각 오차
  /bird_detected             (std_msgs/Bool)              - 이번 프레임 조류 감지 여부

[/bird_visual_bearing 설명]
  cluster_node 가 LiDAR 트랙을 잃었을 때 이 방위각을 폴백으로 사용.
  vector.x : 수평 방위각 오차 (rad). 양수 = 조류가 카메라 왼쪽 → 로봇 좌회전 필요
  vector.y : 수직 방위각 오차 (rad). 양수 = 조류가 카메라 위쪽 → 틸트업 필요
  vector.z : YOLO 신뢰도 (0~1)

[카메라 FOV]
  SDF: horizontal_fov = 1.02974 rad (≈59°), 640×480px
  vertical_fov ≈ 1.02974 * 480/640 = 0.7723 rad (≈44°)

[스레드 구조]
  image_callback : ROS 스핀 스레드 — 프레임을 queue에 넣고 즉시 반환 (블로킹 없음)
  _infer_loop    : 전용 추론 스레드 — queue에서 최신 프레임을 꺼내 YOLO 실행 후 퍼블리시
  → YOLO 추론 속도가 카메라 FPS보다 느려도 ROS 토픽/제어 루프에 영향 없음
"""

import math
import queue
import threading

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge


COCO_BIRD_CLASS_ID = 14

CAM_FOV_H = 1.02974
CAM_W = 640
CAM_H = 480
CAM_FOV_V = CAM_FOV_H * CAM_H / CAM_W  # ≈ 0.7723 rad


class BirdYoloNode(Node):
    def __init__(self):
        super().__init__('bird_yolo_node')

        self.declare_parameter('model_path', 'yolov8s.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.predict(np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8), verbose=False)
        self.get_logger().info('YOLO model ready.')

        # ── 파라미터 ──
        self.conf_threshold = 0.20
        self.target_timeout_sec = 1.0

        # ── ROS 인터페이스 ──
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image, '/pt_camera/image_raw', self.image_callback, 10
        )
        self.target_sub = self.create_subscription(
            PointStamped, '/bird_target', self.target_callback, 10
        )

        self.img_pub = self.create_publisher(Image, '/pt_camera/image_annotated', 10)
        self.bearing_pub = self.create_publisher(Vector3Stamped, '/bird_visual_bearing', 10)
        self.detected_pub = self.create_publisher(Bool, '/bird_detected', 10)

        # ── 상태 ──
        self._lidar_target: PointStamped | None = None
        self._lidar_target_time = 0.0
        self._lidar_lock = threading.Lock()

        # ── 추론 스레드 ──
        # maxsize=1: 항상 최신 프레임만 유지. 추론이 느리면 오래된 프레임은 버림.
        self._infer_queue: queue.Queue = queue.Queue(maxsize=1)
        self._infer_thread = threading.Thread(
            target=self._infer_loop, daemon=True, name='yolo_infer'
        )
        self._infer_thread.start()

        self.get_logger().info(
            f'BirdYoloNode started (threaded). conf={self.conf_threshold}, '
            f'fov_h={CAM_FOV_H:.3f}rad, img={CAM_W}x{CAM_H}'
        )

    # =========================================================
    # ROS 콜백 (절대 블로킹하지 않음)
    # =========================================================
    def target_callback(self, msg: PointStamped):
        with self._lidar_lock:
            self._lidar_target = msg
            self._lidar_target_time = self.get_clock().now().nanoseconds * 1e-9

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # 큐가 이미 찼으면 오래된 프레임을 버리고 최신 프레임으로 교체
        try:
            self._infer_queue.get_nowait()
        except queue.Empty:
            pass
        self._infer_queue.put((cv_img, msg))

    # =========================================================
    # 추론 전용 스레드 (YOLO 실행)
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
        cx_center = w // 2
        cy_center = h // 2

        # ── YOLO 추론 ─────────────────────────────────────────
        results = self.model.track(
            cv_img,
            conf=self.conf_threshold,
            classes=[COCO_BIRD_CLASS_ID],
            verbose=False,
            imgsz=640,      # 카메라 해상도(640×480)와 동일 — 업스케일 없이 최적
            persist=True,   # 프레임 간 트랙 ID 유지
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
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                if conf > best_conf:
                    best_conf = conf
                    best_box = (x1, y1, x2, y2, conf)

                color = (0, 220, 80)
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), color, 2)

                label = f'bird {conf:.2f}'
                (lw, lh), base = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
                )
                cv2.rectangle(
                    cv_img, (x1, y1 - lh - base - 4), (x1 + lw, y1), color, cv2.FILLED
                )
                cv2.putText(
                    cv_img, label, (x1, y1 - base - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA
                )

                bx = (x1 + x2) // 2
                by = (y1 + y2) // 2
                cv2.line(cv_img, (bx - 8, by), (bx + 8, by), (0, 255, 255), 1)
                cv2.line(cv_img, (bx, by - 8), (bx, by + 8), (0, 255, 255), 1)

        # ── 방위각 계산 및 퍼블리시 ──────────────────────────────
        if best_box is not None:
            x1, y1, x2, y2, conf = best_box
            bx = (x1 + x2) / 2.0
            by = (y1 + y2) / 2.0

            yaw_err  = -(bx - cx_center) / (w / 2.0) * (CAM_FOV_H / 2.0)
            tilt_err =  (cy_center - by) / (h / 2.0) * (CAM_FOV_V / 2.0)

            bearing_msg = Vector3Stamped()
            bearing_msg.header.stamp = ros_msg.header.stamp
            bearing_msg.header.frame_id = 'pt_camera_link'
            bearing_msg.vector.x = float(yaw_err)
            bearing_msg.vector.y = float(tilt_err)
            bearing_msg.vector.z = float(conf)
            self.bearing_pub.publish(bearing_msg)

            bxi, byi = int(bx), int(by)
            bearing_text = (
                f'yaw:{math.degrees(yaw_err):.1f}° '
                f'tilt:{math.degrees(tilt_err):.1f}°'
            )
            cv2.putText(
                cv_img, bearing_text, (bxi - 40, byi + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 200, 0), 1, cv2.LINE_AA
            )

        # ── 화면 크로스헤어 ───────────────────────────────────────
        cv2.line(cv_img, (cx_center - 15, cy_center), (cx_center + 15, cy_center), (200, 200, 200), 1)
        cv2.line(cv_img, (cx_center, cy_center - 15), (cx_center, cy_center + 15), (200, 200, 200), 1)

        # ── LiDAR 잠금 상태 표시 ──────────────────────────────────
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        with self._lidar_lock:
            lidar_target = self._lidar_target
            lidar_time   = self._lidar_target_time

        lidar_fresh = (
            lidar_target is not None and
            (now_sec - lidar_time) < self.target_timeout_sec
        )

        if lidar_fresh:
            t = lidar_target
            info = f'LiDAR Lock  x:{t.point.x:.1f}  y:{t.point.y:.1f}  z:{t.point.z:.1f}m'
            cv2.putText(cv_img, info, (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.rectangle(cv_img, (0, 0), (w - 1, h - 1), (255, 100, 0), 2)
        elif not bird_detected:
            cv2.putText(cv_img, 'No bird', (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        mode_color = (0, 200, 255) if lidar_fresh else (100, 100, 255)
        mode_text  = 'LIDAR' if lidar_fresh else ('VISUAL' if bird_detected else 'SEARCH')
        cv2.putText(cv_img, mode_text, (w - 70, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, mode_color, 1, cv2.LINE_AA)

        # ── 퍼블리시 ──────────────────────────────────────────────
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = ros_msg.header
            self.img_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'publish error: {e}')

        detected_msg = Bool()
        detected_msg.data = bird_detected
        self.detected_pub.publish(detected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BirdYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
