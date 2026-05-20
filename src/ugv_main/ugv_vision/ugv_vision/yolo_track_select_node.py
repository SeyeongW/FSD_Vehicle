"""Click-to-lock YOLO tracker (GUI).

Opens the camera, runs YOLO tracking with persistent IDs, and shows an
OpenCV window. Click a detection box to lock onto that track; click empty
space (or press 'r') to release.

Unlike a pure-camera tracker, this node does NOT drive the gimbal. It
publishes the locked target's image-space bounding box on
/perception/locked_bbox. The lidar_camera_fusion_node projects LiDAR
clusters into the image, finds the cluster inside that box, and emits its
3D position on /target/position, which the gimbal_controller_node aims at.

The motion-prediction / re-lock logic is kept because it maintains the
track lock through brief detection misses, which keeps the bbox flowing to
the fusion node.
"""
from __future__ import annotations

import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from ultralytics import YOLO


class YoloTrackSelectNode(Node):
    WINDOW_NAME = 'YOLO Detector (click to lock)'

    def __init__(self):
        super().__init__('yolo_track_select_node')

        self.declare_parameter('model_path', 'yolo11s.pt')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('conf', 0.20)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('fps', 30)
        self.declare_parameter('lost_wait_seconds', 3.0)
        self.declare_parameter('show_window', True)

        model_path = self.get_parameter('model_path').value
        camera_index = int(self.get_parameter('camera_index').value)
        self.frame_w = int(self.get_parameter('frame_width').value)
        self.frame_h = int(self.get_parameter('frame_height').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.fps = int(self.get_parameter('fps').value)
        self.wait_seconds = float(self.get_parameter('lost_wait_seconds').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # Camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera index {camera_index}!')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # ROS interfaces
        self.id_subscription = self.create_subscription(
            Int32, '/perception/set_target_id', self._target_id_callback, 10)
        self.debug_image_pub = self.create_publisher(
            CompressedImage, '/image_raw/compressed', 10)
        # The locked target's image-space box, consumed by the fusion node.
        self.locked_bbox_pub = self.create_publisher(
            Detection2D, '/perception/locked_bbox', 10)

        # YOLO model
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model loaded: {model_path}')

        # Tracking state
        self.locked_id = None
        self.display_id = None
        self.lock_miss_count = 0
        self.lost_count = 0
        self.last_known_box = None
        self.vx = 0.0
        self.vy = 0.0
        self.lost_threshold = self.fps * self.wait_seconds

        # Click-to-lock: detections shared with the mouse callback
        self._det_lock = threading.Lock()
        self._current_ids = []
        self._current_boxes = []
        self.frame_count = 0

        if self.show_window:
            cv2.namedWindow(self.WINDOW_NAME)
            cv2.setMouseCallback(self.WINDOW_NAME, self._mouse_callback)

        self.timer = self.create_timer(1.0 / max(self.fps, 1), self._timer_callback)
        self.get_logger().info(
            'YOLO tracker started. Click a box to lock, click empty area (or press r) to release.')

    # ----- click-to-lock -----------------------------------------------
    def _mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self._det_lock:
            ids = list(self._current_ids)
            boxes = list(self._current_boxes)
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box
            if x1 <= x <= x2 and y1 <= y <= y2:
                self._set_lock(int(ids[i]), source='click')
                return
        self._release_lock()

    def _set_lock(self, target_id, source='topic'):
        self.locked_id = target_id
        self.display_id = target_id
        self.lock_miss_count = 0
        self.lost_count = 0
        self.last_known_box = None
        self.vx = 0.0
        self.vy = 0.0
        self.get_logger().info(f'[{source}] Locked on ID {target_id}')

    def _release_lock(self):
        self.locked_id = None
        self.display_id = None
        self.last_known_box = None
        self.vx = 0.0
        self.vy = 0.0
        self.get_logger().info('Lock released')

    def _target_id_callback(self, msg):
        self._set_lock(msg.data, source='topic')

    def _publish_locked_bbox(self, box):
        x1, y1, x2, y2 = box
        d = Detection2D()
        d.header.stamp = self.get_clock().now().to_msg()
        d.header.frame_id = self.camera_frame
        d.bbox = BoundingBox2D()
        d.bbox.center.position.x = float((x1 + x2) / 2.0)
        d.bbox.center.position.y = float((y1 + y2) / 2.0)
        d.bbox.size_x = float(max(x2 - x1, 1.0))
        d.bbox.size_y = float(max(y2 - y1, 1.0))
        if self.display_id is not None:
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(self.display_id)
            hyp.hypothesis.score = 1.0
            d.results.append(hyp)
        self.locked_bbox_pub.publish(d)

    # ----- main loop ----------------------------------------------------
    def _timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.frame_count += 1

        try:
            h, w = frame.shape[:2]

            results = self.model.track(
                source=frame, classes=[0], verbose=False, persist=True,
                tracker='botsort.yaml', conf=self.conf, iou=self.iou, imgsz=640)
            frame = results[0].plot()
            boxes_raw = results[0].boxes

            current_ids = []
            current_boxes = []
            if boxes_raw.id is not None:
                current_ids = boxes_raw.id.cpu().numpy().astype(int).tolist()
                current_boxes = boxes_raw.xyxy.cpu().numpy().tolist()

            with self._det_lock:
                self._current_ids = current_ids
                self._current_boxes = current_boxes

            chosen_box = None

            if self.locked_id is not None:
                if self.locked_id in current_ids:
                    idx = current_ids.index(self.locked_id)
                    chosen_box = current_boxes[idx]
                    if self.last_known_box is not None:
                        prev_cx = (self.last_known_box[0] + self.last_known_box[2]) / 2
                        prev_cy = (self.last_known_box[1] + self.last_known_box[3]) / 2
                        curr_cx = (chosen_box[0] + chosen_box[2]) / 2
                        curr_cy = (chosen_box[1] + chosen_box[3]) / 2
                        self.vx = curr_cx - prev_cx
                        self.vy = curr_cy - prev_cy
                    else:
                        self.vx = 0.0
                        self.vy = 0.0
                    self.lock_miss_count = 0
                    self.lost_count = 0
                    self.last_known_box = chosen_box
                else:
                    # Target missed this frame — predict and try to re-lock.
                    self.lock_miss_count += 1
                    if self.last_known_box is not None:
                        lx1, ly1, lx2, ly2 = self.last_known_box
                        self.vx *= 0.95
                        self.vy *= 0.95
                        lx1 += self.vx; lx2 += self.vx
                        ly1 += self.vy; ly2 += self.vy
                        self.last_known_box = [lx1, ly1, lx2, ly2]

                        box_w = lx2 - lx1
                        box_h = ly2 - ly1
                        margin_x = box_w * 4.0
                        margin_y = box_h * 4.0
                        sx1 = max(0, lx1 - margin_x)
                        sy1 = max(0, ly1 - margin_y)
                        sx2 = min(w, lx2 + margin_x)
                        sy2 = min(h, ly2 + margin_y)

                        cv2.rectangle(frame, (int(sx1), int(sy1)), (int(sx2), int(sy2)), (0, 255, 255), 3)
                        cv2.putText(frame, 'PREDICTING PATH...', (int(sx1), int(sy1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                        if current_boxes:
                            best_score = 0
                            best_id = None
                            best_box = None
                            last_area = box_w * box_h
                            for i, box in enumerate(current_boxes):
                                bx1, by1, bx2, by2 = box
                                bcx = (bx1 + bx2) / 2
                                bcy = (by1 + by2) / 2
                                if sx1 < bcx < sx2 and sy1 < bcy < sy2:
                                    curr_area = (bx2 - bx1) * (by2 - by1)
                                    if last_area > 0 and curr_area > 0:
                                        size_sim = min(last_area, curr_area) / max(last_area, curr_area)
                                    else:
                                        size_sim = 0
                                    if size_sim > best_score:
                                        best_score = size_sim
                                        best_id = current_ids[i]
                                        best_box = box
                            if best_score > 0.5:
                                self.get_logger().warn(f'Re-locked: {self.locked_id} -> {best_id}')
                                self.locked_id = best_id
                                chosen_box = best_box
                                self.lock_miss_count = 0
                                self.lost_count = 0
                                self.last_known_box = chosen_box

            # Publish the locked box (the fusion node turns it into a 3D target)
            if chosen_box is not None:
                self.lost_count = 0
                self._publish_locked_bbox(chosen_box)
                x1, y1, _, _ = chosen_box
                label = f'LOCK ID:{self.display_id}' if self.display_id is not None else 'LOCK'
                cv2.putText(frame, label, (int(x1), int(y1) - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            elif self.locked_id is not None:
                self.lost_count += 1
                if self.lost_count > self.lost_threshold:
                    self.last_known_box = None
                else:
                    remain = self.wait_seconds - (self.lost_count / self.fps)
                    cv2.putText(frame, f'SEARCHING... {remain:.1f}s', (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            else:
                cv2.putText(frame, 'Click a box to lock', (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

            if self.show_window:
                cv2.imshow(self.WINDOW_NAME, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("'q' pressed — shutting down.")
                    rclpy.shutdown()
                elif key == ord('r'):
                    self._release_lock()

            # Debug image (every other frame to save bandwidth)
            if self.frame_count % 2 == 0:
                _, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.camera_frame
                msg.format = 'jpeg'
                msg.data = enc.tobytes()
                self.debug_image_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackSelectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
