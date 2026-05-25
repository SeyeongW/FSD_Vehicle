"""SIYI gimbal controller node.

Subscribes to /target/position (PointStamped in LiDAR frame) and drives a
SIYI gimbal toward the bearing of that target via a simple proportional
speed loop.

Wraps the SIYIGimbal protocol class so the serial code is isolated from ROS.
"""
from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

try:
    import serial  # noqa: F401
    import struct
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SIYIGimbal:
    """SIYI A8/ZR10-style serial protocol. Speed/center commands only."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud: int = 115200, logger=None):
        self.logger = logger
        self.seq = 0
        self.lock = threading.Lock()
        if not SERIAL_AVAILABLE:
            self.ser = None
            self._log_warn('pyserial not available; gimbal output disabled')
            return
        try:
            import serial as _serial
            self.ser = _serial.Serial(port, baud, timeout=0.1)
            self._log_info(f'Gimbal connected on {port}')
        except Exception as e:
            self.ser = None
            self._log_warn(f'Gimbal not connected ({e}); commands will be no-ops')

    def _log_info(self, msg):
        (self.logger.info if self.logger else print)(msg)

    def _log_warn(self, msg):
        (self.logger.warn if self.logger else print)(msg)

    def send_speed(self, yaw_speed: int, pitch_speed: int):
        if self.ser is None:
            return
        cmd_id = 0x07
        yaw_speed = max(-100, min(100, int(yaw_speed)))
        pitch_speed = max(-100, min(100, int(pitch_speed)))
        payload = struct.pack('<b b', yaw_speed, pitch_speed)
        packet = self._build_packet(cmd_id, payload)
        with self.lock:
            self.ser.write(packet)
        self.seq += 1

    def center(self):
        if self.ser is None:
            return
        cmd_id = 0x08
        payload = struct.pack('<B', 1)
        packet = self._build_packet(cmd_id, payload)
        with self.lock:
            self.ser.write(packet)
        self.seq += 1

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

    def _build_packet(self, cmd_id, payload):
        packet = bytearray()
        packet.append(0x55)
        packet.append(0x66)
        packet.append(0x01)
        packet.extend(struct.pack('<H', len(payload)))
        packet.extend(struct.pack('<H', self.seq))
        packet.append(cmd_id)
        packet.extend(payload)
        crc = self._calc_crc16(packet)
        packet.extend(struct.pack('<H', crc))
        return packet

    @staticmethod
    def _calc_crc16(data):
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return crc


class GimbalControllerNode(Node):
    def __init__(self):
        super().__init__('gimbal_controller_node')

        self.declare_parameter('target_topic', '/target/position')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        # Mount: assume LiDAR x-forward, y-left, z-up. Yaw error = atan2(y, x),
        # pitch error = atan2(z - mount_height, range_xy).
        self.declare_parameter('mount_height', 0.0)
        # Proportional gains (deg -> command units in [-100, 100]).
        self.declare_parameter('kp_yaw', 2.5)
        self.declare_parameter('kp_pitch', 2.5)
        self.declare_parameter('deadband_deg', 1.5)
        self.declare_parameter('max_speed', 60)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('target_timeout_sec', 0.5)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud').value)
        self.mount_height = float(self.get_parameter('mount_height').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.deadband = float(self.get_parameter('deadband_deg').value)
        self.max_speed = int(self.get_parameter('max_speed').value)
        self.timeout = float(self.get_parameter('target_timeout_sec').value)

        self.gimbal = SIYIGimbal(port=port, baud=baud, logger=self.get_logger())

        self.target: PointStamped | None = None
        self.create_subscription(PointStamped,
                                 self.get_parameter('target_topic').value,
                                 self.target_cb, 5)

        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f'Gimbal controller running (port={port}, kp_yaw={self.kp_yaw}, '
            f'kp_pitch={self.kp_pitch}, deadband={self.deadband} deg)')

    def target_cb(self, msg: PointStamped):
        self.target = msg

    def tick(self):
        if self.target is None:
            self.gimbal.send_speed(0, 0)
            return

        t_msg = self.target.header.stamp
        age = (self.get_clock().now().nanoseconds * 1e-9) - (t_msg.sec + t_msg.nanosec * 1e-9)
        if age > self.timeout:
            self.gimbal.send_speed(0, 0)
            return

        x = self.target.point.x
        y = self.target.point.y
        z = self.target.point.z - self.mount_height
        range_xy = math.hypot(x, y)
        if range_xy < 1e-3:
            self.gimbal.send_speed(0, 0)
            return

        yaw_err_deg = math.degrees(math.atan2(y, x))
        pitch_err_deg = math.degrees(math.atan2(z, range_xy))

        yaw_cmd = 0.0 if abs(yaw_err_deg) < self.deadband else self.kp_yaw * yaw_err_deg
        pitch_cmd = 0.0 if abs(pitch_err_deg) < self.deadband else self.kp_pitch * pitch_err_deg

        yaw_cmd = max(-self.max_speed, min(self.max_speed, yaw_cmd))
        pitch_cmd = max(-self.max_speed, min(self.max_speed, pitch_cmd))

        self.gimbal.send_speed(int(yaw_cmd), int(pitch_cmd))

    def destroy_node(self):
        try:
            self.gimbal.send_speed(0, 0)
            self.gimbal.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
