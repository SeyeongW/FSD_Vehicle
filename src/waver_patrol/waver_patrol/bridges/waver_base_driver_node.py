from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from tf2_ros import TransformBroadcaster

from waver_patrol.bridges.cmd_vel_to_json import CmdVelToJson, CmdVelToJsonConfig
from waver_patrol.safety.acceleration_limiter import AccelerationLimiter, AccelerationLimiterConfig
from waver_patrol.safety.command import TwistCommand, WheelCommand
from waver_patrol.safety.command_sanitizer import CommandSanitizer, SanitizerConfig


class WaverBaseDriverNode(Node):
    """Single-owner real base serial driver.

    This node owns one serial port, reads base feedback, and writes only the final
    safety-filtered /cmd_vel. It is intended to replace split feedback/command
    serial ownership during real wheel-off and low-speed wheel-on tests.
    """

    def __init__(self) -> None:
        super().__init__("waver_base_driver_node")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("emergency_stop_topic", "/waver/emergency_stop")
        self.declare_parameter("external_stop_topic", "/waver/external_stop")
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_protocol", "lr")
        self.declare_parameter("stop_repeat", 5)
        self.declare_parameter("serial_reconnect_interval_s", 1.0)
        self.declare_parameter("linear_gain", 2.5)
        self.declare_parameter("angular_gain", 0.55)
        self.declare_parameter("max_left_right", 0.32)
        self.declare_parameter("max_demo_speed", 0.32)
        self.declare_parameter("deadband", 0.015)
        self.declare_parameter("min_linear_ratio", 0.25)
        self.declare_parameter("wheel_delta_per_tick", 0.05)
        self.declare_parameter("require_neutral_before_reverse", True)
        self.declare_parameter("pure_turn_mode", "pivot")
        self.declare_parameter("pure_turn_min_ratio", 0.16)
        self.declare_parameter("pure_turn_max_ratio", 0.16)
        self.declare_parameter("mixed_turn_mode", "inside_brake")
        self.declare_parameter("mixed_turn_inner_ratio", 0.0)
        self.declare_parameter("mixed_turn_outer_ratio", 0.22)
        self.declare_parameter("min_motor_voltage_v", 7.0)
        self.declare_parameter("feedback_request_enabled", True)
        self.declare_parameter("feedback_request_interval_s", 0.5)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_base_m", 0.28)
        self.declare_parameter("odom_distance_scale", 1.0)
        self.declare_parameter("max_encoder_delta_m", 0.5)

        self.serial_port = str(self.get_parameter("serial_port").value).strip()
        if not self.serial_port:
            raise RuntimeError("waver_base_driver_node requires serial_port=/dev/serial/by-id/<WAVER_SERIAL_ID>")
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.command_protocol = str(self.get_parameter("command_protocol").value).strip().lower()
        self.stop_repeat = int(self.get_parameter("stop_repeat").value)
        self.reconnect_interval_s = float(self.get_parameter("serial_reconnect_interval_s").value)
        self.min_motor_voltage_v = float(self.get_parameter("min_motor_voltage_v").value)
        self.feedback_request_enabled = bool(self.get_parameter("feedback_request_enabled").value)
        self.feedback_request_interval_s = float(self.get_parameter("feedback_request_interval_s").value)
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.wheel_base_m = max(0.05, float(self.get_parameter("wheel_base_m").value))
        self.odom_distance_scale = float(self.get_parameter("odom_distance_scale").value)
        self.max_encoder_delta_m = float(self.get_parameter("max_encoder_delta_m").value)

        self.converter = CmdVelToJson(
            CmdVelToJsonConfig(
                linear_gain=float(self.get_parameter("linear_gain").value),
                angular_gain=float(self.get_parameter("angular_gain").value),
                max_left_right=float(self.get_parameter("max_left_right").value),
                deadband=float(self.get_parameter("deadband").value),
                min_linear_ratio=float(self.get_parameter("min_linear_ratio").value),
                pure_turn_mode=str(self.get_parameter("pure_turn_mode").value),
                pure_turn_min_ratio=float(self.get_parameter("pure_turn_min_ratio").value),
                pure_turn_max_ratio=float(self.get_parameter("pure_turn_max_ratio").value),
                mixed_turn_mode=str(self.get_parameter("mixed_turn_mode").value),
                mixed_turn_inner_ratio=float(self.get_parameter("mixed_turn_inner_ratio").value),
                mixed_turn_outer_ratio=float(self.get_parameter("mixed_turn_outer_ratio").value),
            ),
            sanitizer=CommandSanitizer(
                SanitizerConfig(
                    max_left_right=float(self.get_parameter("max_left_right").value),
                    max_demo_speed=float(self.get_parameter("max_demo_speed").value),
                )
            ),
            acceleration_limiter=AccelerationLimiter(
                AccelerationLimiterConfig(
                    max_delta_per_tick=float(self.get_parameter("wheel_delta_per_tick").value),
                    require_neutral_before_reverse=bool(
                        self.get_parameter("require_neutral_before_reverse").value
                    ),
                )
            ),
        )
        self.serial = None
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.estop = False
        self.external_stop = False
        self.latest = WheelCommand.stop(reason="driver startup")
        self.latest_twist = TwistCommand(0.0, 0.0, source="driver_startup")
        self.last_payload = {"T": 1, "L": 0.0, "R": 0.0}
        self.last_cmd_time = 0.0
        self.last_feedback_time = 0.0
        self.last_feedback_request_time = 0.0
        self.last_voltage_v = 0.0
        self.state = "STARTUP_STOP"
        self.odom_initialized = False
        self.wheel_odom_feedback_seen = False
        self.prev_left_m = 0.0
        self.prev_right_m = 0.0
        self.prev_odom_time = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.odom_raw_pub = self.create_publisher(Float32MultiArray, "odom/odom_raw", 50)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 50)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 50)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 20)
        self.voltage_pub = self.create_publisher(Float32, "voltage", 20)
        self.base_state_pub = self.create_publisher(String, "/waver/base_driver_state", 10)
        self.serial_owner_pub = self.create_publisher(String, "/waver/serial_owner_state", 10)

        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd_vel, 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), self.on_estop, 10)
        self.create_subscription(Bool, str(self.get_parameter("external_stop_topic").value), self.on_external_stop, 10)

        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()
        rate = float(self.get_parameter("command_rate_hz").value)
        self.command_timer = self.create_timer(1.0 / max(rate, 1.0), self.command_tick)
        self.state_timer = self.create_timer(0.2, self.publish_state)
        self.get_logger().warn(
            f"waver_base_driver_node owns serial port {self.serial_port}. "
            "Do not run ugv_bringup feedback, ugv_driver, or serial_cmd_vel_bridge on this port."
        )

    def open_serial(self) -> bool:
        try:
            import serial
        except ImportError as exc:
            self.state = f"SERIAL_IMPORT_FAILED {exc}"
            return False
        with self.serial_lock:
            if self.serial is not None and getattr(self.serial, "is_open", False):
                return True
            try:
                self.serial = serial.Serial(
                    self.serial_port,
                    self.baudrate,
                    timeout=0.05,
                    write_timeout=0.05,
                    dsrdtr=None,
                )
                # Waveshare's serial examples explicitly deassert RTS/DTR. On
                # ESP32-based slave boards these lines can otherwise reset or
                # hold the controller in a non-driving state when the port opens.
                self.serial.setRTS(False)
                self.serial.setDTR(False)
                self.state = "CONNECTED_STARTUP_STOP"
                self.send_stop_locked(repeat=self.stop_repeat)
                return True
            except Exception as exc:
                self.serial = None
                self.state = f"SERIAL_CONNECT_FAILED {exc}"
                return False

    def reader_loop(self) -> None:
        while not self.stop_event.is_set():
            if not self.open_serial():
                time.sleep(self.reconnect_interval_s)
                continue
            try:
                with self.serial_lock:
                    line = self.serial.readline() if self.serial is not None else b""
                if not line:
                    continue
                data = json.loads(line.decode("utf-8", errors="replace"))
                self.publish_feedback(data)
            except Exception as exc:
                self.state = f"FEEDBACK_READ_FAILED {exc}"
                time.sleep(0.05)

    def publish_feedback(self, data: dict[str, Any]) -> None:
        if int(data.get("T", 1001)) != 1001:
            return
        self.last_feedback_time = time.monotonic()

        if "odl" in data and "odr" in data:
            self.wheel_odom_feedback_seen = True
            odom_left = float(data.get("odl", 0.0)) / 100.0
            odom_right = float(data.get("odr", 0.0)) / 100.0
            self.odom_raw_pub.publish(Float32MultiArray(data=[odom_left, odom_right]))
            self.publish_wheel_odom(odom_left, odom_right)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "base_imu_link"
        imu.linear_acceleration.x = 9.8 * float(data.get("ax", 0.0)) / 8192.0
        imu.linear_acceleration.y = 9.8 * float(data.get("ay", 0.0)) / 8192.0
        imu.linear_acceleration.z = 9.8 * float(data.get("az", 0.0)) / 8192.0
        imu.angular_velocity.x = math.pi * float(data.get("gx", 0.0)) / (16.4 * 180.0)
        imu.angular_velocity.y = math.pi * float(data.get("gy", 0.0)) / (16.4 * 180.0)
        imu.angular_velocity.z = math.pi * float(data.get("gz", 0.0)) / (16.4 * 180.0)
        self.imu_pub.publish(imu)

        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x = float(data.get("mx", 0.0)) * 0.15
        mag.magnetic_field.y = float(data.get("my", 0.0)) * 0.15
        mag.magnetic_field.z = float(data.get("mz", 0.0)) * 0.15
        self.mag_pub.publish(mag)

        self.last_voltage_v = float(data.get("v", 0.0))
        self.voltage_pub.publish(Float32(data=self.last_voltage_v))

    def on_cmd_vel(self, msg: Twist) -> None:
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.latest = WheelCommand.stop(reason="invalid cmd_vel")
            self.latest_twist = TwistCommand(0.0, 0.0, source="invalid cmd_vel")
            self.last_cmd_time = time.monotonic()
            self.state = "INVALID_CMD_STOP"
            return
        if self.estop or self.external_stop:
            self.latest = WheelCommand.stop(reason="estop/external stop")
            self.latest_twist = TwistCommand(0.0, 0.0, source="estop/external stop")
            self.last_cmd_time = time.monotonic()
            return
        self.latest_twist = TwistCommand(float(msg.linear.x), float(msg.angular.z), source="cmd_vel")
        self.latest = self.converter.convert(self.latest_twist, source="cmd_vel")
        self.last_cmd_time = time.monotonic()
        self.state = "CMD_FRESH"

    def on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)
        if self.estop:
            self.latest = WheelCommand.stop(reason="emergency stop")
            self.latest_twist = TwistCommand(0.0, 0.0, source="emergency stop")
            self.state = "EMERGENCY_STOP"

    def on_external_stop(self, msg: Bool) -> None:
        self.external_stop = bool(msg.data)
        if self.external_stop:
            self.latest = WheelCommand.stop(reason="external stop")
            self.latest_twist = TwistCommand(0.0, 0.0, source="external stop")
            self.state = "EXTERNAL_STOP"

    def command_tick(self) -> None:
        if not self.open_serial():
            return
        cmd = self.safe_command()
        try:
            with self.serial_lock:
                if self.serial is not None:
                    payload = self.command_payload(cmd)
                    self.last_payload = payload
                    self.serial.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
                    if self.should_request_feedback():
                        self.serial.write(b'{"T":130}\n')
                        self.last_feedback_request_time = time.monotonic()
        except Exception as exc:
            self.state = f"SERIAL_WRITE_FAILED {exc}"
            with self.serial_lock:
                try:
                    if self.serial is not None:
                        self.serial.close()
                except Exception:
                    pass
                self.serial = None

    def safe_command(self) -> WheelCommand:
        if self.estop:
            self.state = "EMERGENCY_STOP"
            self.latest_twist = TwistCommand(0.0, 0.0, source="emergency stop")
            self.latest = WheelCommand.stop(reason="emergency stop")
            return self.latest
        if self.external_stop:
            self.state = "EXTERNAL_STOP"
            self.latest_twist = TwistCommand(0.0, 0.0, source="external stop")
            self.latest = WheelCommand.stop(reason="external stop")
            return self.latest
        if self.last_cmd_time == 0.0 or time.monotonic() - self.last_cmd_time > self.cmd_timeout_s:
            self.state = "CMD_TIMEOUT_STOP"
            self.latest_twist = TwistCommand(0.0, 0.0, source="cmd_vel timeout")
            self.latest = WheelCommand.stop(reason="cmd_vel timeout")
            return self.latest
        return self.latest

    def command_payload(self, cmd: WheelCommand) -> dict[str, Any]:
        if self.command_protocol in {"t13", "twist", "x_z", "x-z", "ugv_driver"}:
            if cmd.is_stop:
                return {"T": 13, "X": 0.0, "Z": 0.0}
            return {
                "T": 13,
                "X": round(float(self.latest_twist.linear_x), 4),
                "Z": round(float(self.latest_twist.angular_z), 4),
            }
        return cmd.as_rover_json()

    def should_request_feedback(self) -> bool:
        if not self.feedback_request_enabled:
            return False
        now = time.monotonic()
        return now - self.last_feedback_request_time >= max(self.feedback_request_interval_s, 0.1)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def publish_wheel_odom(self, left_m_raw: float, right_m_raw: float) -> None:
        if not self.publish_odom_enabled:
            return
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        left_m = left_m_raw * self.odom_distance_scale
        right_m = right_m_raw * self.odom_distance_scale
        if not self.odom_initialized:
            self.prev_left_m = left_m
            self.prev_right_m = right_m
            self.prev_odom_time = now
            self.odom_initialized = True
            self.publish_current_odom(stamp, 0.0, 0.0)
            return

        dt = now - self.prev_odom_time
        dl = left_m - self.prev_left_m
        dr = right_m - self.prev_right_m
        self.prev_left_m = left_m
        self.prev_right_m = right_m
        self.prev_odom_time = now

        if dt <= 1e-4 or dt > 1.0:
            self.publish_current_odom(stamp, 0.0, 0.0)
            return
        if (
            not math.isfinite(dl)
            or not math.isfinite(dr)
            or abs(dl) > self.max_encoder_delta_m
            or abs(dr) > self.max_encoder_delta_m
        ):
            self.state = "ODOM_DELTA_REJECTED"
            self.publish_current_odom(stamp, 0.0, 0.0)
            return

        distance = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.wheel_base_m
        mid_yaw = self.odom_yaw + 0.5 * dtheta
        self.odom_x += distance * math.cos(mid_yaw)
        self.odom_y += distance * math.sin(mid_yaw)
        self.odom_yaw = math.atan2(math.sin(self.odom_yaw + dtheta), math.cos(self.odom_yaw + dtheta))
        linear_v = distance / dt
        angular_v = dtheta / dt
        self.publish_current_odom(stamp, linear_v, angular_v)

    def publish_current_odom(self, stamp, linear_v: float, angular_v: float) -> None:
        qx, qy, qz, qw = self.yaw_to_quaternion(self.odom_yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_v
        odom.twist.twist.angular.z = angular_v
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom_x
            tf.transform.translation.y = self.odom_y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def send_stop_locked(self, repeat: int = 1) -> None:
        if self.serial is None:
            return
        stop = WheelCommand.stop(reason="serial stop").as_rover_json()
        if self.command_protocol in {"t13", "twist", "x_z", "x-z", "ugv_driver"}:
            stop = {"T": 13, "X": 0.0, "Z": 0.0}
        for _ in range(max(1, repeat)):
            self.serial.write((json.dumps(stop, separators=(",", ":")) + "\n").encode("utf-8"))

    def publish_state(self) -> None:
        age = time.monotonic() - self.last_feedback_time if self.last_feedback_time else -1.0
        connected = self.serial is not None and getattr(self.serial, "is_open", False)
        if self.last_feedback_time == 0.0:
            motor_power = "UNKNOWN"
        elif self.last_voltage_v < self.min_motor_voltage_v:
            motor_power = "MOTOR_POWER_LOW"
        else:
            motor_power = "OK"
        text = (
            f"{self.state} port={self.serial_port} connected={connected} "
            f"feedback_age_sec={age:.2f} protocol={self.command_protocol} "
            f"voltage_v={self.last_voltage_v:.3f} motor_power={motor_power} "
            f"odom_ok={self.wheel_odom_feedback_seen and self.odom_initialized} "
            f"odom=({self.odom_x:.3f},{self.odom_y:.3f},{self.odom_yaw:.3f}) "
            f"left={self.latest.left:.3f} right={self.latest.right:.3f} "
            f"x={self.latest_twist.linear_x:.3f} z={self.latest_twist.angular_z:.3f} "
            f"payload={self.last_payload}"
        )
        self.base_state_pub.publish(String(data=text))
        self.serial_owner_pub.publish(String(data=f"waver_base_driver_node port={self.serial_port}"))

    def destroy_node(self) -> bool:
        self.stop_event.set()
        with self.serial_lock:
            if self.serial is not None:
                try:
                    self.send_stop_locked(repeat=self.stop_repeat)
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WaverBaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
