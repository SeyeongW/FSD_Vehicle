from __future__ import annotations

import json
import os
import time
from pathlib import Path
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class SoundBackend:
    """Mockable sound hardware backend interface.

    These implementations intentionally avoid direct subprocess/GPIO/serial
    side effects in normal unit-test and dry-run paths. Real hardware output is
    selected only after the node-level ACK gates pass.
    """

    name = "base"

    def validate(self, node: Node) -> str:
        return ""

    def start(self, node: Node, event_id: str) -> str:
        return f"{self.name}:started event_id={event_id}"

    def cancel(self, node: Node, event_id: str, reason: str) -> str:
        return f"{self.name}:cancelled event_id={event_id} reason={reason}"


class DisabledSoundBackend(SoundBackend):
    name = "disabled"

    def validate(self, node: Node) -> str:
        return "HARDWARE_BACKEND_DISABLED"

    def start(self, node: Node, event_id: str) -> str:
        return f"{self.name}:no_output event_id={event_id}"


class MockSoundBackend(SoundBackend):
    name = "mock"


class AlsaSoundBackend(SoundBackend):
    name = "alsa"

    def validate(self, node: Node) -> str:
        if not str(node.get_parameter("sound_file_path").value).strip():
            return "ALSA_SOUND_FILE_REQUIRED"
        return ""


class GpioSoundBackend(SoundBackend):
    name = "gpio"

    def validate(self, node: Node) -> str:
        if int(node.get_parameter("gpio_pin").value) < 0:
            return "GPIO_PIN_REQUIRED"
        return ""


class SerialSoundBackend(SoundBackend):
    name = "serial"

    def validate(self, node: Node) -> str:
        if not str(node.get_parameter("serial_port").value).strip():
            return "SERIAL_PORT_REQUIRED"
        return ""


class SoundDeterrentNode(Node):
    """Safety-gated deterrence task runner.

    By default this node performs a simulated sound task only. It never emits
    real audio/hardware output unless `enable_sound_output` is explicitly true
    and a separate, safety-reviewed backend is connected.
    """

    def __init__(self) -> None:
        super().__init__("sound_deterrent_node")
        self.declare_parameter("enable_sound_output", False)
        self.declare_parameter("deterrence_classes", ["bird"])
        self.declare_parameter("min_confidence_for_sound", 0.65)
        self.declare_parameter("sound_type", "SIMULATED_DETERRENT")
        self.declare_parameter("sound_task_duration_sec", 5.0)
        self.declare_parameter("alert_cooldown_sec", 10.0)
        self.declare_parameter("max_sound_retries_per_target", 1)
        self.declare_parameter("hardware_backend", "disabled")
        self.declare_parameter("sound_file_path", "")
        self.declare_parameter("gpio_pin", -1)
        self.declare_parameter("alsa_device", "default")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("max_volume_percent", 50)
        self.declare_parameter("legal_safety_ack_required", True)
        self.declare_parameter("safety_ack", False)
        self.declare_parameter("event_log_dir", "reports/sound_events")
        self.backend = self.make_backend()

        self.request_active = False
        self.task_active = False
        self.task_started_time = 0.0
        self.last_completed_time = -1e9
        self.target_class = "none"
        self.target_confidence = 0.0
        self.bird_confirmed = False
        self.fusion_valid = False
        self.dynamic_valid = False
        self.camera_centered = False
        self.battery_critical = False
        self.emergency_stop = False
        self.external_stop = False
        self.event_id = ""
        self.retry_count = 0

        self.state_pub = self.create_publisher(String, "/waver/sound_alert_state", 10)
        self.done_pub = self.create_publisher(Bool, "/waver/sound_task_done", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/sound_task_active", 10)
        self.event_id_pub = self.create_publisher(String, "/waver/sound_event_id", 10)

        self.create_subscription(Bool, "/waver/sound_alert_request", self.request_callback, 10)
        self.create_subscription(String, "/waver/target_class", lambda m: setattr(self, "target_class", m.data.strip().lower()), 10)
        self.create_subscription(Float32, "/waver/target_confidence", lambda m: setattr(self, "target_confidence", float(m.data)), 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/bird_target_valid", lambda m: setattr(self, "fusion_valid", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/moving_target_valid", lambda m: setattr(self, "dynamic_valid", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/camera_target_centered", lambda m: setattr(self, "camera_centered", bool(m.data)), 10)
        self.create_subscription(
            String,
            "/waver/battery_safety_state",
            lambda m: setattr(self, "battery_critical", "CRITICAL" in m.data.upper()),
            10,
        )
        self.create_subscription(Bool, "/waver/emergency_stop", lambda m: setattr(self, "emergency_stop", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/external_stop", lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_timer(0.1, self.tick)

    def request_callback(self, msg: Bool) -> None:
        self.request_active = bool(msg.data)
        if not self.request_active and not self.task_active:
            self.done_pub.publish(Bool(data=False))
            self.active_pub.publish(Bool(data=False))

    def tick(self) -> None:
        now = self._now()
        if self.task_active:
            if self.emergency_stop or self.external_stop:
                self.task_active = False
                self.request_active = False
                self.backend.cancel(self, self.event_id, "SAFETY_STOP")
                self.active_pub.publish(Bool(data=False))
                self.done_pub.publish(Bool(data=False))
                self.publish_state(f"SOUND_TASK_CANCELLED event_id={self.event_id} reason=SAFETY_STOP")
                self.write_event("cancelled", "SAFETY_STOP")
                return
            duration = float(self.get_parameter("sound_task_duration_sec").value)
            self.active_pub.publish(Bool(data=True))
            if now - self.task_started_time >= duration:
                self.task_active = False
                self.request_active = False
                self.last_completed_time = now
                self.active_pub.publish(Bool(data=False))
                self.done_pub.publish(Bool(data=True))
                self.publish_state(
                    "SOUND_TASK_DONE "
                    f"event_id={self.event_id} duration_sec={duration:.2f} "
                    f"enable_sound_output={bool(self.get_parameter('enable_sound_output').value)}"
                )
                self.write_event("done", "")
            else:
                self.publish_state(
                    "SOUND_TASK_RUNNING "
                    f"event_id={self.event_id} elapsed_sec={now - self.task_started_time:.2f}"
                )
            return

        if not self.request_active:
            return
        blocked = self.block_reason(now)
        if blocked:
            self.done_pub.publish(Bool(data=False))
            self.active_pub.publish(Bool(data=False))
            self.publish_state(f"REQUEST_BLOCKED_{blocked}")
            self.request_active = False
            return

        self.task_active = True
        self.task_started_time = now
        self.event_id = f"sound_{uuid4().hex[:10]}"
        self.retry_count += 1
        backend_result = self.backend.start(self, self.event_id)
        self.event_id_pub.publish(String(data=self.event_id))
        self.done_pub.publish(Bool(data=False))
        self.active_pub.publish(Bool(data=True))
        self.publish_state(
            "SOUND_TASK_RUNNING "
            f"event_id={self.event_id} target_class={self.target_class} "
            f"confidence={self.target_confidence:.2f} sound_type={self.get_parameter('sound_type').value} "
            f"enable_sound_output={bool(self.get_parameter('enable_sound_output').value)} "
            f"backend={self.get_parameter('hardware_backend').value} backend_result={backend_result}"
        )

    def block_reason(self, now: float) -> str:
        if self.emergency_stop or self.external_stop:
            return "SAFETY_STOP"
        classes = {str(v).strip().lower() for v in self.get_parameter("deterrence_classes").value}
        if self.target_class not in classes:
            return f"BY_CLASS class={self.target_class} deterrence_classes={sorted(classes)}"
        if self.target_class == "bird" and not self.bird_confirmed:
            return "NOT_BIRD bird_confirmed=false"
        if self.target_class == "bird" and not self.fusion_valid:
            return "FUSION_NOT_VALID bird_target_valid=false"
        if self.target_class == "bird" and not self.dynamic_valid:
            return "DYNAMIC_NOT_VALID moving_target_valid=false"
        if self.target_class == "bird" and not self.camera_centered:
            return "CAMERA_NOT_CENTERED"
        if self.battery_critical:
            return "BATTERY_CRITICAL"
        min_conf = float(self.get_parameter("min_confidence_for_sound").value)
        if self.target_confidence < min_conf:
            return f"LOW_CONFIDENCE confidence={self.target_confidence:.2f} threshold={min_conf:.2f}"
        cooldown = float(self.get_parameter("alert_cooldown_sec").value)
        if now - self.last_completed_time < cooldown:
            return f"COOLDOWN remaining_sec={cooldown - (now - self.last_completed_time):.2f}"
        if self.retry_count >= max(1, int(self.get_parameter("max_sound_retries_per_target").value)):
            return "RETRY_LIMIT"
        duration = float(self.get_parameter("sound_task_duration_sec").value)
        if duration <= 0.0 or duration > 3.0:
            return f"DURATION_LIMIT duration_sec={duration:.2f}"
        volume = int(self.get_parameter("max_volume_percent").value)
        if volume < 0 or volume > 50:
            return f"VOLUME_LIMIT max_volume_percent={volume}"
        if bool(self.get_parameter("enable_sound_output").value):
            backend = str(self.get_parameter("hardware_backend").value).strip().lower()
            if bool(self.get_parameter("legal_safety_ack_required").value) and not bool(self.get_parameter("safety_ack").value):
                return "SAFETY_ACK_REQUIRED"
            for env_name in ("WAVER_ACK_SOUND_HARDWARE", "WAVER_ACK_LOCAL_SOUND_LAW", "WAVER_ACK_OPERATOR_SUPERVISION"):
                if os.environ.get(env_name) != "1":
                    return f"ACK_ENV_REQUIRED env={env_name}"
            if backend in {"disabled", "mock", "topic_only", ""}:
                return "HARDWARE_BACKEND_DISABLED"
            if backend not in {"alsa", "gpio", "serial"}:
                return f"HARDWARE_BACKEND_UNSUPPORTED backend={backend}"
            backend_reason = self.backend.validate(self)
            if backend_reason:
                return backend_reason
        return ""

    def make_backend(self) -> SoundBackend:
        if not bool(self.get_parameter("enable_sound_output").value):
            return DisabledSoundBackend()
        backend = str(self.get_parameter("hardware_backend").value).strip().lower()
        if backend == "mock":
            return MockSoundBackend()
        if backend == "alsa":
            return AlsaSoundBackend()
        if backend == "gpio":
            return GpioSoundBackend()
        if backend == "serial":
            return SerialSoundBackend()
        return DisabledSoundBackend()

    def publish_state(self, state: str) -> None:
        self.state_pub.publish(String(data=state))

    def write_event(self, result: str, reason: str) -> None:
        path = Path(str(self.get_parameter("event_log_dir").value)).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        try:
            path.mkdir(parents=True, exist_ok=True)
            payload = {
                "event_id": self.event_id,
                "result": result,
                "reason": reason,
                "target_class": self.target_class,
                "target_confidence": self.target_confidence,
                "bird_confirmed": self.bird_confirmed,
                "fusion_valid": self.fusion_valid,
                "dynamic_valid": self.dynamic_valid,
                "camera_centered": self.camera_centered,
                "enable_sound_output": bool(self.get_parameter("enable_sound_output").value),
                "hardware_backend": str(self.get_parameter("hardware_backend").value),
                "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }
            (path / f"sound_event_{int(time.time())}_{self.event_id or 'unknown'}.json").write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:  # pragma: no cover - filesystem dependent
            self.get_logger().warn(f"failed to write sound event log: {exc}")

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SoundDeterrentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
