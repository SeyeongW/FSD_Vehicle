from __future__ import annotations

import time
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


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
        self.declare_parameter("gpio_pin", -1)
        self.declare_parameter("alsa_device", "default")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("legal_safety_ack_required", True)
        self.declare_parameter("safety_ack", False)

        self.request_active = False
        self.task_active = False
        self.task_started_time = 0.0
        self.last_completed_time = -1e9
        self.target_class = "none"
        self.target_confidence = 0.0
        self.bird_confirmed = False
        self.emergency_stop = False
        self.external_stop = False
        self.event_id = ""

        self.state_pub = self.create_publisher(String, "/waver/sound_alert_state", 10)
        self.done_pub = self.create_publisher(Bool, "/waver/sound_task_done", 10)
        self.active_pub = self.create_publisher(Bool, "/waver/sound_task_active", 10)
        self.event_id_pub = self.create_publisher(String, "/waver/sound_event_id", 10)

        self.create_subscription(Bool, "/waver/sound_alert_request", self.request_callback, 10)
        self.create_subscription(String, "/waver/target_class", lambda m: setattr(self, "target_class", m.data.strip().lower()), 10)
        self.create_subscription(Float32, "/waver/target_confidence", lambda m: setattr(self, "target_confidence", float(m.data)), 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
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
        self.event_id_pub.publish(String(data=self.event_id))
        self.done_pub.publish(Bool(data=False))
        self.active_pub.publish(Bool(data=True))
        self.publish_state(
            "SOUND_TASK_RUNNING "
            f"event_id={self.event_id} target_class={self.target_class} "
            f"confidence={self.target_confidence:.2f} sound_type={self.get_parameter('sound_type').value} "
            f"enable_sound_output={bool(self.get_parameter('enable_sound_output').value)}"
        )

    def block_reason(self, now: float) -> str:
        if self.emergency_stop or self.external_stop:
            return "SAFETY_STOP"
        classes = {str(v).strip().lower() for v in self.get_parameter("deterrence_classes").value}
        if self.target_class not in classes:
            return f"BY_CLASS class={self.target_class} deterrence_classes={sorted(classes)}"
        if self.target_class == "bird" and not self.bird_confirmed:
            return "NOT_BIRD bird_confirmed=false"
        min_conf = float(self.get_parameter("min_confidence_for_sound").value)
        if self.target_confidence < min_conf:
            return f"LOW_CONFIDENCE confidence={self.target_confidence:.2f} threshold={min_conf:.2f}"
        cooldown = float(self.get_parameter("alert_cooldown_sec").value)
        if now - self.last_completed_time < cooldown:
            return f"COOLDOWN remaining_sec={cooldown - (now - self.last_completed_time):.2f}"
        if bool(self.get_parameter("enable_sound_output").value):
            backend = str(self.get_parameter("hardware_backend").value).strip().lower()
            if bool(self.get_parameter("legal_safety_ack_required").value) and not bool(self.get_parameter("safety_ack").value):
                return "SAFETY_ACK_REQUIRED"
            if backend == "disabled":
                return "HARDWARE_BACKEND_DISABLED"
        return ""

    def publish_state(self, state: str) -> None:
        self.state_pub.publish(String(data=state))

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
