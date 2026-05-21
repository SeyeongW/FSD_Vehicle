from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SoundAlertStub(Node):
    """Safe placeholder for future sound output.

    enable_sound_output defaults false so real speakers/amps cannot fire by accident.
    """

    def __init__(self) -> None:
        super().__init__("sound_alert_stub")
        self.declare_parameter("sound_alert_request_topic", "/waver/sound_alert_request")
        self.declare_parameter("target_class_topic", "/waver/target_class")
        self.declare_parameter("bird_confirmed_topic", "/waver/bird_confirmed")
        self.declare_parameter("mode_topic", "/waver/mode")
        self.declare_parameter("default_mode", "AUTO")
        self.declare_parameter("state_topic", "/waver/sound_alert_state")
        self.declare_parameter("done_topic", "/waver/sound_task_done")
        self.declare_parameter("enable_sound_output", False)
        self.declare_parameter("sound_task_duration_sec", 5.0)
        self.declare_parameter("alert_cooldown_sec", 10.0)
        self.declare_parameter("require_auto_mode", True)
        self.declare_parameter("sound_type", "SIMULATED_GUNSHOT")
        self.bird_confirmed = False
        self.requested = False
        self.running = False
        self.task_start_time = 0.0
        self.target_class = "unknown"
        self.mode = str(self.get_parameter("default_mode").value).strip().upper()
        self.last_alert_time = -1e9
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.done_pub = self.create_publisher(Bool, str(self.get_parameter("done_topic").value), 10)
        self.create_subscription(Bool, str(self.get_parameter("sound_alert_request_topic").value), self.request_callback, 10)
        self.create_subscription(String, str(self.get_parameter("target_class_topic").value), self.class_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("bird_confirmed_topic").value), self.bird_callback, 10)
        self.create_subscription(String, str(self.get_parameter("mode_topic").value), self.mode_callback, 10)
        self.create_timer(0.2, self.tick)

    def request_callback(self, msg: Bool) -> None:
        self.requested = bool(msg.data)

    def class_callback(self, msg: String) -> None:
        self.target_class = msg.data

    def bird_callback(self, msg: Bool) -> None:
        self.bird_confirmed = bool(msg.data)

    def mode_callback(self, msg: String) -> None:
        self.mode = msg.data.strip().upper()

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.done_pub.publish(Bool(data=False))
        if self.running:
            duration = float(self.get_parameter("sound_task_duration_sec").value)
            if now - self.task_start_time >= duration:
                self.running = False
                self.last_alert_time = now
                self.done_pub.publish(Bool(data=True))
                self.state_pub.publish(String(data=f"SOUND_TASK_DONE sound_type={self.get_parameter('sound_type').value}"))
            else:
                self.state_pub.publish(String(data=f"SOUND_TASK_RUNNING enable_sound_output={bool(self.get_parameter('enable_sound_output').value)}"))
            return
        if not self.requested:
            self.state_pub.publish(String(data="IDLE_NO_REQUEST"))
            return
        if not self.bird_confirmed:
            self.state_pub.publish(String(data=f"REQUEST_BLOCKED_NOT_BIRD class={self.target_class}"))
            return
        if bool(self.get_parameter("require_auto_mode").value) and self.mode not in {"AUTO", "MISSION", "TRACK_ONLY"}:
            self.state_pub.publish(String(data=f"SOUND_BLOCKED_BY_MODE mode={self.mode}"))
            return
        if now - self.last_alert_time < float(self.get_parameter("alert_cooldown_sec").value):
            self.state_pub.publish(String(data="COOLDOWN_BIRD_CONFIRMED"))
            return
        self.running = True
        self.task_start_time = now
        if bool(self.get_parameter("enable_sound_output").value):
            self.state_pub.publish(String(data="SOUND_OUTPUT_REQUESTED_STUB_ONLY_NO_AUDIO_DRIVER"))
        else:
            self.state_pub.publish(
                String(
                    data=(
                        f"SIMULATED_DETERRENT_SOUND_TASK sound_type={self.get_parameter('sound_type').value} "
                        "enable_sound_output=false legal_safety_note=stub_only"
                    )
                )
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SoundAlertStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.state_pub.publish(String(data="SHUTDOWN_SOUND_DISABLED"))
            node.done_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
