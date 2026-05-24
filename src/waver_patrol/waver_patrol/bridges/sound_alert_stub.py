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
        self.declare_parameter("done_latch_sec", 1.2)
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
        self.done_until = 0.0
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
        # 역할: launch 종료 경계에서 timer callback이 늦게 들어와도
        # rcl context-invalid/segfault성 shutdown 문제를 피한다.
        if not rclpy.ok():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.done_pub.publish(Bool(data=now < self.done_until))
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f"Skipping sound done publish: {exc}")
            return
        if self.running:
            duration = float(self.get_parameter("sound_task_duration_sec").value)
            if now - self.task_start_time >= duration:
                self.running = False
                self.last_alert_time = now
                self.done_until = now + float(self.get_parameter("done_latch_sec").value)
                self.safe_publish(done=True, state=f"SOUND_TASK_DONE sound_type={self.get_parameter('sound_type').value}")
            else:
                self.safe_publish(
                    state=f"SOUND_TASK_RUNNING enable_sound_output={bool(self.get_parameter('enable_sound_output').value)}"
                )
            return
        if not self.requested:
            self.safe_publish(state="IDLE_NO_REQUEST")
            return
        if not self.bird_confirmed:
            self.safe_publish(state=f"REQUEST_BLOCKED_NOT_BIRD class={self.target_class}")
            return
        if bool(self.get_parameter("require_auto_mode").value) and self.mode not in {"AUTO", "MISSION", "TRACK_ONLY"}:
            self.safe_publish(state=f"SOUND_BLOCKED_BY_MODE mode={self.mode}")
            return
        if now - self.last_alert_time < float(self.get_parameter("alert_cooldown_sec").value):
            self.safe_publish(state="COOLDOWN_BIRD_CONFIRMED")
            return
        self.running = True
        self.task_start_time = now
        if bool(self.get_parameter("enable_sound_output").value):
            self.safe_publish(state="SOUND_OUTPUT_REQUESTED_STUB_ONLY_NO_AUDIO_DRIVER")
        else:
            self.safe_publish(
                state=(
                    f"SIMULATED_DETERRENT_SOUND_TASK sound_type={self.get_parameter('sound_type').value} "
                    "enable_sound_output=false legal_safety_note=stub_only"
                )
            )

    def safe_publish(self, state: str | None = None, done: bool | None = None) -> None:
        # 역할: 실제 음향 출력 stub는 실험 중 계속 상태만 내보내므로,
        # shutdown 타이밍 예외는 경고 후 skip하여 trial 자체를 깨지 않게 한다.
        if not rclpy.ok():
            return
        try:
            if done is not None:
                self.done_pub.publish(Bool(data=done))
            if state is not None:
                self.state_pub.publish(String(data=state))
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f"Skipping sound state publish: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SoundAlertStub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok() and "context is not valid" not in str(exc):
            raise
    finally:
        if rclpy.ok():
            try:
                node.state_pub.publish(String(data="SHUTDOWN_SOUND_DISABLED"))
                node.done_pub.publish(Bool(data=False))
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
