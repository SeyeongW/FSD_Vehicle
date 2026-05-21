from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from waver_patrol.mission.mission_utils import MissionRoute, Waypoint, load_route_yaml


class MissionState(str, Enum):
    IDLE = "IDLE"
    PATROL_NAVIGATING = "PATROL_NAVIGATING"
    PATROL_DWELL = "PATROL_DWELL"
    PATROL_INTERRUPTED_BY_RADAR_TARGET = "PATROL_INTERRUPTED_BY_RADAR_TARGET"
    TARGET_NAVIGATING = "TARGET_NAVIGATING"
    TARGET_REACHED = "TARGET_REACHED"
    TARGET_REDETECTION_WAIT = "TARGET_REDETECTION_WAIT"
    TARGET_CLASSIFICATION_WAIT = "TARGET_CLASSIFICATION_WAIT"
    TARGET_CONFIRMED_BIRD = "TARGET_CONFIRMED_BIRD"
    TARGET_NOT_BIRD = "TARGET_NOT_BIRD"
    SOUND_TASK_RUNNING = "SOUND_TASK_RUNNING"
    SOUND_TASK_DONE = "SOUND_TASK_DONE"
    RETURN_TO_INTERRUPTED_WAYPOINT = "RETURN_TO_INTERRUPTED_WAYPOINT"
    RESUME_PATROL = "RESUME_PATROL"
    BATTERY_RETURN_NAVIGATING = "BATTERY_RETURN_NAVIGATING"
    HOLD_AT_HOME = "HOLD_AT_HOME"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"
    NAV2_RECOVERY = "NAV2_RECOVERY"
    NAV2_FAILED = "NAV2_FAILED"
    SENSOR_STALE_STOP = "SENSOR_STALE_STOP"


class NavGoalType(str, Enum):
    NONE = "NONE"
    PATROL = "PATROL"
    RADAR_TARGET = "RADAR_TARGET"
    RETURN_TO_INTERRUPTED_WAYPOINT = "RETURN_TO_INTERRUPTED_WAYPOINT"
    BATTERY_RETURN = "BATTERY_RETURN"


TARGET_MISSION_STATES = {
    MissionState.PATROL_INTERRUPTED_BY_RADAR_TARGET,
    MissionState.TARGET_NAVIGATING,
    MissionState.TARGET_REACHED,
    MissionState.TARGET_REDETECTION_WAIT,
    MissionState.TARGET_CLASSIFICATION_WAIT,
    MissionState.TARGET_CONFIRMED_BIRD,
    MissionState.TARGET_NOT_BIRD,
    MissionState.SOUND_TASK_RUNNING,
    MissionState.SOUND_TASK_DONE,
    MissionState.RETURN_TO_INTERRUPTED_WAYPOINT,
}


class MissionPatrolManagerNode(Node):
    """Mission-level patrol manager driven by Nav2 NavigateToPose.

    역할:
      - 순찰 waypoint를 Nav2 goal로 보낸다.
      - 4D radar/object mission goal이 들어오면 현재 waypoint를 저장하고 목표 임무로 전환한다.
      - bird confirmed 후 sound stub를 요청하고, 완료 후 interrupted waypoint로 복귀한다.
      - 배터리 복귀와 E-Stop은 모든 임무보다 우선한다.

    안전:
      - 이 노드는 `/cmd_vel`을 절대 발행하지 않는다. Nav2 controller 출력은 launch remap으로
        `/waver/cmd_vel_nav2`에만 연결하고, 최종 `/cmd_vel`은 safety_cmd_mux_node가 발행한다.
    """

    def __init__(self) -> None:
        super().__init__("mission_patrol_manager_node")
        self.declare_parameter("waypoint_file", "waypoints/waver_nav2_patrol_mission.yaml")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("patrol_loop", True)
        self.declare_parameter("default_mode", "AUTO")
        self.declare_parameter("resume_policy", "RETURN_TO_INTERRUPTED_WAYPOINT")
        self.declare_parameter("target_mission_timeout_sec", 120.0)
        self.declare_parameter("target_goal_tolerance_xy", 0.5)
        self.declare_parameter("target_re_detection_timeout_sec", 20.0)
        self.declare_parameter("target_classification_timeout_sec", 15.0)
        self.declare_parameter("bird_confidence_threshold", 0.70)
        self.declare_parameter("patrol_goal_timeout_sec", 180.0)
        self.declare_parameter("waypoint_dwell_sec_default", 2.0)
        self.declare_parameter("sound_task_timeout_sec", 15.0)
        self.declare_parameter("nav2_action_timeout_sec", 5.0)
        self.declare_parameter("nav2_result_timeout_sec", 240.0)
        self.declare_parameter("cancel_timeout_sec", 2.0)
        self.declare_parameter("max_goal_retries", 2)
        self.declare_parameter("recovery_retry_count", 2)
        self.declare_parameter("sim_nav_goal_arrived_topic", "/waver/sim_nav_goal_arrived")
        self.declare_parameter("enable_sim_nav_goal_arrival", False)
        self.declare_parameter("ignore_new_target_during_mission", True)
        self.declare_parameter("update_target_if_closer", False)
        self.declare_parameter("update_target_if_higher_priority", True)
        self.declare_parameter("target_update_cooldown_sec", 5.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")

        share = get_package_share_directory("waver_patrol")
        self.route: MissionRoute = load_route_yaml(str(self.get_parameter("waypoint_file").value), share)
        if not self.route.waypoints:
            self.get_logger().error("Waypoint list is empty. Mission manager will stay IDLE.")

        self.state = MissionState.IDLE
        self.mode = str(self.get_parameter("default_mode").value).strip().upper()
        self.current_index = 0
        self.interrupted_index: Optional[int] = None
        self.interrupted_goal: Optional[PoseStamped] = None
        self.object_goal: Optional[PoseStamped] = None
        self.object_goal_active = False
        self.return_home_active = False
        self.return_goal: Optional[PoseStamped] = None
        self.sound_task_done = False
        self.bird_confirmed = False
        self.target_class = "unknown"
        self.target_confidence = 0.0
        self.estop = False
        self.external_stop = False
        self.active_goal_type = NavGoalType.NONE
        self.active_goal_pose: Optional[PoseStamped] = None
        self.goal_handle = None
        self.goal_start_time = 0.0
        self.state_enter_time = self._now()
        self.goal_retry_count = 0
        self.mission_id = 0
        self.target_mission_id = 0

        self.state_pub = self.create_publisher(String, "/waver/mission_state", 10)
        self.event_pub = self.create_publisher(String, "/waver/mission_event", 10)
        self.current_wp_pub = self.create_publisher(PoseStamped, "/waver/current_waypoint", 10)
        self.active_goal_pub = self.create_publisher(PoseStamped, "/waver/active_nav_goal", 10)
        self.sound_request_pub = self.create_publisher(Bool, "/waver/sound_alert_request", 10)
        self.index_pub = self.create_publisher(Int32, "/waver/patrol_index", 10)

        self.create_subscription(PoseStamped, "/waver/object_mission_goal", self.object_goal_callback, 10)
        self.create_subscription(Bool, "/waver/object_mission_goal_active", lambda m: setattr(self, "object_goal_active", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/return_home_active", lambda m: setattr(self, "return_home_active", bool(m.data)), 10)
        self.create_subscription(PoseStamped, "/waver/return_goal", lambda m: setattr(self, "return_goal", m), 10)
        self.create_subscription(String, "/waver/mode", lambda m: setattr(self, "mode", m.data.strip().upper()), 10)
        self.create_subscription(Bool, "/waver/emergency_stop", lambda m: setattr(self, "estop", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/external_stop", lambda m: setattr(self, "external_stop", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/sound_task_done", lambda m: setattr(self, "sound_task_done", bool(m.data)), 10)
        self.create_subscription(Bool, "/waver/bird_confirmed", lambda m: setattr(self, "bird_confirmed", bool(m.data)), 10)
        self.create_subscription(String, "/waver/target_class", lambda m: setattr(self, "target_class", m.data), 10)
        self.create_subscription(Float32, "/waver/target_confidence", lambda m: setattr(self, "target_confidence", float(m.data)), 10)

        self.use_nav2 = bool(self.get_parameter("use_nav2").value)
        if (not self.use_nav2) or bool(self.get_parameter("enable_sim_nav_goal_arrival").value):
            self.create_subscription(
                Bool,
                str(self.get_parameter("sim_nav_goal_arrived_topic").value),
                self.sim_goal_arrived_callback,
                10,
            )
        self.nav_client = ActionClient(self, NavigateToPose, str(self.get_parameter("nav2_action_name").value)) if self.use_nav2 else None
        self.create_timer(0.2, self.tick)
        self.publish_event("STARTUP", "mission_patrol_manager initialized")

    def object_goal_callback(self, msg: PoseStamped) -> None:
        # 역할: radar/3D LiDAR가 같은 객체 좌표를 반복 발행해도 target mission, sound task,
        # interrupted waypoint 복귀 중에는 새 목표로 재진입하지 않는다.
        if self.target_mission_active() and bool(self.get_parameter("ignore_new_target_during_mission").value):
            self.publish_event("TARGET_IGNORED", f"already_in_target_mission state={self.state.value}")
            return
        self.object_goal = msg
        self.object_goal_active = True

    def sim_goal_arrived_callback(self, msg: Bool) -> None:
        # 역할: Gazebo/headless 반복검증에서 Nav2 action server 없이 goal 성공을 흉내 낸다.
        # 실차에서는 use_nav2=true라 이 경로가 비활성화되고, 실제 Nav2 result만 사용한다.
        if not bool(msg.data):
            return
        if self.use_nav2 and not bool(self.get_parameter("enable_sim_nav_goal_arrival").value):
            return
        if self.active_goal_type == NavGoalType.NONE:
            return
        self.publish_event("SIM_NAV_GOAL_ARRIVED", self.active_goal_type.value)
        self.handle_goal_success()

    def tick(self) -> None:
        self.publish_status()
        if self.estop or self.external_stop or self.mode in {"EMERGENCY", "DISABLED"}:
            self.handle_stop("emergency_or_external_stop")
            return
        if self.mode in {"STANDBY", "MANUAL"}:
            self.cancel_active_goal("mode_blocks_mission")
            self.set_state(MissionState.IDLE, f"mode={self.mode}")
            return
        if self.return_home_active:
            self.handle_battery_return()
            return
        if self.state == MissionState.HOLD_AT_HOME:
            return
        if self.object_goal_active and self.object_goal is not None:
            if self.target_mission_active() and bool(self.get_parameter("ignore_new_target_during_mission").value):
                self.object_goal_active = False
                self.publish_event("TARGET_IGNORED_ACTIVE_MISSION", f"state={self.state.value}")
                return
            self.start_target_mission()
            return
        if self.state in {MissionState.TARGET_REACHED, MissionState.TARGET_REDETECTION_WAIT, MissionState.TARGET_CLASSIFICATION_WAIT}:
            self.handle_target_confirmation()
            return
        if self.state == MissionState.SOUND_TASK_RUNNING:
            self.handle_sound_task()
            return
        if self.state in {MissionState.SOUND_TASK_DONE, MissionState.TARGET_NOT_BIRD}:
            self.return_to_interrupted_waypoint()
            return
        if self.state == MissionState.PATROL_DWELL:
            if self._now() - self.state_enter_time >= self.current_waypoint().dwell_s:
                self.advance_waypoint()
            return
        if self.state in {MissionState.IDLE, MissionState.RESUME_PATROL, MissionState.NAV2_FAILED}:
            self.start_patrol_goal()
            return
        self.check_goal_timeout()

    def start_patrol_goal(self) -> None:
        if not self.route.waypoints:
            self.set_state(MissionState.IDLE, "no_waypoints")
            return
        self.send_nav_goal(self.current_waypoint().pose, NavGoalType.PATROL, MissionState.PATROL_NAVIGATING)

    def start_target_mission(self) -> None:
        if self.object_goal is None:
            return
        if self.target_mission_active() and bool(self.get_parameter("ignore_new_target_during_mission").value):
            # 역할: radar/object 후보가 계속 들어와도 현재 Nav2 target goal을 반복 cancel하지 않는다.
            # 실차에서 goal spam은 회피/재계획 안정성을 해치므로 sound/return까지 현재 target mission을 유지한다.
            self.object_goal_active = False
            self.publish_event("TARGET_IGNORED_ACTIVE_MISSION", f"state={self.state.value}")
            return
        if self.active_goal_type != NavGoalType.RADAR_TARGET:
            self.interrupted_index = self.current_index
            self.interrupted_goal = self.current_waypoint().pose if self.route.waypoints else None
            self.target_mission_id += 1
            self.publish_event("PATROL_INTERRUPTED_BY_RADAR_TARGET", f"target_mission_id={self.target_mission_id}")
        self.object_goal_active = False
        self.cancel_active_goal("target_mission_interrupt")
        self.send_nav_goal(self.object_goal, NavGoalType.RADAR_TARGET, MissionState.TARGET_NAVIGATING)

    def handle_target_confirmation(self) -> None:
        elapsed = self._now() - self.state_enter_time
        if self.state == MissionState.TARGET_REACHED:
            self.set_state(MissionState.TARGET_CLASSIFICATION_WAIT, "target_goal_reached")
            return
        threshold = float(self.get_parameter("bird_confidence_threshold").value)
        if self.bird_confirmed and self.target_confidence >= threshold:
            self.set_state(MissionState.TARGET_CONFIRMED_BIRD, f"class={self.target_class} conf={self.target_confidence:.2f}")
            self.sound_request_pub.publish(Bool(data=True))
            self.set_state(MissionState.SOUND_TASK_RUNNING, "sound_alert_requested")
            return
        if elapsed > float(self.get_parameter("target_classification_timeout_sec").value):
            self.set_state(MissionState.TARGET_NOT_BIRD, f"classification_timeout class={self.target_class}")
            return

    def handle_sound_task(self) -> None:
        self.sound_request_pub.publish(Bool(data=True))
        if self.sound_task_done:
            self.sound_request_pub.publish(Bool(data=False))
            self.set_state(MissionState.SOUND_TASK_DONE, "sound_task_done")
        elif self._now() - self.state_enter_time > float(self.get_parameter("sound_task_timeout_sec").value):
            self.sound_request_pub.publish(Bool(data=False))
            self.set_state(MissionState.SOUND_TASK_DONE, "sound_task_timeout_safe_completion")

    def return_to_interrupted_waypoint(self) -> None:
        if self.interrupted_goal is None:
            self.set_state(MissionState.RESUME_PATROL, "no_interrupted_goal")
            return
        self.send_nav_goal(
            self.interrupted_goal,
            NavGoalType.RETURN_TO_INTERRUPTED_WAYPOINT,
            MissionState.RETURN_TO_INTERRUPTED_WAYPOINT,
        )

    def handle_battery_return(self) -> None:
        if self.active_goal_type == NavGoalType.BATTERY_RETURN:
            self.check_goal_timeout()
            return
        self.cancel_active_goal("battery_return_preempts_all")
        goal = self.return_goal if self.return_goal is not None else self.route.charge
        self.send_nav_goal(goal, NavGoalType.BATTERY_RETURN, MissionState.BATTERY_RETURN_NAVIGATING)

    def handle_stop(self, reason: str) -> None:
        self.cancel_active_goal(reason)
        self.sound_request_pub.publish(Bool(data=False))
        self.set_state(MissionState.EMERGENCY_STOPPED, reason)

    def send_nav_goal(self, goal: PoseStamped, goal_type: NavGoalType, state: MissionState) -> None:
        if self.active_goal_type == goal_type and self.active_goal_pose is not None:
            return
        self.active_goal_type = goal_type
        self.active_goal_pose = goal
        self.goal_start_time = self._now()
        self.goal_retry_count = 0
        goal.header.stamp = self.get_clock().now().to_msg()
        self.active_goal_pub.publish(goal)
        self.set_state(state, f"goal_type={goal_type.value}")
        if not self.use_nav2:
            self.publish_event("NAV2_DISABLED_SIMULATED_GOAL", goal_type.value)
            return
        if self.nav_client is None or not self.nav_client.wait_for_server(timeout_sec=float(self.get_parameter("nav2_action_timeout_sec").value)):
            self.set_state(MissionState.NAV2_FAILED, "action_server_unavailable")
            return
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal
        future = self.nav_client.send_goal_async(action_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.set_state(MissionState.NAV2_FAILED, f"goal_send_exception={exc}")
            return
        if not self.goal_handle.accepted:
            self.set_state(MissionState.NAV2_FAILED, "goal_rejected")
            return
        self.publish_event("NAV2_GOAL_ACCEPTED", self.active_goal_type.value)
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.set_state(MissionState.NAV2_FAILED, f"result_exception={exc}")
            return
        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.handle_goal_success()
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_event("NAV2_GOAL_CANCELED", self.active_goal_type.value)
        else:
            self.handle_goal_failure(f"status={status}")

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        recoveries = getattr(feedback, "number_of_recoveries", 0)
        if recoveries and self.state != MissionState.NAV2_RECOVERY:
            self.set_state(MissionState.NAV2_RECOVERY, f"recoveries={recoveries}")

    def handle_goal_success(self) -> None:
        goal_type = self.active_goal_type
        self.publish_event("NAV2_GOAL_SUCCEEDED", goal_type.value)
        self.goal_handle = None
        self.active_goal_type = NavGoalType.NONE
        if goal_type == NavGoalType.PATROL:
            self.set_state(MissionState.PATROL_DWELL, self.current_waypoint().name)
        elif goal_type == NavGoalType.RADAR_TARGET:
            self.set_state(MissionState.TARGET_REACHED, "target_goal_reached")
        elif goal_type == NavGoalType.RETURN_TO_INTERRUPTED_WAYPOINT:
            if self.interrupted_index is not None:
                self.current_index = self.interrupted_index
            self.set_state(MissionState.RESUME_PATROL, "interrupted_waypoint_reached")
        elif goal_type == NavGoalType.BATTERY_RETURN:
            self.set_state(MissionState.HOLD_AT_HOME, "battery_return_arrived")

    def handle_goal_failure(self, reason: str) -> None:
        self.publish_event("NAV2_GOAL_FAILED", reason)
        if self.goal_retry_count < int(self.get_parameter("max_goal_retries").value):
            self.goal_retry_count += 1
            if self.active_goal_pose is not None:
                pose = self.active_goal_pose
                goal_type = self.active_goal_type
                state = self.state
                self.active_goal_type = NavGoalType.NONE
                self.goal_handle = None
                self.send_nav_goal(pose, goal_type, state)
                return
        self.goal_handle = None
        self.active_goal_type = NavGoalType.NONE
        self.set_state(MissionState.NAV2_FAILED, reason)

    def check_goal_timeout(self) -> None:
        if self.active_goal_type == NavGoalType.NONE:
            return
        timeout = float(self.get_parameter("nav2_result_timeout_sec").value)
        if self.active_goal_type == NavGoalType.PATROL:
            timeout = float(self.get_parameter("patrol_goal_timeout_sec").value)
        elif self.active_goal_type == NavGoalType.RADAR_TARGET:
            timeout = float(self.get_parameter("target_mission_timeout_sec").value)
        if self._now() - self.goal_start_time > timeout:
            self.cancel_active_goal("goal_timeout")
            self.handle_goal_failure("goal_timeout")

    def cancel_active_goal(self, reason: str) -> None:
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.publish_event("NAV2_CANCEL_REQUESTED", reason)
            except Exception as exc:
                self.publish_event("NAV2_CANCEL_FAILED", f"{reason} exception={exc}")
        self.goal_handle = None
        self.active_goal_type = NavGoalType.NONE
        self.active_goal_pose = None

    def current_waypoint(self) -> Waypoint:
        return self.route.waypoints[self.current_index % len(self.route.waypoints)]

    def target_mission_active(self) -> bool:
        # 역할: target 접근, 재분류, 음향 업무, interrupted waypoint 복귀를 하나의 원자적 임무로 본다.
        # 이 구간에서 반복 radar 좌표가 들어와도 현재 임무를 깨지 않아야 순찰 복귀가 보장된다.
        return self.active_goal_type == NavGoalType.RADAR_TARGET or self.state in TARGET_MISSION_STATES

    def advance_waypoint(self) -> None:
        self.current_index += 1
        if self.current_index >= len(self.route.waypoints):
            if bool(self.get_parameter("patrol_loop").value) and self.route.loop:
                self.current_index = 0
            else:
                self.set_state(MissionState.IDLE, "route_complete")
                return
        self.set_state(MissionState.RESUME_PATROL, f"next_index={self.current_index}")

    def set_state(self, new_state: MissionState, reason: str) -> None:
        if self.state == new_state:
            return
        previous = self.state
        self.state = new_state
        self.state_enter_time = self._now()
        self.publish_event("STATE_CHANGE", f"previous={previous.value} next={new_state.value} reason={reason}")

    def publish_status(self) -> None:
        self.state_pub.publish(String(data=f"{self.state.value} mode={self.mode} nav_goal={self.active_goal_type.value}"))
        self.index_pub.publish(Int32(data=int(self.current_index)))
        if self.route.waypoints:
            wp = self.current_waypoint().pose
            wp.header.stamp = self.get_clock().now().to_msg()
            self.current_wp_pub.publish(wp)

    def publish_event(self, event_type: str, reason: str) -> None:
        text = (
            f"timestamp={self._now():.3f} mission_id={self.mission_id} target_mission_id={self.target_mission_id} "
            f"state={self.state.value} event_type={event_type} reason={reason} "
            f"waypoint_index={self.current_index}"
        )
        self.event_pub.publish(String(data=text))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionPatrolManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cancel_active_goal("shutdown")
            node.sound_request_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
