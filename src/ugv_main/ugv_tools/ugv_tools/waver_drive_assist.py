#!/usr/bin/env python3
# Copyright 2026 Waver project contributors.
"""Shared Waver driving-assist primitives.

Role:
  - Keep keyboard, Gazebo patrol, and real-robot serial bridge behavior aligned.
  - Apply conservative speed, acceleration, stale-command, and scan safety rules.
  - Provide a small "defensive driver" layer inspired by Nav2 Collision Monitor
    and Regulated Pure Pursuit patterns without pretending to replace Nav2.

The module deliberately has no ROS imports so the core logic can be unit-tested and
reused by CLI tools.
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable


# 역할: 속도/거리/위험도 계산에서 공통으로 쓰는 범위 제한 함수다.
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# 역할: 잘못된 센서값이나 명령값을 fail-safe 기본값으로 바꾼다.
def finite_float(value: float, fallback: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


# 역할: LiDAR sector 분석 결과를 사람이 읽을 수 있는 위험 단계로 표현한다.
class HazardLevel(str, Enum):
    CLEAR = "clear"
    CAUTION = "caution"
    SLOW = "slow"
    STOP = "stop"
    SENSOR_STALE = "sensor_stale"
    SENSOR_DEGRADED = "sensor_degraded"


@dataclass
class DriveCommand:
    # 역할: 키보드/순찰/브리지 사이에서 공유하는 내부 속도 명령 형식이다.
    linear: float = 0.0
    angular: float = 0.0
    source: str = "unknown"
    reason: str = "stop"

    @property
    def is_stop(self) -> bool:
        # 역할: 0에 가까운 명령을 정지로 다뤄 미세 노이즈가 모터로 나가지 않게 한다.
        return abs(self.linear) < 1e-6 and abs(self.angular) < 1e-6


@dataclass
class AssistConfig:
    # 역할: 실증 환경에 맞춰 조정할 모든 안전 threshold를 한 구조체에 모은다.
    max_linear_speed: float = 0.28
    max_angular_speed: float = 0.9
    max_linear_accel: float = 0.45
    max_angular_accel: float = 1.8
    deadband_linear: float = 0.005
    deadband_angular: float = 0.005
    command_timeout_s: float = 0.3
    min_rate_dt_s: float = 0.01
    hard_stop_distance_m: float = 0.45
    slow_down_distance_m: float = 1.2
    clear_distance_m: float = 1.8
    ttc_hard_stop_s: float = 1.5
    ttc_slow_down_s: float = 3.0
    scan_stale_s: float = 0.6
    min_valid_scan_points: int = 40
    lidar_required: bool = False
    allow_reverse: bool = True
    reverse_speed: float = 0.08
    require_neutral_before_reverse: bool = True


@dataclass
class ScanSummary:
    # 역할: LaserScan을 전/좌/우/후방 sector별 최소 거리와 TTC 상태로 압축한다.
    front: float = math.inf
    front_left: float = math.inf
    front_right: float = math.inf
    left: float = math.inf
    right: float = math.inf
    rear: float = math.inf
    valid_points: int = 0
    age_s: float = math.inf
    hazard: HazardLevel = HazardLevel.CLEAR
    best_escape: str = "none"
    ttc_s: float = math.inf
    closing_speed_mps: float = 0.0


# 역할: 키보드, Gazebo 순찰, 실차 브리지에서 공통으로 사용하는 주행 보조 계층이다.
class DriveAssist:
    """Small defensive-driving layer.

    Role:
      - Sanitize and rate-limit commands.
      - Use LaserScan sectors as an independent last-line safety check.
      - Select simple wait/turn/backup hints when the forward path is blocked.
    """

    def __init__(self, config: AssistConfig | None = None):
        # 역할: 마지막 출력/입력/scan 시간을 저장해 timeout과 가속 제한을 계산한다.
        self.config = config or AssistConfig()
        self.last_output = DriveCommand()
        self.last_output_time = time.monotonic()
        self.last_input_time = 0.0
        self.last_scan_time = 0.0
        self.scan = ScanSummary()
        self._front_history: list[tuple[float, float]] = []

    def update_config(self, **kwargs) -> None:
        # 역할: launch나 테스트에서 필요한 threshold만 선택적으로 갱신한다.
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)

    def sanitize(self, command: DriveCommand) -> DriveCommand:
        # 역할: NaN/Inf 제거, deadband 적용, 선속도/각속도 상한 clamp를 수행한다.
        linear = finite_float(command.linear)
        angular = finite_float(command.angular)
        if abs(linear) < self.config.deadband_linear:
            linear = 0.0
        if abs(angular) < self.config.deadband_angular:
            angular = 0.0
        reverse_limit = min(self.config.max_linear_speed, self.config.reverse_speed)
        if not self.config.allow_reverse:
            reverse_limit = 0.0
        return DriveCommand(
            linear=clamp(linear, -reverse_limit, self.config.max_linear_speed),
            angular=clamp(angular, -self.config.max_angular_speed, self.config.max_angular_speed),
            source=command.source,
            reason=command.reason,
        )

    def update_scan(
        self,
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        stamp_time: float | None = None,
        now: float | None = None,
    ) -> ScanSummary:
        # 역할: LaserScan 원시 range를 sector별 robust minimum과 TTC 후보로 변환한다.
        now = time.monotonic() if now is None else now
        stamp_time = now if stamp_time is None else stamp_time
        sectors: dict[str, list[float]] = {
            "front": [],
            "front_left": [],
            "front_right": [],
            "left": [],
            "right": [],
            "rear": [],
        }
        valid_points = 0
        for index, raw_range in enumerate(ranges):
            # 역할: range_min/range_max 밖 값과 NaN/Inf는 장애물 판단에서 제외한다.
            distance = finite_float(raw_range, math.inf)
            if distance < range_min or distance > range_max:
                continue
            angle = self._normalize_angle(angle_min + index * angle_increment)
            valid_points += 1
            if abs(angle) <= math.radians(18):
                sectors["front"].append(distance)
            elif math.radians(18) < angle <= math.radians(60):
                sectors["front_left"].append(distance)
            elif math.radians(-60) <= angle < math.radians(-18):
                sectors["front_right"].append(distance)
            elif math.radians(60) < angle <= math.radians(135):
                sectors["left"].append(distance)
            elif math.radians(-135) <= angle < math.radians(-60):
                sectors["right"].append(distance)
            elif abs(angle) >= math.radians(135):
                sectors["rear"].append(distance)

        summary = ScanSummary(
            front=self._robust_min(sectors["front"]),
            front_left=self._robust_min(sectors["front_left"]),
            front_right=self._robust_min(sectors["front_right"]),
            left=self._robust_min(sectors["left"]),
            right=self._robust_min(sectors["right"]),
            rear=self._robust_min(sectors["rear"]),
            valid_points=valid_points,
            age_s=max(0.0, now - stamp_time),
        )
        self._front_history.append((now, summary.front))
        self._front_history = self._front_history[-6:]
        summary.closing_speed_mps = self._closing_speed()
        if summary.closing_speed_mps > 0.01:
            summary.ttc_s = summary.front / summary.closing_speed_mps
        else:
            summary.ttc_s = math.inf
        summary.hazard = self._hazard_from_summary(summary)
        summary.best_escape = self._best_escape(summary)
        self.scan = summary
        self.last_scan_time = now
        return summary

    def assisted_command(self, command: DriveCommand, now: float | None = None) -> DriveCommand:
        # 역할: 최종 명령 송신 직전에 speed limit, scan hard stop, acceleration limit을 적용한다.
        now = time.monotonic() if now is None else now
        command = self.sanitize(command)
        self.last_input_time = now

        scan = self.scan
        if self.last_scan_time > 0.0:
            scan.age_s = max(0.0, now - self.last_scan_time)
            scan.hazard = self._hazard_from_summary(scan)
        if self.config.lidar_required and scan.age_s > self.config.scan_stale_s:
            # 역할: 실차에서 LiDAR 필수 모드면 scan stale은 무조건 정지로 취급한다.
            command = DriveCommand(source=command.source, reason="lidar stale required stop")
        stop_hazards = {
            HazardLevel.STOP,
            HazardLevel.SENSOR_STALE,
            HazardLevel.SENSOR_DEGRADED,
        }
        if command.linear > 0.0 and scan.hazard in stop_hazards:
            command = DriveCommand(source=command.source, reason=f"{scan.hazard.value} stop")
        elif command.linear > 0.0 and scan.hazard == HazardLevel.SLOW:
            scale = clamp(
                (scan.front - self.config.hard_stop_distance_m)
                / max(self.config.slow_down_distance_m, 0.1),
                0.15,
                0.6,
            )
            command = DriveCommand(
                linear=command.linear * scale,
                angular=command.angular,
                source=command.source,
                reason=f"scan slow {scan.front:.2f}m",
            )
        elif command.linear > 0.0 and scan.hazard == HazardLevel.CAUTION:
            command = DriveCommand(
                linear=command.linear * 0.8,
                angular=command.angular,
                source=command.source,
                reason="caution speed trim",
            )

        command = self._rate_limit(command, now)
        self.last_output = command
        self.last_output_time = now
        return command

    def stale_safe_command(self, now: float | None = None) -> DriveCommand:
        # 역할: 입력이 일정 시간 갱신되지 않으면 정지를 반환하는 보조 함수다.
        now = time.monotonic() if now is None else now
        if (
            self.last_input_time == 0.0
            or now - self.last_input_time > self.config.command_timeout_s
        ):
            self.last_output = DriveCommand(reason="command stale stop")
            self.last_output_time = now
        return self.last_output

    def recovery_hint(self) -> str:
        # 역할: 전방이 막혔을 때 좌/우 회피 또는 후진 가능 여부를 간단히 추천한다.
        scan = self.scan
        if scan.hazard != HazardLevel.STOP:
            return "cruise"
        if scan.best_escape == "left":
            return "turn_left"
        if scan.best_escape == "right":
            return "turn_right"
        if self.config.allow_reverse and scan.rear > self.config.clear_distance_m:
            return "backup"
        return "wait"

    def _rate_limit(self, command: DriveCommand, now: float) -> DriveCommand:
        # 역할: 급격한 선속도/각속도 변화량을 tick 간 최대 가속도로 제한한다.
        stop_tokens = ("stop", "stale", "timeout", "invalid")
        if command.is_stop and any(token in command.reason for token in stop_tokens):
            return command
        dt = clamp(now - self.last_output_time, self.config.min_rate_dt_s, 0.25)
        if (
            self.config.require_neutral_before_reverse
            and self.last_output.linear * command.linear < 0.0
            and abs(self.last_output.linear) > self.config.deadband_linear
        ):
            # 역할: 실차에서 전진/후진을 바로 반전하지 않고 먼저 중립 정지를 한 tick 거친다.
            return DriveCommand(
                0.0,
                0.0,
                source=command.source,
                reason="neutral before reverse",
            )
        max_dl = self.config.max_linear_accel * dt
        max_da = self.config.max_angular_accel * dt
        linear = self.last_output.linear + clamp(
            command.linear - self.last_output.linear,
            -max_dl,
            max_dl,
        )
        angular = self.last_output.angular + clamp(
            command.angular - self.last_output.angular,
            -max_da,
            max_da,
        )
        return DriveCommand(
            linear=linear,
            angular=angular,
            source=command.source,
            reason=command.reason,
        )

    def _hazard_from_summary(self, summary: ScanSummary) -> HazardLevel:
        # 역할: 거리/TTC/valid point 수로 hard stop, slow down, degraded 상태를 결정한다.
        if summary.age_s > self.config.scan_stale_s:
            return HazardLevel.SENSOR_STALE if self.config.lidar_required else HazardLevel.CLEAR
        if summary.valid_points < self.config.min_valid_scan_points:
            if self.config.lidar_required:
                return HazardLevel.SENSOR_DEGRADED
            return HazardLevel.CAUTION
        if summary.front <= self.config.hard_stop_distance_m:
            return HazardLevel.STOP
        if summary.ttc_s <= self.config.ttc_hard_stop_s:
            return HazardLevel.STOP
        if (
            summary.front <= self.config.slow_down_distance_m
            or summary.ttc_s <= self.config.ttc_slow_down_s
        ):
            return HazardLevel.SLOW
        if summary.front <= self.config.clear_distance_m:
            return HazardLevel.CAUTION
        return HazardLevel.CLEAR

    def _best_escape(self, summary: ScanSummary) -> str:
        # 역할: 좌/우 sector 중 clearance가 더 큰 방향을 회피 후보로 고른다.
        left_clearance = min(summary.front_left, summary.left)
        right_clearance = min(summary.front_right, summary.right)
        if (
            left_clearance < self.config.hard_stop_distance_m
            and right_clearance < self.config.hard_stop_distance_m
        ):
            return "none"
        return "left" if left_clearance >= right_clearance else "right"

    def _closing_speed(self) -> float:
        # 역할: front 거리 변화량으로 접근 속도와 TTC 계산에 쓸 closing speed를 추정한다.
        if len(self._front_history) < 2:
            return 0.0
        old_t, old_d = self._front_history[0]
        new_t, new_d = self._front_history[-1]
        dt = new_t - old_t
        if dt <= 1e-3 or not math.isfinite(old_d) or not math.isfinite(new_d):
            return 0.0
        return max(0.0, (old_d - new_d) / dt)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        # 역할: LaserScan 각도를 -pi~pi 범위로 맞춰 sector 분류를 안정화한다.
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _robust_min(values: list[float]) -> float:
        # 역할: 튀는 LiDAR 값 하나가 hard stop을 만들지 않도록 작은 값 몇 개의 median을 쓴다.
        if not values:
            return math.inf
        values = sorted(values)
        head = values[: max(1, min(5, len(values)))]
        return statistics.median(head)
