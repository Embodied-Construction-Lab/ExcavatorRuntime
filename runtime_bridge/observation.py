"""把 Orin 状态、bucket tip 和规划切片组装成 ONNX 38 维 observation。"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from runtime_bridge.protocol import ACTION_ORDER, MachineStatePacket


DEFAULT_WAYPOINT_VALUES = [0.0] * 12


@dataclass(frozen=True)
class BucketTipObservation:
    """FK 输出的 bucket tip 观测片段，坐标单位 m，pitch 暂按当前链路的 rad。"""

    position_m: tuple[float, float, float]
    pitch_rad: float
    stamp_ms: int | None = None


def load_machine_profile(path: Path) -> dict[str, Any]:
    """读取唯一机型常数来源 machine_profile.json。"""
    profile = json.loads(path.read_text(encoding="utf-8"))
    actuators = profile.get("actuators")
    if isinstance(actuators, Mapping):
        for name, actuator in actuators.items():
            if isinstance(actuator, Mapping):
                normalized_command_deadzones(actuator, actuator_name=str(name))
    return profile


def normalized_command_deadzones(
    actuator: Mapping[str, Any], *, actuator_name: str = "actuator"
) -> tuple[float, float]:
    """读取正负归一化策略命令死区；未配置时保持零死区以支持现场标定。"""
    values: list[float] = []
    for direction in ("positive", "negative"):
        field_name = f"command_deadzone_{direction}_normalized"
        value = actuator.get(field_name, 0.0)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 <= float(value) < 1.0
        ):
            raise ValueError(
                f"{actuator_name}.{field_name} must be finite and in [0, 1)"
            )
        values.append(float(value))
    return values[0], values[1]


def load_waypoint_slice_values(path: Path) -> list[float]:
    """读取 idx 15..26 的 waypoint observation 切片；不存在时返回全 0。"""
    if not path.exists():
        return list(DEFAULT_WAYPOINT_VALUES)
    data = json.loads(path.read_text(encoding="utf-8"))
    values = data.get("values", DEFAULT_WAYPOINT_VALUES)
    if not isinstance(values, list) or len(values) != 12:
        raise ValueError(f"waypoint slice values 必须是长度12数组: {path}")
    return [float(value) for value in values]


def clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    """限制数值范围，匹配 Unity observation 中速度类 clamp 行为。"""
    return max(lower, min(upper, float(value)))


def position_observation_range(actuator: Mapping[str, Any]) -> tuple[float, float]:
    """返回 PC 部署观测范围；未配置时回退到 Unity prismatic 范围。"""
    deploy = actuator.get("deploy_position_observation")
    field_name = "range"
    configured_range = actuator.get("range")
    if deploy is not None:
        required_fields = {"source", "range", "status"}
        if (
            not isinstance(deploy, Mapping)
            or not required_fields.issubset(deploy)
            or set(deploy) - required_fields
        ):
            raise ValueError("deploy_position_observation contract is invalid")
        if deploy.get("source") != "stm32_absolute_cable_encoder":
            raise ValueError("deploy_position_observation source is invalid")
        if deploy.get("status") not in {"firmware_safety_bounds", "field_calibrated"}:
            raise ValueError("deploy_position_observation status is invalid")
        configured_range = deploy.get("range")
        field_name = "deploy_position_observation.range"
    if (
        not isinstance(configured_range, (list, tuple))
        or len(configured_range) != 2
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in configured_range
        )
    ):
        raise ValueError(f"{field_name} must contain two finite numbers")
    lower, upper = (float(value) for value in configured_range)
    if lower >= upper:
        raise ValueError(f"{field_name} lower bound must be less than upper bound")
    return lower, upper


def unity_observation_sign(actuator: Mapping[str, Any]) -> float:
    """返回原始执行器反馈映射到 Unity 策略观测时使用的符号。"""
    observation_sign = actuator.get(
        "deploy_observation_sign", actuator.get("sign", 1)
    )
    if isinstance(observation_sign, bool) or observation_sign not in {-1, 1}:
        raise ValueError("deploy_observation_sign must be -1 or +1")
    return float(observation_sign)


def normalize_position(raw_position: float, actuator: Mapping[str, Any]) -> float:
    """按部署绝对编码器范围（或 Unity 回退范围）映射到 [-1, 1]。"""
    range_min, range_max = position_observation_range(actuator)
    span = range_max - range_min
    sign = unity_observation_sign(actuator)
    normalized = (float(raw_position) - range_min) / span * 2.0 - 1.0
    return clamp(normalized * sign)


def normalize_velocity(raw_velocity: float, actuator: dict[str, Any]) -> float:
    """按正负方向最大速度把速度归一化，和 Unity NormalizedActuatorVelocity 对齐。"""
    sign = unity_observation_sign(actuator)
    signed_velocity = float(raw_velocity) * sign
    max_speed = (
        float(actuator["max_speed_positive"])
        if signed_velocity >= 0.0
        else float(actuator["max_speed_negative"])
    )
    if max_speed <= 1e-9:
        return 0.0
    return clamp(signed_velocity / max_speed)


def delta_angle_deg(previous_deg: float, current_deg: float) -> float:
    """复刻 Unity Mathf.DeltaAngle 的角度差，单位 deg。"""
    delta = (current_deg - previous_deg + 180.0) % 360.0 - 180.0
    return delta


class ObservationBuilder:
    """有状态 observation 构造器，用上一帧估计 tip velocity 和 pitch velocity。"""

    def __init__(self, machine_profile: dict[str, Any], task_mode: str = "MoveToDig") -> None:
        self.machine_profile = machine_profile
        self.task_mode = task_mode
        self.previous_tip: BucketTipObservation | None = None

    def build(
        self,
        state: MachineStatePacket,
        bucket_tip: BucketTipObservation,
        waypoint_values: Sequence[float],
        previous_action: Sequence[float],
        episode_progress: float = 0.0,
    ) -> list[float]:
        """按 scale_excavator_v2_38d 顺序组装完整 38 维 observation。"""
        schema = self.machine_profile["observation_schema"]
        if int(schema["total_dim"]) != 38:
            raise ValueError(f"当前只支持38维observation，实际为 {schema['total_dim']}")

        actuators = self.machine_profile["actuators"]
        normalizers = schema["normalizers"]
        obs: list[float] = []

        # 0..5：三路液压缸位置/速度，顺序 boom, stick, bucket。
        for name in ("boom", "stick", "bucket"):
            actuator_state = state.actuator_state[name]
            actuator_profile = actuators[name]
            obs.append(normalize_position(actuator_state["position_m"], actuator_profile))
            obs.append(normalize_velocity(actuator_state["velocity_mps"], actuator_profile))

        # 6..8：swing 用 sin/cos 表示角度，速度按 max speed 归一化。
        swing_state = state.actuator_state["swing"]
        swing_angle = float(swing_state["position_rad"]) * unity_observation_sign(
            actuators["swing"]
        )
        obs.extend(
            [
                math.sin(swing_angle),
                math.cos(swing_angle),
                normalize_velocity(swing_state["velocity_rad_s"], actuators["swing"]),
            ]
        )

        # 9..14：bucket tip 在 machine_root 下的位置和由相邻帧差分得到的速度。
        position_normalizer = float(normalizers["position_normalizer"])
        tip_velocity_scale = float(normalizers["tip_velocity_scale"])
        tip = tuple(float(value) for value in bucket_tip.position_m)
        obs.extend([value / max(position_normalizer, 0.01) for value in tip])
        obs.extend(self._tip_velocity_observation(bucket_tip, tip_velocity_scale))

        # 15..26：规划器输出的 waypoint 相对误差、progress、tube、isFinal 切片。
        if len(waypoint_values) != 12:
            raise ValueError("waypoint_values 必须是长度12数组，对应 observation idx 15..26")
        obs.extend(float(value) for value in waypoint_values)

        # 27..29：任务模式 one-hot 和 episode progress。真机第一版 episode_progress 可保持 0。
        obs.append(1.0 if self.task_mode == "MoveToDig" else 0.0)
        obs.append(1.0 if self.task_mode == "CarryMaterial" else 0.0)
        obs.append(clamp(episode_progress, 0.0, 1.0))

        # 30..33：上一帧策略动作，顺序必须是 boom, stick, bucket, swing。
        if len(previous_action) != len(ACTION_ORDER):
            raise ValueError("previous_action 必须是长度4数组")
        obs.extend(clamp(value) for value in previous_action)

        # 34..37：bucket pitch。当前先使用 FK topic 给出的 pitch_rad，后续再统一到 Unity 有符号定义。
        pitch_deg = math.degrees(float(bucket_tip.pitch_rad))
        target_pitch_deg = float(self.machine_profile["task_profile"]["bucket_pitch_targets_deg"][self.task_mode])
        pitch_norm_deg = float(normalizers["pitch_norm_deg"])
        obs.append(pitch_deg / max(pitch_norm_deg, 1.0))
        obs.append(target_pitch_deg / max(pitch_norm_deg, 1.0))
        obs.append((pitch_deg - target_pitch_deg) / max(pitch_norm_deg, 1.0))
        obs.append(self._pitch_velocity_observation(bucket_tip, pitch_deg))

        self.previous_tip = bucket_tip
        if len(obs) != 38:
            raise AssertionError(f"observation维度错误: {len(obs)}")
        return obs

    def _tip_velocity_observation(self, bucket_tip: BucketTipObservation, velocity_scale: float) -> list[float]:
        """用上一帧 bucket tip 差分估计 tip velocity observation。"""
        previous = self.previous_tip
        if previous is None:
            return [0.0, 0.0, 0.0]
        dt = self._dt_s(previous, bucket_tip)
        if dt <= 1e-6:
            return [0.0, 0.0, 0.0]
        return [
            (float(bucket_tip.position_m[i]) - float(previous.position_m[i])) / dt / max(velocity_scale, 0.001)
            for i in range(3)
        ]

    def _pitch_velocity_observation(self, bucket_tip: BucketTipObservation, pitch_deg: float) -> float:
        """用上一帧 pitch 差分估计 idx 37，单位按 deg/s/180。"""
        previous = self.previous_tip
        if previous is None:
            return 0.0
        dt = self._dt_s(previous, bucket_tip)
        if dt <= 1e-6:
            return 0.0
        previous_pitch_deg = math.degrees(float(previous.pitch_rad))
        return clamp(delta_angle_deg(previous_pitch_deg, pitch_deg) / dt / 180.0)

    @staticmethod
    def _dt_s(previous: BucketTipObservation, current: BucketTipObservation) -> float:
        """优先用 ROS/FK 时间戳计算 dt，缺失时返回 0。"""
        if previous.stamp_ms is None or current.stamp_ms is None:
            return 0.0
        return max((int(current.stamp_ms) - int(previous.stamp_ms)) / 1000.0, 0.0)
