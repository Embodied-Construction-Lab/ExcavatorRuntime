"""Pure PC-side motion gates and live trajectory observation helpers."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Mapping, Protocol, Sequence

import numpy as np

from localmap.localmap_core.trajectory import build_waypoint_observation_slice
from runtime_bridge.observation import position_observation_range
from runtime_bridge.protocol import (
    ACTION_ORDER,
    MachineStatePacket,
    PolicyActionPacket,
    encode_packet,
    make_zero_action,
)


LIVE_MOTION_AUTHORIZATION = "ALLOW_LIVE_MACHINE_MOTION"


class DatagramSender(Protocol):
    def send(self, payload: bytes) -> object: ...


@dataclass(frozen=True)
class MotionGateDecision:
    allowed: bool
    reason: str


@dataclass(frozen=True)
class ManualJogDecision:
    allowed: bool
    reason: str
    physical_action: tuple[float, float, float, float]
    position_m: float


@dataclass(frozen=True)
class FollowCanaryEnvelope:
    """Immutable Follow axis mask and full directional physical envelope."""

    allowed_actuators: tuple[str, ...]
    physical_minimum: tuple[float, float, float, float]
    physical_maximum: tuple[float, float, float, float]

    @classmethod
    def from_machine_profile(
        cls,
        machine_profile: Mapping[str, Any],
        *,
        allowed_actuators: Sequence[str],
    ) -> "FollowCanaryEnvelope":
        allowed = tuple(allowed_actuators)
        if (
            not allowed
            or len(set(allowed)) != len(allowed)
            or any(name not in set(ACTION_ORDER) for name in allowed)
        ):
            raise ValueError(
                "follow allowed_actuators must be unique action-contract names"
            )
        actuators = machine_profile.get("actuators")
        if not isinstance(actuators, Mapping):
            raise ValueError("machine_profile.actuators is missing")
        minimum: list[float] = []
        maximum: list[float] = []
        for name in ACTION_ORDER:
            if name not in allowed:
                minimum.append(0.0)
                maximum.append(0.0)
                continue
            actuator = actuators.get(name)
            if not isinstance(actuator, Mapping):
                raise ValueError(f"machine_profile actuator is missing: {name}")
            positive = _positive_finite(
                f"{name}.max_speed_positive", actuator.get("max_speed_positive")
            )
            negative = _positive_finite(
                f"{name}.max_speed_negative", actuator.get("max_speed_negative")
            )
            minimum.append(-negative)
            maximum.append(positive)
        return cls(
            allowed_actuators=allowed,
            physical_minimum=tuple(minimum),
            physical_maximum=tuple(maximum),
        )

    def apply_normalized(self, raw_action: Sequence[float]) -> tuple[float, float, float, float]:
        values = tuple(raw_action)
        if len(values) != len(ACTION_ORDER) or not all(
            isinstance(value, int | float)
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in values
        ):
            raise ValueError("follow policy output must contain four finite numbers")
        return tuple(
            float(value)
            if name in self.allowed_actuators
            else 0.0
            for name, value in zip(ACTION_ORDER, values, strict=True)
        )

    def evaluate_physical(self, physical_action: Sequence[float]) -> MotionGateDecision:
        values = tuple(float(value) for value in physical_action)
        if len(values) != len(ACTION_ORDER) or not all(math.isfinite(value) for value in values):
            raise ValueError("physical_action must contain four finite values")
        for index, (name, value) in enumerate(zip(ACTION_ORDER, values, strict=True)):
            if name not in self.allowed_actuators and value != 0.0:
                return MotionGateDecision(False, "follow_canary_axis_locked")
            if (
                value < self.physical_minimum[index] - 1e-12
                or value > self.physical_maximum[index] + 1e-12
            ):
                return MotionGateDecision(False, "follow_canary_envelope_violation")
        return MotionGateDecision(True, "follow_canary_envelope_valid")


def _positive_finite(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be positive and finite")
    return float(value)


def evaluate_follow_canary_supervision(
    *,
    expected_session: str,
    heartbeat_session: str,
    heartbeat_age_ms: float,
    heartbeat_timeout_ms: int,
) -> MotionGateDecision:
    """Fail closed when a supervised Follow loses its operator heartbeat."""
    if not expected_session:
        raise ValueError("expected follow canary session must be non-empty")
    heartbeat_fresh = (
        heartbeat_session == expected_session
        and math.isfinite(heartbeat_age_ms)
        and 0.0 <= heartbeat_age_ms <= heartbeat_timeout_ms
    )
    if not heartbeat_fresh:
        return MotionGateDecision(False, "supervision_heartbeat_timeout")
    return MotionGateDecision(True, "follow_canary_supervised")


def motion_authorization_granted(value: str) -> bool:
    """Require an exact, deliberate launch-time authorization token."""
    return value == LIVE_MOTION_AUTHORIZATION


def evaluate_motion_state(state: MachineStatePacket) -> MotionGateDecision:
    safety = state.safety
    if safety["estop"]:
        return MotionGateDecision(False, "estop")
    if not safety["stm32_alive"]:
        return MotionGateDecision(False, "stm32_not_alive")
    if not safety["sensor_valid"]:
        return MotionGateDecision(False, "sensor_invalid")
    if safety["fault_flags"]:
        return MotionGateDecision(False, "fault_flags")
    if not safety["control_enabled"]:
        return MotionGateDecision(False, "control_disabled")
    return MotionGateDecision(True, "motion")


def evaluate_actuator_state(
    state: MachineStatePacket,
    machine_profile: Mapping[str, Any],
    *,
    enforce_bounds: bool = True,
) -> MotionGateDecision:
    """Validate actuator positions, optionally enforcing configured observation bounds."""
    if not isinstance(enforce_bounds, bool):
        raise ValueError("enforce_bounds must be a boolean")
    actuators = machine_profile.get("actuators")
    if not isinstance(actuators, Mapping):
        raise ValueError("machine_profile.actuators is missing")
    for name in ("boom", "stick", "bucket"):
        actuator = actuators.get(name)
        if not isinstance(actuator, Mapping):
            raise ValueError(f"machine_profile actuator is missing: {name}")
        try:
            lower, upper = position_observation_range(actuator)
        except ValueError as exc:
            raise ValueError(f"machine_profile actuator range is invalid: {name}: {exc}") from exc
        value = state.actuator_state[name]["position_m"]
        if not all(
            isinstance(item, int | float) and not isinstance(item, bool) and math.isfinite(item)
            for item in (lower, upper, value)
        ):
            return MotionGateDecision(False, f"{name}_position_invalid")
        if enforce_bounds and (
            float(value) < float(lower) - 1e-9
            or float(value) > float(upper) + 1e-9
        ):
            return MotionGateDecision(False, f"{name}_position_out_of_range")
    swing = state.actuator_state["swing"]["position_rad"]
    if isinstance(swing, bool) or not isinstance(swing, int | float) or not math.isfinite(swing):
        return MotionGateDecision(False, "swing_position_invalid")
    return MotionGateDecision(
        True,
        "actuator_state_valid"
        if enforce_bounds
        else "actuator_position_bounds_not_enforced",
    )


def build_manual_jog_action(
    state: MachineStatePacket,
    machine_profile: Mapping[str, Any],
    *,
    actuator: str,
    direction: int,
    allowed_actuators: Sequence[str],
    speed_fraction: float,
) -> ManualJogDecision:
    """Build one bounded single-axis command while preserving the requested action sign."""
    allowed = tuple(allowed_actuators)
    if actuator not in allowed or actuator not in {"boom", "stick", "bucket"}:
        raise ValueError(f"manual jog actuator is not an allowed actuator: {actuator!r}")
    if isinstance(direction, bool) or direction not in (-1, 1):
        raise ValueError("manual jog direction must be -1 or +1")
    if not math.isfinite(speed_fraction) or not 0.0 < speed_fraction <= 0.2:
        raise ValueError("manual jog speed_fraction must be in (0, 0.2]")

    actuators = machine_profile.get("actuators")
    profile = actuators.get(actuator) if isinstance(actuators, Mapping) else None
    if not isinstance(profile, Mapping):
        raise ValueError(f"machine_profile actuator is missing: {actuator}")
    deploy = profile.get("deploy_position_observation")
    if (
        not isinstance(deploy, Mapping)
        or deploy.get("source") != "stm32_absolute_cable_encoder"
        or deploy.get("status") not in {"firmware_safety_bounds", "field_calibrated"}
    ):
        raise ValueError(
            "manual jog requires firmware_safety_bounds or field_calibrated absolute encoder range"
        )
    position_observation_range(profile)
    position = float(state.actuator_state[actuator]["position_m"])
    if not math.isfinite(position):
        return ManualJogDecision(False, f"{actuator}_position_invalid", (0.0,) * 4, position)

    action_index = profile.get("action_index")
    if isinstance(action_index, bool) or not isinstance(action_index, int) or not 0 <= action_index < 4:
        raise ValueError(f"manual jog action_index is invalid for {actuator}")
    limit_name = "max_speed_positive" if direction > 0 else "max_speed_negative"
    speed_limit = profile.get(limit_name)
    if (
        isinstance(speed_limit, bool)
        or not isinstance(speed_limit, int | float)
        or not math.isfinite(speed_limit)
        or speed_limit <= 0.0
    ):
        raise ValueError(f"manual jog {limit_name} is invalid for {actuator}")
    values = [0.0, 0.0, 0.0, 0.0]
    values[action_index] = direction * float(speed_limit) * speed_fraction
    return ManualJogDecision(
        True,
        "manual_jog_allowed",
        tuple(values),
        position,
    )


def evaluate_state_provenance(
    state: MachineStatePacket,
    *,
    expected_machine_id: str,
    last_seq: int | None,
    last_stamp_ms: int | None = None,
    received_pc_ms: int | None = None,
    expected_clock_offset_ms: int | None = None,
    clock_offset_tolerance_ms: int = 250,
) -> MotionGateDecision:
    if state.source != "orin":
        return MotionGateDecision(False, "state_source_mismatch")
    if state.machine_id != expected_machine_id:
        return MotionGateDecision(False, "machine_id_mismatch")
    if state.stamp_ms <= 0:
        return MotionGateDecision(False, "state_stamp_invalid")
    if last_seq is not None and state.seq <= last_seq:
        return MotionGateDecision(False, "state_sequence_not_increasing")
    if last_stamp_ms is not None and state.stamp_ms <= last_stamp_ms:
        return MotionGateDecision(False, "state_stamp_not_increasing")
    if expected_clock_offset_ms is not None:
        if received_pc_ms is None:
            raise ValueError("received_pc_ms is required with expected_clock_offset_ms")
        observed_offset = int(state.stamp_ms) - int(received_pc_ms)
        if abs(observed_offset - int(expected_clock_offset_ms)) > clock_offset_tolerance_ms:
            return MotionGateDecision(False, "state_clock_offset_jump")
    return MotionGateDecision(True, "state_provenance_valid")


class MotionCommandSink:
    """The sole state-gated encoder/sender for PC live motion datagrams."""

    def __init__(
        self,
        sender: DatagramSender,
        *,
        valid_for_ms: int,
        max_state_age_s: float = 0.2,
        physical_action_limits: Sequence[float] | None = None,
    ) -> None:
        if valid_for_ms <= 0:
            raise ValueError("valid_for_ms must be positive")
        if not math.isfinite(max_state_age_s) or max_state_age_s <= 0.0:
            raise ValueError("max_state_age_s must be positive and finite")
        if physical_action_limits is None:
            limits = None
        else:
            limits = tuple(float(value) for value in physical_action_limits)
            if len(limits) != len(ACTION_ORDER) or not all(
                math.isfinite(value) and value > 0.0 for value in limits
            ):
                raise ValueError("physical_action_limits must contain four positive finite values")
        self._sender = sender
        self._valid_for_ms = int(valid_for_ms)
        self._max_state_age_s = float(max_state_age_s)
        self._physical_action_limits = limits
        self._action_seq = 0
        self._last_state_seq: int | None = None
        self._action_datagrams = 0
        self._send_lock = threading.Lock()
        self._disarmed = False

    @property
    def action_datagrams(self) -> int:
        return self._action_datagrams

    def send_velocity(
        self,
        state: MachineStatePacket,
        physical_action: Sequence[float],
        *,
        action_stamp_ms: int,
        state_age_s: float = 0.0,
        physical_envelope: FollowCanaryEnvelope | None = None,
    ) -> MotionGateDecision:
        values = tuple(float(value) for value in physical_action)
        if len(values) != len(ACTION_ORDER) or not all(math.isfinite(value) for value in values):
            raise ValueError("physical_action must contain four finite values")
        if not math.isfinite(state_age_s) or state_age_s < 0.0:
            raise ValueError("state_age_s must be finite and non-negative")
        if self._physical_action_limits is not None and any(
            abs(value) > limit + 1e-12
            for value, limit in zip(values, self._physical_action_limits, strict=True)
        ):
            raise ValueError("physical_action exceeds configured limit")

        with self._send_lock:
            if self._disarmed:
                decision = MotionGateDecision(False, "command_sink_disarmed")
            elif self._last_state_seq is not None and state.seq < self._last_state_seq:
                decision = MotionGateDecision(False, "state_out_of_order")
            elif state_age_s > self._max_state_age_s:
                decision = MotionGateDecision(False, "state_stale")
            else:
                decision = evaluate_motion_state(state)
            if decision.allowed and physical_envelope is not None:
                decision = physical_envelope.evaluate_physical(values)
            self._last_state_seq = (
                state.seq
                if self._last_state_seq is None
                else max(state.seq, self._last_state_seq)
            )

            packet = (
                PolicyActionPacket(
                    seq=self._action_seq,
                    stamp_ms=int(action_stamp_ms),
                    action=list(values),
                    action_type="normalized_velocity_command",
                    valid_for_ms=self._valid_for_ms,
                    action_order=ACTION_ORDER,
                )
                if decision.allowed
                else make_zero_action(
                    self._action_seq,
                    self._valid_for_ms,
                    stamp_ms=int(action_stamp_ms),
                )
            )
            self._send_packet_locked(packet)
            return decision

    def send_zero(self, *, action_stamp_ms: int) -> None:
        with self._send_lock:
            self._send_zero_locked(action_stamp_ms=int(action_stamp_ms))

    def disarm(self, *, action_stamp_ms: int) -> None:
        """Atomically prevent every future non-zero send and emit a terminal zero."""
        with self._send_lock:
            self._disarmed = True
            self._send_zero_locked(action_stamp_ms=int(action_stamp_ms))

    def _send_zero_locked(self, *, action_stamp_ms: int) -> None:
        packet = make_zero_action(
            self._action_seq,
            self._valid_for_ms,
            stamp_ms=action_stamp_ms,
        )
        self._send_packet_locked(packet)

    def _send_packet_locked(self, packet: PolicyActionPacket) -> None:
        self._sender.send(encode_packet(packet))
        self._action_seq += 1
        self._action_datagrams += 1


def build_dynamic_waypoint_values(
    trajectory: Mapping[str, Any],
    machine_profile: dict[str, Any],
    *,
    bucket_tip_ros: Sequence[float],
    current_index: int,
) -> list[float]:
    """Recompute observation indices 15..26 from the current ROS tip and index."""
    tip = np.asarray(tuple(float(value) for value in bucket_tip_ros), dtype=np.float64)
    if tip.shape != (3,) or not np.isfinite(tip).all():
        raise ValueError("bucket_tip_ros must contain three finite values")
    values = build_waypoint_observation_slice(
        trajectory_command=dict(trajectory),
        machine_profile=machine_profile,
        bucket_tip_base=tip,
        current_waypoint_index=int(current_index),
    )["values"]
    return [float(value) for value in values]
