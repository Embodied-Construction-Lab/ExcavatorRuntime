"""固定挖掘/倾倒动作执行器。

先由外部 planner/策略把 bucket tip 送到目标点，再用固定的归一化执行器目标
完成挖掘或倾倒。这里不做路径规划，只根据 Orin
回传的 actuator_state 追踪固定动作段。
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Sequence

from runtime_bridge.observation import normalize_position, normalized_command_deadzones
from runtime_bridge.protocol import ACTION_ORDER, MachineStatePacket, PolicyActionPacket, make_zero_action, now_ms


FIXED_ACTION_PROFILE_SCHEMA = "fixed_action_profile.v2"
DEFAULT_WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
_ACTION_CONTRACT_FIELDS = (
    "profile_id",
    "machine_id",
    "action_order",
    "machine_profile_sha256",
    "urdf_sha256",
    "controller",
    "start_envelopes",
    "actions",
)


class FixedActionProfileError(ValueError):
    """版本化固定动作模板不满足机器或几何契约。"""


@dataclass(frozen=True)
class FixedActionStep:
    """固定动作中的一段绝对归一化执行器目标。None 表示本段不控制该轴。"""

    step_id: str
    label: str
    target_normalized_qpos: tuple[
        float | None,
        float | None,
        float | None,
        float | None,
    ]


@dataclass(frozen=True)
class FixedActionController:
    kp: float
    min_action: float
    max_action: float
    tolerance: float
    step_timeout_s: float
    hold_s: float


@dataclass(frozen=True)
class FixedActionStartEnvelope:
    normalized_actuator_position: Mapping[str, tuple[float, float]]
    swing_rad: tuple[float, float]


@dataclass(frozen=True)
class FixedActionStartDecision:
    allowed: bool
    reason: str


@dataclass(frozen=True)
class FixedActionProfile:
    """与机型配置和 URDF 摘要绑定的不可变 Dig/Dump 模板。"""

    profile_id: str
    machine_id: str
    action_order: tuple[str, str, str, str]
    validation_status: str
    machine_profile_sha256: str
    urdf_sha256: str
    controller: FixedActionController
    start_envelopes: Mapping[str, FixedActionStartEnvelope]
    actions: Mapping[str, tuple[FixedActionStep, ...]]
    sha256: str

    def sequence(self, name: str) -> tuple[FixedActionStep, ...]:
        try:
            return self.actions[name]
        except KeyError as exc:
            raise FixedActionProfileError(f"固定动作模板不存在: {name}") from exc

    def evaluate_start(
        self,
        name: str,
        state: MachineStatePacket,
        machine_profile: Mapping[str, Any],
    ) -> FixedActionStartDecision:
        try:
            envelope = self.start_envelopes[name]
        except KeyError as exc:
            raise FixedActionProfileError(f"固定动作起始包络不存在: {name}") from exc
        current = current_normalized_joint_pose(state, dict(machine_profile))
        for index, actuator in enumerate(("boom", "stick", "bucket")):
            lower, upper = envelope.normalized_actuator_position[actuator]
            if current[index] < lower - 1e-9 or current[index] > upper + 1e-9:
                return FixedActionStartDecision(
                    False, f"{name}_{actuator}_outside_start_envelope"
                )
        swing_rad = float(state.actuator_state["swing"]["position_rad"])
        if not math.isfinite(swing_rad):
            return FixedActionStartDecision(False, f"{name}_swing_not_finite")
        if _position_range_is_bounded(machine_profile, "swing") and not _inside(
            swing_rad,
            envelope.swing_rad,
        ):
            return FixedActionStartDecision(False, f"{name}_swing_outside_start_envelope")
        return FixedActionStartDecision(True, "fixed_action_start_valid")


@dataclass(frozen=True)
class FixedActionStatus:
    """固定动作执行状态，供命令行打印和后续 planner 编排使用。"""

    phase: str
    step_index: int
    step_label: str
    max_error: float
    done: bool
    failed: bool = False
    reason_code: str = ""


def load_fixed_action_profile(
    path: Path,
    *,
    machine_profile_path: Path,
    urdf_path: Path,
    expected_sha256: str,
    workspace_root: Path = DEFAULT_WORKSPACE_ROOT,
) -> FixedActionProfile:
    """严格加载动作模板，并验证它仍绑定当前机型配置和 URDF。"""
    try:
        raw = Path(path).read_bytes()
        root = json.loads(raw)
    except (OSError, json.JSONDecodeError) as exc:
        raise FixedActionProfileError(f"无法读取固定动作模板 {path}: {exc}") from exc
    _require_fields(
        "root",
        root,
        {
            "schema_version",
            "profile_id",
            "machine_id",
            "action_order",
            "validation_status",
            "validation_evidence",
            "machine_profile_sha256",
            "urdf_sha256",
            "controller",
            "start_envelopes",
            "actions",
        },
    )
    if root["schema_version"] != FIXED_ACTION_PROFILE_SCHEMA:
        raise FixedActionProfileError(
            f"schema_version 必须是 {FIXED_ACTION_PROFILE_SCHEMA}"
        )
    profile_id = _non_empty_string("profile_id", root["profile_id"])
    machine_id = _non_empty_string("machine_id", root["machine_id"])
    if root["validation_status"] not in {"candidate", "field_validated"}:
        raise FixedActionProfileError(
            "validation_status 必须是 candidate 或 field_validated"
        )
    action_contract_sha256 = fixed_action_contract_sha256(root)
    validated_max_action = _validate_evidence(
        root["validation_status"],
        root["validation_evidence"],
        workspace_root=workspace_root,
        profile_id=profile_id,
        action_contract_sha256=action_contract_sha256,
    )
    if not isinstance(root["action_order"], list) or tuple(root["action_order"]) != ACTION_ORDER:
        raise FixedActionProfileError(
            f"action_order 必须是 {list(ACTION_ORDER)}"
        )
    try:
        machine_profile_raw = Path(machine_profile_path).read_bytes()
        machine_profile = json.loads(machine_profile_raw)
        urdf_raw = Path(urdf_path).read_bytes()
    except (OSError, json.JSONDecodeError) as exc:
        raise FixedActionProfileError(f"无法读取动作模板绑定制品: {exc}") from exc
    if not isinstance(machine_profile, dict) or machine_profile.get("machine_id") != machine_id:
        raise FixedActionProfileError("machine_id 与 machine_profile 不一致")
    if tuple(machine_profile.get("action_order", ())) != ACTION_ORDER:
        raise FixedActionProfileError("machine_profile.action_order 与协议不一致")
    _require_digest(
        "machine_profile_sha256",
        root["machine_profile_sha256"],
        machine_profile_raw,
    )
    _require_digest("urdf_sha256", root["urdf_sha256"], urdf_raw)
    actual_profile_sha256 = hashlib.sha256(raw).hexdigest()
    if expected_sha256 != actual_profile_sha256:
        raise FixedActionProfileError("expected_sha256 与固定动作模板文件不一致")
    controller = _parse_controller(root["controller"])
    if validated_max_action is not None and controller.max_action > validated_max_action:
        raise FixedActionProfileError(
            "controller.max_action 超过现场证据验证的最大动作"
        )
    actions = root["actions"]
    _require_fields("actions", actions, {"dig", "dump"})
    start_envelopes = _parse_start_envelopes(root["start_envelopes"])
    parsed_actions = {
        name: _parse_action_steps(name, actions[name]) for name in ("dig", "dump")
    }
    return FixedActionProfile(
        profile_id=profile_id,
        machine_id=machine_id,
        action_order=ACTION_ORDER,
        validation_status=root["validation_status"],
        machine_profile_sha256=root["machine_profile_sha256"],
        urdf_sha256=root["urdf_sha256"],
        controller=controller,
        start_envelopes=MappingProxyType(start_envelopes),
        actions=MappingProxyType(parsed_actions),
        sha256=actual_profile_sha256,
    )


def fixed_action_contract_sha256(profile: Mapping[str, Any]) -> str:
    """Hash every field whose exact value must have been exercised in the field."""
    missing = [name for name in _ACTION_CONTRACT_FIELDS if name not in profile]
    if missing:
        raise FixedActionProfileError(
            "动作契约缺少字段: " + ", ".join(sorted(missing))
        )
    contract = {name: profile[name] for name in _ACTION_CONTRACT_FIELDS}
    try:
        encoded = json.dumps(
            contract,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise FixedActionProfileError(f"动作契约无法规范化: {exc}") from exc
    return hashlib.sha256(encoded).hexdigest()


def _position_range_is_bounded(
    machine_profile: Mapping[str, Any],
    actuator: str,
) -> bool:
    actuator_profile = machine_profile.get("actuators", {}).get(actuator, {})
    required = actuator_profile.get("range_enabled_required")
    if isinstance(required, bool):
        return required
    configured_range = actuator_profile.get("range")
    return (
        isinstance(configured_range, (list, tuple))
        and len(configured_range) == 2
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in configured_range
        )
    )


def _require_fields(name: str, value: object, expected: set[str]) -> None:
    if not isinstance(value, dict):
        raise FixedActionProfileError(f"{name} 必须是 JSON object")
    missing = expected - set(value)
    unknown = set(value) - expected
    if missing:
        raise FixedActionProfileError(f"{name} 缺少字段: {', '.join(sorted(missing))}")
    if unknown:
        raise FixedActionProfileError(f"{name} 包含未知字段: {', '.join(sorted(unknown))}")


def _non_empty_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise FixedActionProfileError(f"{name} 必须是非空字符串")
    return value


def _require_digest(name: str, configured: object, payload: bytes) -> None:
    actual = hashlib.sha256(payload).hexdigest()
    if configured != actual:
        raise FixedActionProfileError(f"{name} 与当前制品不一致")


def _parse_action_steps(name: str, value: object) -> tuple[FixedActionStep, ...]:
    if not isinstance(value, list) or not value:
        raise FixedActionProfileError(f"actions.{name} 必须是非空数组")
    parsed: list[FixedActionStep] = []
    step_ids: set[str] = set()
    for index, item in enumerate(value):
        prefix = f"actions.{name}[{index}]"
        _require_fields(
            prefix,
            item,
            {"step_id", "label", "target_normalized_position"},
        )
        step_id = _non_empty_string(f"{prefix}.step_id", item["step_id"])
        if step_id in step_ids:
            raise FixedActionProfileError(f"actions.{name} 存在重复 step_id: {step_id}")
        step_ids.add(step_id)
        label = _non_empty_string(f"{prefix}.label", item["label"])
        targets = item["target_normalized_position"]
        if not isinstance(targets, Mapping):
            raise FixedActionProfileError(
                f"{prefix}.target_normalized_position 必须是 JSON object"
            )
        if not targets or not set(targets).issubset(set(ACTION_ORDER)):
            raise FixedActionProfileError(
                f"{prefix}.target_normalized_position 包含无效执行器"
            )
        if "swing" in targets:
            raise FixedActionProfileError(
                f"{prefix}.target_normalized_position 暂不支持 swing"
            )
        converted = tuple(
            (
                _finite_number(
                    f"{prefix}.target_normalized_position.{actuator}",
                    targets[actuator],
                )
                if actuator in targets
                else None
            )
            for actuator in ACTION_ORDER
        )
        if any(component is not None and abs(component) > 1.0 for component in converted):
            raise FixedActionProfileError(
                f"{prefix}.target_normalized_position 必须在 [-1,1] 内"
            )
        parsed.append(FixedActionStep(step_id, label, converted))
    return tuple(parsed)


def _parse_controller(value: object) -> FixedActionController:
    fields = {
        "kp",
        "min_action",
        "max_action",
        "tolerance",
        "step_timeout_s",
        "hold_s",
    }
    _require_fields("controller", value, fields)
    converted = {
        name: _finite_number(f"controller.{name}", value[name]) for name in fields
    }
    if not 0.0 < converted["kp"] <= 100.0:
        raise FixedActionProfileError("controller.kp 超出范围")
    if not 0.0 <= converted["min_action"] <= converted["max_action"] <= 1.0:
        raise FixedActionProfileError("controller action 范围无效")
    if not 0.0 < converted["tolerance"] <= 1.0:
        raise FixedActionProfileError("controller.tolerance 超出范围")
    if not 0.0 < converted["step_timeout_s"] <= 60.0:
        raise FixedActionProfileError("controller.step_timeout_s 超出范围")
    if not 0.0 <= converted["hold_s"] <= 10.0:
        raise FixedActionProfileError("controller.hold_s 超出范围")
    return FixedActionController(**converted)


def _parse_start_envelopes(value: object) -> dict[str, FixedActionStartEnvelope]:
    _require_fields("start_envelopes", value, {"dig", "dump"})
    parsed: dict[str, FixedActionStartEnvelope] = {}
    for phase in ("dig", "dump"):
        entry = value[phase]
        _require_fields(
            f"start_envelopes.{phase}",
            entry,
            {"normalized_actuator_position", "swing_rad"},
        )
        normalized = entry["normalized_actuator_position"]
        _require_fields(
            f"start_envelopes.{phase}.normalized_actuator_position",
            normalized,
            {"boom", "stick", "bucket"},
        )
        ranges = {
            name: _number_range(
                f"start_envelopes.{phase}.normalized_actuator_position.{name}",
                normalized[name],
                minimum=-1.0,
                maximum=1.0,
            )
            for name in ("boom", "stick", "bucket")
        }
        parsed[phase] = FixedActionStartEnvelope(
            normalized_actuator_position=MappingProxyType(ranges),
            swing_rad=_number_range(
                f"start_envelopes.{phase}.swing_rad",
                entry["swing_rad"],
                minimum=-math.tau,
                maximum=math.tau,
            ),
        )
    return parsed


def _number_range(
    name: str, value: object, *, minimum: float, maximum: float
) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise FixedActionProfileError(f"{name} 必须是两个数的数组")
    lower, upper = (_finite_number(name, item) for item in value)
    if lower < minimum or upper > maximum or lower > upper:
        raise FixedActionProfileError(f"{name} 范围无效")
    return lower, upper


def _inside(value: float, bounds: tuple[float, float]) -> bool:
    return bounds[0] - 1e-9 <= value <= bounds[1] + 1e-9


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise FixedActionProfileError(f"{name} 必须是有限数")
    return float(value)


def _validate_evidence(
    status: str,
    value: object,
    *,
    workspace_root: Path,
    profile_id: str,
    action_contract_sha256: str,
) -> float | None:
    if status == "candidate":
        if value is not None:
            raise FixedActionProfileError("candidate 的 validation_evidence 必须为 null")
        return None
    fields = {
        "validated_at",
        "validated_by",
        "evaluation_report",
        "evaluation_report_sha256",
        "experiment_run_ids",
        "validated_phases",
        "max_validated_normalized_command",
        "action_contract_sha256",
    }
    _require_fields("validation_evidence", value, fields)
    for name in ("validated_at", "validated_by", "evaluation_report"):
        _non_empty_string(f"validation_evidence.{name}", value[name])
    digest = value["evaluation_report_sha256"]
    if not isinstance(digest, str) or len(digest) != 64 or any(
        character not in "0123456789abcdef" for character in digest
    ):
        raise FixedActionProfileError("validation_evidence.evaluation_report_sha256 无效")
    run_ids = value["experiment_run_ids"]
    if not isinstance(run_ids, list) or not run_ids or any(
        not isinstance(item, str) or not item.strip() for item in run_ids
    ):
        raise FixedActionProfileError("validation_evidence.experiment_run_ids 无效")
    if value["validated_phases"] != ["dig", "dump"]:
        raise FixedActionProfileError("validation_evidence 必须覆盖 dig 和 dump")
    if value["action_contract_sha256"] != action_contract_sha256:
        raise FixedActionProfileError("validation_evidence 未绑定当前固定动作契约")
    maximum = _finite_number(
        "validation_evidence.max_validated_normalized_command",
        value["max_validated_normalized_command"],
    )
    if not 0.0 < maximum <= 1.0:
        raise FixedActionProfileError("max_validated_normalized_command 超出范围")
    configured_report = Path(value["evaluation_report"])
    workspace = Path(workspace_root).resolve()
    if (
        configured_report.is_absolute()
        or not configured_report.parts
        or configured_report.parts[0] != "EvaluationReport"
        or configured_report.suffix.lower() != ".md"
    ):
        raise FixedActionProfileError("evaluation_report 必须是 EvaluationReport 下的 Markdown")
    report_path = (workspace / configured_report).resolve()
    if not report_path.is_relative_to(workspace / "EvaluationReport") or not report_path.is_file():
        raise FixedActionProfileError("evaluation_report 不存在或越出 EvaluationReport")
    report_bytes = report_path.read_bytes()
    if hashlib.sha256(report_bytes).hexdigest() != digest:
        raise FixedActionProfileError("evaluation_report_sha256 与报告文件不一致")
    report_text = report_bytes.decode("utf-8", errors="strict")
    required_lines = (
        f"fixed_action_profile_id: {profile_id}",
        f"fixed_action_contract_sha256: {action_contract_sha256}",
        *(f"experiment_run_id: {run_id}" for run_id in run_ids),
    )
    if any(line not in report_text.splitlines() for line in required_lines):
        raise FixedActionProfileError("evaluation_report 未绑定当前动作契约或实验 Run ID")
    return maximum


def physical_velocity_action_from_normalized(action: Sequence[float], machine_profile: dict[str, Any]) -> list[float]:
    """把训练语义的 [-1,1] 动作转换为发送给 Orin 的物理速度。

    输出仍放在 policy_action.action 字段中，action_type 保持 normalized_velocity_command 以兼容 Orin。
    PC 只选择方向对应的速度幅值，不改变 ONNX 动作符号；真机低层方向适配由 STM32 负责。
    """
    if len(action) != len(ACTION_ORDER):
        raise ValueError("action 必须是长度4数组")
    actuators = machine_profile["actuators"]
    result: list[float] = []
    for name, value in zip(ACTION_ORDER, action, strict=True):
        normalized = clamp(float(value), -1.0, 1.0)
        actuator = actuators[name]
        positive_deadzone, negative_deadzone = normalized_command_deadzones(
            actuator, actuator_name=name
        )
        deadzone = positive_deadzone if normalized >= 0.0 else negative_deadzone
        if abs(normalized) <= deadzone:
            result.append(0.0)
            continue
        speed_limit = (
            float(actuator["max_speed_positive"])
            if normalized >= 0.0
            else float(actuator["max_speed_negative"])
        )
        result.append(normalized * speed_limit)
    return result


class FixedActionExecutor:
    """用 Orin 状态闭环追踪固定动作序列。"""

    def __init__(
        self,
        steps: Sequence[FixedActionStep],
        machine_profile: dict[str, Any],
        *,
        kp: float = 1.5,
        min_action: float = 0.08,
        max_action: float = 1.0,
        tolerance: float = 0.03,
        step_timeout_s: float = 3.0,
        hold_s: float = 0.15,
    ) -> None:
        if not steps:
            raise ValueError("固定动作序列不能为空")
        self.steps = list(steps)
        self.machine_profile = machine_profile
        self.kp = max(float(kp), 0.01)
        self.min_action = max(float(min_action), 0.0)
        self.max_action = max(float(max_action), 0.01)
        self.tolerance = max(float(tolerance), 0.001)
        self.step_timeout_s = max(float(step_timeout_s), 0.1)
        self.hold_s = max(float(hold_s), 0.0)
        self.step_index = 0
        self.step_started_at_s: float | None = None
        self.hold_until_s: float | None = None
        self.failed_reason = ""
        self.latched_axes = (False, False, False, False)

    @property
    def done(self) -> bool:
        """是否已经完成全部固定动作段。"""
        return self.step_index >= len(self.steps)

    def step(self, state: MachineStatePacket, *, now_s: float, seq: int, valid_for_ms: int) -> tuple[PolicyActionPacket, FixedActionStatus]:
        """根据当前状态推进一步，返回应发送给 Orin 的动作包。"""
        if self.failed_reason:
            return make_zero_action(seq, valid_for_ms), self._status(
                "failed", 0.0, False, failed=True, reason_code=self.failed_reason
            )
        if self.done:
            return make_zero_action(seq, valid_for_ms), self._status("done", 0.0, True)

        if self.hold_until_s is not None:
            if now_s < self.hold_until_s:
                return make_zero_action(seq, valid_for_ms), self._status("hold", 0.0, False)
            self._advance_step()
            if self.done:
                return make_zero_action(seq, valid_for_ms), self._status("done", 0.0, True)

        current_qpos = current_normalized_joint_pose(state, self.machine_profile)
        if self.step_started_at_s is None:
            self.step_started_at_s = now_s

        step = self.steps[self.step_index]
        raw_error = tuple(
            (
                0.0
                if step.target_normalized_qpos[index] is None
                else float(step.target_normalized_qpos[index]) - current_qpos[index]
            )
            for index in range(4)
        )
        timed_out = (now_s - self.step_started_at_s) >= self.step_timeout_s
        if timed_out:
            self.failed_reason = "STEP_TIMEOUT"
            return make_zero_action(seq, valid_for_ms), self._status(
                "failed",
                max(abs(value) for value in raw_error),
                False,
                failed=True,
                reason_code=self.failed_reason,
            )
        self.latched_axes = tuple(
            was_latched
            or (
                step.target_normalized_qpos[index] is not None
                and abs(raw_error[index]) <= self.tolerance
            )
            for index, was_latched in enumerate(self.latched_axes)
        )
        error = tuple(
            0.0 if self.latched_axes[index] else raw_error[index]
            for index in range(4)
        )
        max_error = max(abs(value) for value in error)
        if max_error <= self.tolerance:
            self.hold_until_s = now_s + self.hold_s
            return make_zero_action(seq, valid_for_ms), self._status("hold", max_error, False)

        normalized_action = tuple(self._servo_axis(value) for value in error)
        physical_action = physical_velocity_action_from_normalized(
            normalized_action, self.machine_profile
        )
        packet = PolicyActionPacket(
            seq=seq,
            stamp_ms=now_ms(),
            action=physical_action,
            action_type="normalized_velocity_command",
            valid_for_ms=valid_for_ms,
        )
        return packet, self._status("running", max_error, False)

    def _advance_step(self) -> None:
        """进入下一段固定动作。"""
        self.step_index += 1
        self.step_started_at_s = None
        self.hold_until_s = None
        self.latched_axes = (False, False, False, False)

    def _servo_axis(self, error: float) -> float:
        """把归一化位置误差转换为归一化速度动作。"""
        if abs(error) <= self.tolerance:
            return 0.0
        max_action = clamp(self.max_action, 0.01, 1.0)
        min_action = clamp(self.min_action, 0.0, max_action)
        raw_action = self.kp * error
        magnitude = clamp(abs(raw_action), min_action, max_action)
        return (1.0 if error >= 0.0 else -1.0) * magnitude

    def _status(
        self,
        phase: str,
        max_error: float,
        done: bool,
        *,
        failed: bool = False,
        reason_code: str = "",
    ) -> FixedActionStatus:
        """构造当前执行状态。"""
        if self.done:
            return FixedActionStatus(
                phase,
                self.step_index,
                "完成",
                max_error,
                done,
                failed,
                reason_code,
            )
        return FixedActionStatus(
            phase,
            self.step_index,
            self.steps[self.step_index].label,
            max_error,
            done,
            failed,
            reason_code,
        )


def current_normalized_joint_pose(state: MachineStatePacket, machine_profile: dict[str, Any]) -> tuple[float, float, float, float]:
    """从 Orin actuator_state 计算 boom/stick/bucket/swing 的归一化姿态。"""
    actuators = machine_profile["actuators"]
    boom = normalize_position(state.actuator_state["boom"]["position_m"], actuators["boom"])
    stick = normalize_position(state.actuator_state["stick"]["position_m"], actuators["stick"])
    bucket = normalize_position(state.actuator_state["bucket"]["position_m"], actuators["bucket"])
    # 当前固定挖掘/倾倒序列不使用 swing 增量；这里保留0作为后续回转固定动作扩展点。
    swing = 0.0
    return (boom, stick, bucket, swing)


def clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    """限制数值范围。"""
    return max(lower, min(upper, float(value)))
