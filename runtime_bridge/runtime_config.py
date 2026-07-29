"""PC-Orin 运行配置：用一个不可变对象隐藏网络、制品路径和运行常量。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "runtime_bridge" / "config" / "runtime.json"
RUNTIME_CONFIG_SCHEMA = "runtime_bridge_config_v10"


class RuntimeConfigError(ValueError):
    """运行配置格式或取值不满足部署契约。"""


@dataclass(frozen=True)
class NetworkConfig:
    state_bind_host: str
    state_port: int
    orin_host: str
    action_port: int
    action_valid_ms: int
    action_time_source: str

    @property
    def state_endpoint(self) -> tuple[str, int]:
        return self.state_bind_host, self.state_port

    @property
    def action_endpoint(self) -> tuple[str, int]:
        return self.orin_host, self.action_port


@dataclass(frozen=True)
class ArtifactConfig:
    onnx: Path
    machine_profile: Path
    fixed_action_profile: Path
    urdf: Path
    waypoint_slice: Path
    latest_observation: Path

    def require_policy_inputs(self) -> None:
        """策略入口启动前要求模型、机型配置和 waypoint 产物全部存在。"""
        self._require_file("onnx")
        self._require_file("machine_profile")
        self._require_file("waypoint_slice")

    def require_machine_profile(self) -> None:
        """固定动作入口只依赖机型配置，不要求策略制品。"""
        self._require_file("machine_profile")

    def require_fixed_action_inputs(self) -> None:
        """固定动作必须绑定模板、机型配置和当前 URDF。"""
        self._require_file("machine_profile")
        self._require_file("fixed_action_profile")
        self._require_file("urdf")

    def require_live_control_inputs(self) -> None:
        """Live control 启动前检查策略和固定动作的全部只读制品。"""
        self._require_file("onnx")
        self.require_fixed_action_inputs()

    def _require_file(self, name: str) -> None:
        path = getattr(self, name)
        if not path.is_file():
            raise RuntimeConfigError(f"artifacts.{name} 不存在或不是文件: {path}")


@dataclass(frozen=True)
class PolicyConfig:
    bucket_tip_timeout_ms: int
    machine_state_timeout_ms: int


@dataclass(frozen=True)
class FixedActionConfig:
    expected_profile_sha256: str


@dataclass(frozen=True)
class ManualJogConfig:
    enabled: bool
    allowed_actuators: tuple[str, ...]
    speed_fraction: float
    command_period_ms: int
    heartbeat_timeout_ms: int
    max_hold_ms: int


@dataclass(frozen=True)
class FollowControlConfig:
    mode: str
    allowed_actuators: tuple[str, ...]
    heartbeat_timeout_ms: int


@dataclass(frozen=True)
class DiagnosticsConfig:
    print_every: int
    write_every: int


@dataclass(frozen=True)
class ActionJournalConfig:
    directory: Path
    max_file_bytes: int
    retained_files: int


@dataclass(frozen=True)
class RuntimeConfig:
    network: NetworkConfig
    artifacts: ArtifactConfig
    policy: PolicyConfig
    fixed_action: FixedActionConfig
    manual_jog: ManualJogConfig
    follow_control: FollowControlConfig
    diagnostics: DiagnosticsConfig
    action_journal: ActionJournalConfig


def _validate_fields(section: str, data: object, expected: set[str]) -> None:
    if not isinstance(data, dict):
        raise RuntimeConfigError(f"{section} 必须是 JSON object")
    missing = expected - set(data)
    if missing:
        names = ", ".join(sorted(missing))
        raise RuntimeConfigError(f"{section} 缺少字段: {names}")
    unknown = set(data) - expected
    if unknown:
        names = ", ".join(sorted(unknown))
        raise RuntimeConfigError(f"{section} 包含未知字段: {names}")


def _require_int_range(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise RuntimeConfigError(f"{name} 必须是 {minimum}..{maximum} 的整数，实际为 {value!r}")
    return value


def _require_non_empty_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeConfigError(f"{name} 必须是非空字符串，实际为 {value!r}")
    return value


def _require_number_range(name: str, value: object, minimum: float, maximum: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(value)
        or not minimum <= value <= maximum
    ):
        raise RuntimeConfigError(f"{name} 必须在 {minimum}..{maximum} 范围内，实际为 {value!r}")
    return float(value)


def load_runtime_config(
    path: Path = DEFAULT_RUNTIME_CONFIG,
    *,
    project_root: Path = PROJECT_ROOT,
) -> RuntimeConfig:
    """加载运行配置；所有相对制品路径统一相对 AiryLidar 根目录解析。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeConfigError("root 必须是 JSON object")
    if data.get("schema") != RUNTIME_CONFIG_SCHEMA:
        raise RuntimeConfigError(
            f"runtime config schema 必须是 {RUNTIME_CONFIG_SCHEMA}，实际为 {data.get('schema')!r}"
        )
    _validate_fields(
        "root",
        data,
        {
            "schema",
            "network",
            "artifacts",
            "policy",
            "fixed_action",
            "manual_jog",
            "follow_control",
            "diagnostics",
            "action_journal",
        },
    )
    network = data["network"]
    artifacts = data["artifacts"]
    policy = data["policy"]
    fixed_action = data["fixed_action"]
    manual_jog = data["manual_jog"]
    follow_control = data["follow_control"]
    diagnostics = data["diagnostics"]
    action_journal = data["action_journal"]
    _validate_fields(
        "network",
        network,
        {
            "state_bind_host",
            "state_port",
            "orin_host",
            "action_port",
            "action_valid_ms",
            "action_time_source",
        },
    )
    _validate_fields(
        "artifacts",
        artifacts,
        {
            "onnx",
            "machine_profile",
            "fixed_action_profile",
            "urdf",
            "waypoint_slice",
            "latest_observation",
        },
    )
    _validate_fields(
        "policy", policy, {"bucket_tip_timeout_ms", "machine_state_timeout_ms"}
    )
    _validate_fields(
        "fixed_action",
        fixed_action,
        {
            "expected_profile_sha256",
        },
    )
    _validate_fields(
        "manual_jog",
        manual_jog,
        {
            "enabled",
            "allowed_actuators",
            "speed_fraction",
            "command_period_ms",
            "heartbeat_timeout_ms",
            "max_hold_ms",
        },
    )
    _validate_fields(
        "follow_control",
        follow_control,
        {
            "mode",
            "allowed_actuators",
            "heartbeat_timeout_ms",
        },
    )
    _validate_fields("diagnostics", diagnostics, {"print_every", "write_every"})
    _validate_fields(
        "action_journal",
        action_journal,
        {"directory", "max_file_bytes", "retained_files"},
    )
    _require_non_empty_string("network.state_bind_host", network.get("state_bind_host"))
    _require_non_empty_string("network.orin_host", network.get("orin_host"))
    _require_int_range("network.state_port", network.get("state_port"), 1, 65535)
    _require_int_range("network.action_port", network.get("action_port"), 1, 65535)
    _require_int_range("network.action_valid_ms", network.get("action_valid_ms"), 1, 60000)
    if network.get("action_time_source") not in {"orin", "pc"}:
        raise RuntimeConfigError(
            "network.action_time_source 必须是 'orin' 或 'pc'，"
            f"实际为 {network.get('action_time_source')!r}"
        )
    _require_int_range(
        "policy.bucket_tip_timeout_ms",
        policy.get("bucket_tip_timeout_ms"),
        1,
        60000,
    )
    _require_int_range(
        "policy.machine_state_timeout_ms",
        policy.get("machine_state_timeout_ms"),
        100,
        2000,
    )
    _require_int_range("diagnostics.print_every", diagnostics.get("print_every"), 0, 1000000)
    _require_int_range("diagnostics.write_every", diagnostics.get("write_every"), 0, 1000000)
    _require_int_range(
        "action_journal.max_file_bytes",
        action_journal.get("max_file_bytes"),
        1024,
        1073741824,
    )
    _require_int_range(
        "action_journal.retained_files",
        action_journal.get("retained_files"),
        1,
        1000,
    )
    expected_profile_sha256 = fixed_action.get("expected_profile_sha256")
    if (
        not isinstance(expected_profile_sha256, str)
        or len(expected_profile_sha256) != 64
        or any(character not in "0123456789abcdef" for character in expected_profile_sha256)
    ):
        raise RuntimeConfigError(
            "fixed_action.expected_profile_sha256 必须是64位小写SHA256"
        )
    if not isinstance(manual_jog.get("enabled"), bool):
        raise RuntimeConfigError("manual_jog.enabled 必须是布尔值")
    allowed_actuators = manual_jog.get("allowed_actuators")
    if (
        not isinstance(allowed_actuators, list)
        or not allowed_actuators
        or any(not isinstance(name, str) for name in allowed_actuators)
        or len(set(allowed_actuators)) != len(allowed_actuators)
        or any(name not in {"boom", "stick", "bucket"} for name in allowed_actuators)
    ):
        raise RuntimeConfigError(
            "manual_jog.allowed_actuators 必须是 boom/stick/bucket 的非空无重复数组"
        )
    speed_fraction = _require_number_range(
        "manual_jog.speed_fraction", manual_jog.get("speed_fraction"), 0.01, 0.2
    )
    command_period_ms = _require_int_range(
        "manual_jog.command_period_ms", manual_jog.get("command_period_ms"), 25, 100
    )
    heartbeat_timeout_ms = _require_int_range(
        "manual_jog.heartbeat_timeout_ms",
        manual_jog.get("heartbeat_timeout_ms"),
        75,
        500,
    )
    if heartbeat_timeout_ms < command_period_ms * 3:
        raise RuntimeConfigError(
            "manual_jog.heartbeat_timeout_ms 必须至少是 command_period_ms 的3倍"
        )
    max_hold_ms = _require_int_range(
        "manual_jog.max_hold_ms", manual_jog.get("max_hold_ms"), 250, 5000
    )
    if follow_control.get("mode") != "supervised_canary":
        raise RuntimeConfigError(
            "follow_control.mode 当前必须是 'supervised_canary'，"
            f"实际为 {follow_control.get('mode')!r}"
        )
    follow_allowed_actuators = follow_control.get("allowed_actuators")
    if (
        not isinstance(follow_allowed_actuators, list)
        or not follow_allowed_actuators
        or any(not isinstance(name, str) for name in follow_allowed_actuators)
        or len(set(follow_allowed_actuators)) != len(follow_allowed_actuators)
        or follow_allowed_actuators != ["boom", "stick", "bucket", "swing"]
    ):
        raise RuntimeConfigError(
            "follow_control.allowed_actuators 必须严格为 "
            "['boom', 'stick', 'bucket', 'swing']，不得改写 ONNX 动作轴"
        )
    follow_heartbeat_timeout_ms = _require_int_range(
        "follow_control.heartbeat_timeout_ms",
        follow_control.get("heartbeat_timeout_ms"),
        75,
        500,
    )
    def resolve(name: str, value: object) -> Path:
        if not isinstance(value, str) or not value.strip():
            raise RuntimeConfigError(f"{name} 必须是非空路径字符串")
        candidate = Path(value)
        return candidate if candidate.is_absolute() else project_root / candidate

    artifact_config = ArtifactConfig(
        onnx=resolve("artifacts.onnx", artifacts["onnx"]),
        machine_profile=resolve("artifacts.machine_profile", artifacts["machine_profile"]),
        fixed_action_profile=resolve(
            "artifacts.fixed_action_profile", artifacts["fixed_action_profile"]
        ),
        urdf=resolve("artifacts.urdf", artifacts["urdf"]),
        waypoint_slice=resolve("artifacts.waypoint_slice", artifacts["waypoint_slice"]),
        latest_observation=resolve("artifacts.latest_observation", artifacts["latest_observation"]),
    )

    return RuntimeConfig(
        network=NetworkConfig(**network),
        artifacts=artifact_config,
        policy=PolicyConfig(**policy),
        fixed_action=FixedActionConfig(**fixed_action),
        manual_jog=ManualJogConfig(
            enabled=manual_jog["enabled"],
            allowed_actuators=tuple(allowed_actuators),
            speed_fraction=speed_fraction,
            command_period_ms=command_period_ms,
            heartbeat_timeout_ms=heartbeat_timeout_ms,
            max_hold_ms=max_hold_ms,
        ),
        follow_control=FollowControlConfig(
            mode=follow_control["mode"],
            allowed_actuators=tuple(follow_allowed_actuators),
            heartbeat_timeout_ms=follow_heartbeat_timeout_ms,
        ),
        diagnostics=DiagnosticsConfig(**diagnostics),
        action_journal=ActionJournalConfig(
            directory=resolve("action_journal.directory", action_journal["directory"]),
            max_file_bytes=action_journal["max_file_bytes"],
            retained_files=action_journal["retained_files"],
        ),
    )
