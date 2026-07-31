"""Receive-only PC Machine State bridge configuration."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_CONFIG = PROJECT_ROOT / "runtime_bridge" / "config" / "runtime.json"
RUNTIME_CONFIG_SCHEMA = "pc_state_bridge_config_v1"


class RuntimeConfigError(ValueError):
    """PC state bridge configuration is invalid."""


@dataclass(frozen=True)
class NetworkConfig:
    state_bind_host: str
    state_port: int

    @property
    def state_endpoint(self) -> tuple[str, int]:
        return self.state_bind_host, self.state_port


@dataclass(frozen=True)
class DiagnosticsConfig:
    print_every: int
    write_every: int


@dataclass(frozen=True)
class RuntimeConfig:
    network: NetworkConfig
    diagnostics: DiagnosticsConfig


def _validate_fields(section: str, data: object, expected: set[str]) -> dict:
    if not isinstance(data, dict):
        raise RuntimeConfigError(f"{section} 必须是 JSON object")
    missing = expected - set(data)
    if missing:
        raise RuntimeConfigError(f"{section} 缺少字段: {', '.join(sorted(missing))}")
    unknown = set(data) - expected
    if unknown:
        raise RuntimeConfigError(f"{section} 包含未知字段: {', '.join(sorted(unknown))}")
    return data


def _require_int_range(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise RuntimeConfigError(
            f"{name} 必须是 {minimum}..{maximum} 的整数，实际为 {value!r}"
        )
    return value


def load_runtime_config(path: Path = DEFAULT_RUNTIME_CONFIG) -> RuntimeConfig:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    root = _validate_fields("root", data, {"schema", "network", "diagnostics"})
    if root["schema"] != RUNTIME_CONFIG_SCHEMA:
        raise RuntimeConfigError(
            f"runtime config schema 必须是 {RUNTIME_CONFIG_SCHEMA}，"
            f"实际为 {root['schema']!r}"
        )
    network = _validate_fields(
        "network", root["network"], {"state_bind_host", "state_port"}
    )
    diagnostics = _validate_fields(
        "diagnostics", root["diagnostics"], {"print_every", "write_every"}
    )
    host = network["state_bind_host"]
    if not isinstance(host, str) or not host.strip():
        raise RuntimeConfigError("network.state_bind_host 必须是非空字符串")
    return RuntimeConfig(
        network=NetworkConfig(
            state_bind_host=host,
            state_port=_require_int_range(
                "network.state_port", network["state_port"], 1, 65535
            ),
        ),
        diagnostics=DiagnosticsConfig(
            print_every=_require_int_range(
                "diagnostics.print_every", diagnostics["print_every"], 0, 1_000_000
            ),
            write_every=_require_int_range(
                "diagnostics.write_every", diagnostics["write_every"], 0, 1_000_000
            ),
        ),
    )
