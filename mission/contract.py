"""版本化、不可变的文件式挖掘 Mission 契约。"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping


class MissionContractError(ValueError):
    """Mission 文件不满足可审计契约。"""


@dataclass(frozen=True)
class MissionTarget:
    position_m: tuple[float, float, float]
    normal: tuple[float, float, float]
    radius_m: float


@dataclass(frozen=True)
class MissionLimits:
    waypoint_tolerance_m: float
    waypoint_dwell_s: float
    tracking_timeout_s: float
    settle_s: float


@dataclass(frozen=True)
class ExcavationMission:
    mission_id: str
    mission_type: str
    frame_id: str
    target_status: str
    targets: Mapping[str, MissionTarget]
    limits: MissionLimits
    sha256: str


def _fields(name: str, value: object, expected: set[str]) -> dict:
    if not isinstance(value, dict):
        raise MissionContractError(f"{name} 必须是JSON object")
    missing = expected - set(value)
    if missing:
        raise MissionContractError(f"{name} 缺少字段: {', '.join(sorted(missing))}")
    unknown = set(value) - expected
    if unknown:
        raise MissionContractError(f"{name} 包含未知字段: {', '.join(sorted(unknown))}")
    return value


def _number(
    name: str,
    value: object,
    *,
    positive: bool = False,
    nonnegative: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise MissionContractError(f"{name} 必须是有限数值")
    converted = float(value)
    if positive and converted <= 0.0:
        raise MissionContractError(f"{name} 必须大于0")
    if nonnegative and converted < 0.0:
        raise MissionContractError(f"{name} 必须大于等于0")
    return converted


def _triplet(name: str, values: object) -> tuple[float, float, float]:
    if not isinstance(values, list) or len(values) != 3:
        raise MissionContractError(f"{name} 必须是长度3数组")
    return tuple(_number(f"{name}[{index}]", value) for index, value in enumerate(values))


def load_mission(path: Path) -> ExcavationMission:
    """加载一次 Mission Snapshot；活动期间不再隐式重读文件。"""
    try:
        raw = Path(path).read_bytes()
        data = json.loads(raw)
    except (OSError, json.JSONDecodeError) as exc:
        raise MissionContractError(f"无法读取Mission: {path}: {exc}") from exc
    root = _fields(
        "root",
        data,
        {
            "schema_version",
            "mission_id",
            "mission_type",
            "frame_id",
            "target_status",
            "targets",
            "limits",
        },
    )
    if root["schema_version"] != "excavation_mission.v1":
        raise MissionContractError("schema_version必须是excavation_mission.v1")
    if not isinstance(root["mission_id"], str) or not root["mission_id"].strip():
        raise MissionContractError("mission_id必须是非空字符串")
    if root["mission_type"] != "dig_transport_dump":
        raise MissionContractError("mission_type必须是dig_transport_dump")
    if root["frame_id"] != "machine_root_ros":
        raise MissionContractError("frame_id必须是machine_root_ros")
    if root["target_status"] not in {"placeholder", "rviz_adjusted", "field_validated"}:
        raise MissionContractError("target_status无效")

    target_data = _fields("targets", root["targets"], {"dig", "dump"})
    targets = {
        name: _load_target(name, target_data[name])
        for name in ("dig", "dump")
    }
    limit_data = _fields(
        "limits",
        root["limits"],
        {"waypoint_tolerance_m", "waypoint_dwell_s", "tracking_timeout_s", "settle_s"},
    )
    limits = MissionLimits(
        waypoint_tolerance_m=_number(
            "limits.waypoint_tolerance_m",
            limit_data["waypoint_tolerance_m"],
            positive=True,
        ),
        waypoint_dwell_s=_number(
            "limits.waypoint_dwell_s",
            limit_data["waypoint_dwell_s"],
            nonnegative=True,
        ),
        tracking_timeout_s=_number(
            "limits.tracking_timeout_s",
            limit_data["tracking_timeout_s"],
            positive=True,
        ),
        settle_s=_number("limits.settle_s", limit_data["settle_s"], positive=True),
    )
    undersized_targets = [
        name
        for name, target in targets.items()
        if target.radius_m + 1e-9 < limits.waypoint_tolerance_m
    ]
    if undersized_targets:
        names = ", ".join(undersized_targets)
        raise MissionContractError(
            f"目标半径必须大于等于waypoint容差: {names}"
        )
    return ExcavationMission(
        mission_id=root["mission_id"],
        mission_type=root["mission_type"],
        frame_id=root["frame_id"],
        target_status=root["target_status"],
        targets=MappingProxyType(targets),
        limits=limits,
        sha256=hashlib.sha256(raw).hexdigest(),
    )


def _load_target(name: str, value: object) -> MissionTarget:
    target = _fields(
        f"targets.{name}",
        value,
        {"position_m", "normal", "radius_m"},
    )
    normal = _triplet(f"targets.{name}.normal", target["normal"])
    norm = math.sqrt(sum(component * component for component in normal))
    if not math.isclose(norm, 1.0, abs_tol=1e-6):
        raise MissionContractError(f"targets.{name}.normal必须是单位向量")
    return MissionTarget(
        position_m=_triplet(f"targets.{name}.position_m", target["position_m"]),
        normal=normal,
        radius_m=_number(f"targets.{name}.radius_m", target["radius_m"], positive=True),
    )
