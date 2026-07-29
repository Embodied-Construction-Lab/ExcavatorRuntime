"""不可变的多挖掘点演示程序契约。"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

from mission.contract import (
    MissionContractError,
    MissionLimits,
    MissionTarget,
    _fields,
    _load_target,
    _number,
)


class DemoProgramError(MissionContractError):
    """演示程序文件不满足可审计契约。"""


@dataclass(frozen=True)
class DemoDigPoint:
    point_id: str
    target: MissionTarget


@dataclass(frozen=True)
class ExcavationDemoProgram:
    demo_id: str
    frame_id: str
    target_status: str
    dig_points: tuple[DemoDigPoint, ...]
    dump_target: MissionTarget
    limits: MissionLimits
    sha256: str


def load_demo_program(path: Path) -> ExcavationDemoProgram:
    """加载一次多点演示程序；运行期间不再隐式重读。"""
    try:
        raw = Path(path).read_bytes()
        data = json.loads(raw)
    except (OSError, json.JSONDecodeError) as exc:
        raise DemoProgramError(f"无法读取演示程序: {path}: {exc}") from exc

    try:
        root = _fields(
            "root",
            data,
            {
                "schema_version",
                "demo_id",
                "frame_id",
                "target_status",
                "dig_points",
                "dump_target",
                "limits",
            },
        )
        if root["schema_version"] != "excavation_demo.v1":
            raise DemoProgramError("schema_version必须是excavation_demo.v1")
        if not isinstance(root["demo_id"], str) or not root["demo_id"].strip():
            raise DemoProgramError("demo_id必须是非空字符串")
        if root["frame_id"] != "machine_root_ros":
            raise DemoProgramError("frame_id必须是machine_root_ros")
        if root["target_status"] not in {
            "placeholder",
            "rviz_adjusted",
            "field_validated",
        }:
            raise DemoProgramError("target_status无效")
        dig_data = root["dig_points"]
        if not isinstance(dig_data, list) or not dig_data:
            raise DemoProgramError("dig_points必须至少包含一个挖掘点")

        dig_points = tuple(
            _load_dig_point(index, value) for index, value in enumerate(dig_data)
        )
        point_ids = [point.point_id for point in dig_points]
        if len(set(point_ids)) != len(point_ids):
            raise DemoProgramError("dig_points.point_id必须唯一")

        dump_target = _load_target("dump_target", root["dump_target"])
        limits = _load_limits(root["limits"])
        all_targets = [point.target for point in dig_points] + [dump_target]
        if any(
            target.radius_m + 1e-9 < limits.waypoint_tolerance_m
            for target in all_targets
        ):
            raise DemoProgramError("目标半径必须大于等于waypoint容差")
    except MissionContractError as exc:
        if isinstance(exc, DemoProgramError):
            raise
        raise DemoProgramError(str(exc)) from exc

    return ExcavationDemoProgram(
        demo_id=root["demo_id"],
        frame_id=root["frame_id"],
        target_status=root["target_status"],
        dig_points=dig_points,
        dump_target=dump_target,
        limits=limits,
        sha256=hashlib.sha256(raw).hexdigest(),
    )


def _load_dig_point(index: int, value: object) -> DemoDigPoint:
    name = f"dig_points[{index}]"
    point = _fields(
        name,
        value,
        {"point_id", "position_m", "normal", "radius_m"},
    )
    point_id = point["point_id"]
    if not isinstance(point_id, str) or not point_id.strip():
        raise DemoProgramError(f"{name}.point_id必须是非空字符串")
    target = _load_target(
        name,
        {
            "position_m": point["position_m"],
            "normal": point["normal"],
            "radius_m": point["radius_m"],
        },
    )
    return DemoDigPoint(point_id=point_id, target=target)


def _load_limits(value: object) -> MissionLimits:
    limit_data = _fields(
        "limits",
        value,
        {
            "waypoint_tolerance_m",
            "waypoint_dwell_s",
            "tracking_timeout_s",
            "settle_s",
        },
    )
    return MissionLimits(
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
