"""与 ROS 消息无关的 Mission 目标可视化描述。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from mission.contract import ExcavationMission
from mission.dig_point_catalog import DigPointCatalog


MARKER_STYLE_SCHEMA = "mission_marker_style.v1"


class MissionMarkerStyleError(ValueError):
    """Mission marker style configuration is invalid."""


@dataclass(frozen=True)
class MissionMarkerStyle:
    target_diameter_m: float


@dataclass(frozen=True)
class MissionMarkerSpec:
    phase: str
    frame_id: str
    position_m: tuple[float, float, float]
    diameter_m: float
    color_rgba: tuple[float, float, float, float]
    label: str


def load_mission_marker_style(path: Path) -> MissionMarkerStyle:
    """Load the display-only style without changing Mission control semantics."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise MissionMarkerStyleError(f"invalid marker style JSON: {exc}") from exc
    expected = {"schema_version", "target_diameter_m"}
    if not isinstance(data, dict) or set(data) != expected:
        raise MissionMarkerStyleError(
            "marker style must contain exactly schema_version and target_diameter_m"
        )
    if data["schema_version"] != MARKER_STYLE_SCHEMA:
        raise MissionMarkerStyleError(
            f"marker style schema_version must be {MARKER_STYLE_SCHEMA}"
        )
    diameter = data["target_diameter_m"]
    if (
        isinstance(diameter, bool)
        or not isinstance(diameter, int | float)
        or not math.isfinite(diameter)
        or not 0.001 <= diameter <= 2.0
    ):
        raise MissionMarkerStyleError(
            "target_diameter_m must be finite and within 0.001..2.0 m"
        )
    return MissionMarkerStyle(target_diameter_m=float(diameter))


def build_mission_marker_specs(
    mission: ExcavationMission,
    style: MissionMarkerStyle,
) -> tuple[MissionMarkerSpec, ...]:
    """从同一个不可变 Mission Snapshot 生成 dig/dump 标记。"""
    colors = {
        "dig": (1.0, 0.45, 0.0, 0.85),
        "dump": (0.65, 0.15, 1.0, 0.85),
    }
    status = mission.target_status.upper()
    return tuple(
        MissionMarkerSpec(
            phase=phase,
            frame_id=mission.frame_id,
            position_m=mission.targets[phase].position_m,
            diameter_m=style.target_diameter_m,
            color_rgba=colors[phase],
            label=f"{phase.upper()} [{status}] {mission.mission_id}",
        )
        for phase in ("dig", "dump")
    )


def build_catalog_marker_specs(
    catalog: DigPointCatalog,
    mission: ExcavationMission,
    style: MissionMarkerStyle,
) -> tuple[MissionMarkerSpec, ...]:
    """Display the authoritative fixed dig catalog and the common dump target."""
    near_ids = frozenset(catalog.groups.get("near", ()))
    far_ids = frozenset(catalog.groups.get("far", ()))
    dig_specs = tuple(
        MissionMarkerSpec(
            phase=f"dig:{point_id}",
            frame_id=catalog.frame_id,
            position_m=position,
            diameter_m=style.target_diameter_m,
            color_rgba=(
                (1.0, 0.45, 0.0, 0.9)
                if point_id in near_ids
                else (0.0, 0.65, 1.0, 0.9)
                if point_id in far_ids
                else (0.7, 0.7, 0.7, 0.9)
            ),
            label=f"DIG {point_id}",
        )
        for point_id, position in catalog.points.items()
    )
    dump_spec = MissionMarkerSpec(
        phase="dump",
        frame_id=mission.frame_id,
        position_m=mission.targets["dump"].position_m,
        diameter_m=style.target_diameter_m,
        color_rgba=(0.65, 0.15, 1.0, 0.85),
        label=f"DUMP [{mission.target_status.upper()}] {mission.mission_id}",
    )
    return (*dig_specs, dump_spec)
