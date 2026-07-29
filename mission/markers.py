"""与 ROS 消息无关的 Mission 目标可视化描述。"""

from __future__ import annotations

from dataclasses import dataclass

from mission.contract import ExcavationMission
from mission.demo import ExcavationDemoProgram


@dataclass(frozen=True)
class MissionMarkerSpec:
    phase: str
    frame_id: str
    position_m: tuple[float, float, float]
    diameter_m: float
    color_rgba: tuple[float, float, float, float]
    label: str


def build_mission_marker_specs(mission: ExcavationMission) -> tuple[MissionMarkerSpec, ...]:
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
            diameter_m=mission.targets[phase].radius_m * 2.0,
            color_rgba=colors[phase],
            label=f"{phase.upper()} [{status}] {mission.mission_id}",
        )
        for phase in ("dig", "dump")
    )


def build_demo_marker_specs(
    program: ExcavationDemoProgram,
) -> tuple[MissionMarkerSpec, ...]:
    """显示全部有序挖掘点和公共倾倒点。"""
    status = program.target_status.upper()
    dig_specs = tuple(
        MissionMarkerSpec(
            phase=f"dig:{point.point_id}",
            frame_id=program.frame_id,
            position_m=point.target.position_m,
            diameter_m=point.target.radius_m * 2.0,
            color_rgba=(1.0, 0.45, 0.0, 0.85),
            label=f"DIG {point.point_id} [{status}] {program.demo_id}",
        )
        for point in program.dig_points
    )
    dump_spec = MissionMarkerSpec(
        phase="dump",
        frame_id=program.frame_id,
        position_m=program.dump_target.position_m,
        diameter_m=program.dump_target.radius_m * 2.0,
        color_rgba=(0.65, 0.15, 1.0, 0.85),
        label=f"DUMP [{status}] {program.demo_id}",
    )
    return (*dig_specs, dump_spec)
