"""Resolve immutable Mission and demo target snapshots for live planning."""

from __future__ import annotations

from dataclasses import dataclass

from mission.contract import (
    ExcavationMission,
    MissionLimits,
    MissionTarget,
)
from mission.demo import ExcavationDemoProgram


@dataclass(frozen=True)
class ResolvedPlanningTarget:
    target: MissionTarget
    limits: MissionLimits
    target_status: str


def resolve_planning_target(
    message,
    *,
    mission: ExcavationMission,
    demo: ExcavationDemoProgram | None,
) -> ResolvedPlanningTarget:
    if (
        message.mission_id == mission.mission_id
        and message.mission_sha256 == mission.sha256
    ):
        try:
            target = mission.targets[message.target_kind]
        except KeyError as exc:
            raise ValueError("target kind不属于加载的Mission snapshot") from exc
        return ResolvedPlanningTarget(
            target=target,
            limits=mission.limits,
            target_status=mission.target_status,
        )

    if demo is None or (
        message.mission_id != demo.demo_id
        or message.mission_sha256 != demo.sha256
    ):
        raise ValueError("target does not match a loaded Mission snapshot")

    if message.target_kind == "dump":
        if message.target_id != f"{demo.demo_id}:dump":
            raise ValueError("dump target_id未声明在演示程序中")
        target = demo.dump_target
    elif message.target_kind == "dig":
        prefix = f"{demo.demo_id}:dig:"
        if not message.target_id.startswith(prefix):
            raise ValueError("dig target_id未声明在演示程序中")
        point_id = message.target_id[len(prefix) :]
        matching = [point.target for point in demo.dig_points if point.point_id == point_id]
        if len(matching) != 1:
            raise ValueError("dig target_id未声明在演示程序中")
        target = matching[0]
    else:
        raise ValueError("target kind未声明在演示程序中")

    return ResolvedPlanningTarget(
        target=target,
        limits=demo.limits,
        target_status=demo.target_status,
    )
