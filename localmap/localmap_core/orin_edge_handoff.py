"""Validate a file-based Trajectory Snapshot before handing it to Orin."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from mission.contract import ExcavationMission


_TASK_MODE_BY_PHASE = {"dig": "MoveToDig", "dump": "CarryMaterial"}


def build_orin_edge_handoff(
    trajectory_path: Path,
    *,
    mission: ExcavationMission,
    phase: str,
    source_bucket_tip: Mapping[str, Any],
    created_at_s: float,
) -> dict[str, Any]:
    """Return an auditable manifest only for a current executable trajectory."""
    if phase not in _TASK_MODE_BY_PHASE:
        raise ValueError("phase must be dig or dump")
    raw = Path(trajectory_path).read_bytes()
    try:
        trajectory = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"trajectory is not valid JSON: {exc}") from exc
    if not isinstance(trajectory, Mapping):
        raise ValueError("trajectory must be a JSON object")

    _validate_trajectory_contract(trajectory, mission=mission, phase=phase)
    tip_stamp_s, tip_position = _source_bucket_tip(source_bucket_tip)
    created = _finite("created_at_s", created_at_s)
    if created < tip_stamp_s:
        raise ValueError("handoff creation precedes the frozen Bucket Tip")

    waypoints = tuple(
        _point(f"waypoints_base[{index}]", point)
        for index, point in enumerate(trajectory["waypoints_base"])
    )
    start_distance = math.dist(tip_position, waypoints[0])
    if start_distance > mission.limits.waypoint_tolerance_m:
        raise ValueError(
            "trajectory first waypoint is outside the frozen Bucket Tip tolerance: "
            f"distance_m={start_distance:.6f}, "
            f"limit_m={mission.limits.waypoint_tolerance_m:.6f}"
        )
    endpoint_distance = math.dist(
        waypoints[-1],
        mission.targets[phase].position_m,
    )
    if endpoint_distance > mission.targets[phase].radius_m:
        raise ValueError("trajectory endpoint is outside the Mission target radius")

    return {
        "schema_version": "orin_edge_trajectory_handoff.v1",
        "created_at_s": created,
        "mission": {
            "id": mission.mission_id,
            "sha256": mission.sha256,
            "phase": phase,
            "target_status": mission.target_status,
        },
        "source_bucket_tip": {
            "stamp_s": tip_stamp_s,
            "frame_id": "machine_root_ros",
            "position_m": list(tip_position),
        },
        "trajectory": {
            "file_name": Path(trajectory_path).name,
            "sha256": hashlib.sha256(raw).hexdigest(),
            "frame_id": "machine_root_ros",
            "phase": phase,
            "task_mode": _TASK_MODE_BY_PHASE[phase],
            "waypoint_count": len(waypoints),
            "timestamp_s": _finite(
                "trajectory.timestamp_s", trajectory.get("timestamp_s")
            ),
        },
        "start_distance_m": start_distance,
        "waypoint_tolerance_m": mission.limits.waypoint_tolerance_m,
        "endpoint_distance_m": endpoint_distance,
        "target_radius_m": mission.targets[phase].radius_m,
    }


def write_orin_edge_handoff(
    destination: Path,
    manifest: Mapping[str, Any],
) -> None:
    """Atomically publish one validated handoff manifest."""
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        manifest,
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
    ) + "\n"
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f".{path.name}.",
            dir=path.parent,
            delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _validate_trajectory_contract(
    trajectory: Mapping[str, Any],
    *,
    mission: ExcavationMission,
    phase: str,
) -> None:
    if trajectory.get("schema_version") != "trajectory_command.v1":
        raise ValueError("trajectory schema_version must be trajectory_command.v1")
    if trajectory.get("frame_id") != "machine_root_ros":
        raise ValueError("trajectory frame must be machine_root_ros")
    if (
        trajectory.get("planning_scope") != "execution_strict"
        or trajectory.get("execution_eligible") is not True
    ):
        raise ValueError("trajectory must be execution_strict and execution eligible")
    if trajectory.get("task_mode") != _TASK_MODE_BY_PHASE[phase]:
        raise ValueError("trajectory task mode does not match Mission phase")
    waypoints = trajectory.get("waypoints_base")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("trajectory waypoints_base must be a non-empty array")
    if trajectory.get("waypoint_count") != len(waypoints):
        raise ValueError("trajectory waypoint_count does not match waypoints_base")

    mission_reference = trajectory.get("mission")
    expected_reference = {
        "id": mission.mission_id,
        "sha256": mission.sha256,
        "phase": phase,
    }
    if mission_reference != expected_reference:
        raise ValueError("trajectory Mission provenance does not match")

    planner = trajectory.get("planner")
    if not isinstance(planner, Mapping):
        raise ValueError("trajectory planner provenance is missing")
    workspace_constraint = planner.get("workspace_constraint")
    if workspace_constraint == "disabled_by_operator":
        if (
            planner.get("reachable_workspace") is not None
            or planner.get("workspace_disable_reason")
            != "operator_temporary_workspace_invalid"
        ):
            raise ValueError("disabled execution workspace provenance is invalid")
    elif workspace_constraint == "field_validated":
        if not isinstance(planner.get("reachable_workspace"), Mapping):
            raise ValueError("field-validated execution workspace provenance is invalid")
    else:
        raise ValueError("trajectory workspace constraint is invalid")


def _source_bucket_tip(
    value: Mapping[str, Any],
) -> tuple[float, tuple[float, float, float]]:
    if value.get("frame_id") != "machine_root_ros":
        raise ValueError("source Bucket Tip frame must be machine_root_ros")
    return (
        _finite("source_bucket_tip.stamp_s", value.get("stamp_s")),
        _point("source_bucket_tip.position_m", value.get("position_m")),
    )


def _point(name: str, value: object) -> tuple[float, float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 3
    ):
        raise ValueError(f"{name} must contain three values")
    return tuple(_finite(f"{name}[{index}]", item) for index, item in enumerate(value))


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted
