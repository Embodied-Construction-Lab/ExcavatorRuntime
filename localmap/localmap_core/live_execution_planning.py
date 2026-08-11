"""Pure contracts for converting a live Plan goal into execution-strict artifacts."""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping
from typing import Any

from .planning_intent import PlanningIntent


_TASK_MODES = {"dig": "MoveToDig", "dump": "CarryMaterial"}


def _thaw_json(value: Any) -> Any:
    """Return a mutable JSON-shaped copy of recursively frozen live input."""
    if isinstance(value, Mapping):
        return {key: _thaw_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thaw_json(item) for item in value]
    return value


def validate_execution_workspace_provenance(
    workspace: Mapping[str, Any], urdf_bytes: bytes
) -> None:
    if workspace.get("coordinate_frame") != "machine_root_ros":
        raise ValueError("execution workspace frame must be machine_root_ros")
    source = workspace.get("source")
    if not isinstance(source, Mapping) or source.get("status") != "field_validated":
        raise ValueError("execution workspace must be field_validated")
    expected = hashlib.sha256(urdf_bytes).hexdigest()
    if source.get("urdf_sha256") != expected:
        raise ValueError("execution workspace does not match the current URDF")


def inject_live_target(
    local_map: Mapping[str, Any], target: Mapping[str, Any]
) -> tuple[dict[str, Any], PlanningIntent]:
    phase = target.get("target_kind")
    if phase not in _TASK_MODES or target.get("mission_phase") != phase:
        raise ValueError("target kind and mission phase mismatch")
    if local_map.get("frame_id") != "machine_root_ros":
        raise ValueError("local map frame must be machine_root_ros")
    snapshot = _thaw_json(local_map)
    for name in ("dig_targets", "dump_targets"):
        if not isinstance(snapshot.get(name), list):
            raise ValueError(f"local map {name} must be an array")
    target_id = _string(target, "target_id")
    mission_id = _string(target, "mission_id")
    mission_sha256 = _sha256(target.get("mission_sha256"))
    position = _triplet(target.get("position_m"), "position_m")
    normal = _triplet(target.get("normal"), "normal")
    if not math.isclose(math.sqrt(sum(value * value for value in normal)), 1.0, abs_tol=1e-6):
        raise ValueError("target normal must be a unit vector")
    radius = float(target.get("radius_m", 0.0))
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("target radius must be positive")
    collection = "dig_targets" if phase == "dig" else "dump_targets"
    if any(item.get("id") == target_id for item in snapshot[collection]):
        raise ValueError("target ID already exists in live local map")
    snapshot[collection] = [
        *snapshot[collection],
        {
            "id": target_id,
            "position_m": list(position),
            "normal": list(normal),
            "radius_m": radius,
            "confidence": 1.0,
            "mission": {"id": mission_id, "sha256": mission_sha256, "phase": phase},
        },
    ]
    return snapshot, PlanningIntent(target_id, phase, _TASK_MODES[phase])


def build_execution_snapshot_fields(
    trajectory: Mapping[str, Any],
    target: Mapping[str, Any],
    *,
    source_bucket_tip_stamp_s: float,
    source_local_map_stamp_s: float,
    inputs_frozen_at_s: float,
    created_at_s: float,
    waypoint_tolerance_m: float,
    waypoint_dwell_s: float,
    tracking_timeout_s: float,
    control_stage: str,
) -> dict[str, Any]:
    if trajectory.get("frame_id") != "machine_root_ros":
        raise ValueError("trajectory frame must be machine_root_ros")
    if trajectory.get("planning_scope") != "execution_strict" or not trajectory.get(
        "execution_eligible"
    ):
        raise ValueError("trajectory must be execution_strict and execution eligible")
    planner = trajectory.get("planner")
    if not isinstance(planner, Mapping):
        raise ValueError("trajectory workspace constraint provenance is missing")
    workspace_constraint = planner.get("workspace_constraint")
    if workspace_constraint == "disabled_by_operator":
        if (
            planner.get("reachable_workspace") is not None
            or planner.get("workspace_disable_reason")
            != "operator_temporary_workspace_invalid"
        ):
            raise ValueError("trajectory workspace constraint provenance is invalid")
    elif workspace_constraint == "field_validated":
        if not isinstance(planner.get("reachable_workspace"), Mapping):
            raise ValueError("trajectory workspace constraint provenance is invalid")
    else:
        raise ValueError("trajectory workspace constraint provenance is invalid")
    if control_stage != "commissioning":
        raise ValueError("control_stage must be commissioning")
    if workspace_constraint not in {
        "disabled_by_operator", "field_validated"
    }:
        raise ValueError("commissioning trajectory workspace constraint is invalid")
    phase = target.get("target_kind")
    if phase not in _TASK_MODES or trajectory.get("task_mode") != _TASK_MODES[phase]:
        raise ValueError("trajectory task mode does not match target")
    mission = trajectory.get("mission")
    if not isinstance(mission, Mapping) or (
        mission.get("id") != target.get("mission_id")
        or mission.get("sha256") != target.get("mission_sha256")
        or mission.get("phase") != target.get("mission_phase")
    ):
        raise ValueError("trajectory mission provenance mismatch")
    waypoints = trajectory.get("waypoints_base")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("trajectory waypoints are missing")
    converted = [_triplet(point, "waypoint") for point in waypoints]
    target_position = _triplet(target.get("position_m"), "target.position_m")
    radius = float(target.get("radius_m", 0.0))
    if math.dist(converted[-1], target_position) > radius:
        raise ValueError("trajectory endpoint is outside target radius")
    frozen_at = float(inputs_frozen_at_s)
    created = float(created_at_s)
    if (
        not math.isfinite(frozen_at)
        or frozen_at <= 0.0
        or not math.isfinite(created)
        or created < frozen_at
    ):
        raise ValueError("planning snapshot timestamps are invalid")
    for name, stamp in (
        ("bucket tip", source_bucket_tip_stamp_s),
        ("local map", source_local_map_stamp_s),
    ):
        if not math.isfinite(stamp) or stamp <= 0.0 or stamp > frozen_at:
            raise ValueError(f"{name} source timestamp is invalid for frozen planning inputs")
    return {
        "frame_id": "machine_root_ros",
        "created_at_s": created,
        "mission_id": _string(target, "mission_id"),
        "mission_sha256": _sha256(target.get("mission_sha256")),
        "mission_phase": phase,
        "task_mode": _TASK_MODES[phase],
        "planning_scope": "execution_strict",
        "control_stage": control_stage,
        "workspace_constraint": workspace_constraint,
        "execution_eligible": True,
        "source_bucket_tip_stamp_s": float(source_bucket_tip_stamp_s),
        "source_local_map_stamp_s": float(source_local_map_stamp_s),
        "inputs_frozen_at_s": frozen_at,
        "valid_until_s": created + 10.0,
        "input_source": "live",
        "map_source": "live_local_map",
        "clock_mode": "ros_clock",
        "waypoints": converted,
        "waypoint_tolerance_m": float(waypoint_tolerance_m),
        "waypoint_dwell_s": float(waypoint_dwell_s),
        "tracking_timeout_s": float(tracking_timeout_s),
    }


def _string(values: Mapping[str, Any], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be non-empty")
    return value


def _sha256(value: object) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ValueError("mission_sha256 must be lowercase sha256")
    return value


def _triplet(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    converted = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in converted):
        raise ValueError(f"{name} must contain finite values")
    return converted
