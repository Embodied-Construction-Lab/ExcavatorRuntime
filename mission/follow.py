"""不可变 Follow Machine Behavior 核心；不包含 ROS 或动作发送实现。"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace
from typing import Sequence

try:
    from airy_excavator_interfaces.snapshot_digest import trajectory_snapshot_sha256
except ModuleNotFoundError as exc:  # source-tree tests before colcon install
    if exc.name != "airy_excavator_interfaces":
        raise
    from ros_interfaces.airy_excavator_interfaces.airy_excavator_interfaces.snapshot_digest import (
        trajectory_snapshot_sha256,
    )

from mission.trajectory_tracker import TrajectoryTracker


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_PHASE_TASK_MODE = {"dig": "MoveToDig", "dump": "CarryMaterial"}
_SHADOW_SCOPES = {"preview_global", "workspace_strict"}
_EXECUTION_SCOPE = "execution_strict"
_PLANNING_SCOPES = {*_SHADOW_SCOPES, _EXECUTION_SCOPE}
_INPUT_SOURCES = {"fixture", "replay", "live"}
_MAP_SOURCES = {
    "fixture": {"fixture_empty", "fixture_none"},
    "replay": {"replay_local_map", "replay_none"},
    "live": {"live_local_map", "live_none"},
}
_CLOCK_MODES = {"ros_clock"}
_MAX_SOURCE_AGE_S = 2.0


class TrajectoryDigestMismatch(ValueError):
    """Trajectory content does not match its immutable identity digest."""


@dataclass(frozen=True)
class FollowTrajectorySnapshot:
    frame_id: str
    created_at_s: float
    trajectory_id: str
    trajectory_sha256: str
    mission_id: str
    mission_sha256: str
    mission_phase: str
    task_mode: str
    planning_scope: str
    control_stage: str
    workspace_constraint: str
    execution_eligible: bool
    source_bucket_tip_stamp_s: float
    source_local_map_stamp_s: float
    inputs_frozen_at_s: float
    valid_until_s: float
    input_source: str
    map_source: str
    clock_mode: str
    waypoints: tuple[tuple[float, float, float], ...]
    waypoint_tolerance_m: float
    waypoint_dwell_s: float
    tracking_timeout_s: float

    def __post_init__(self) -> None:
        if self.frame_id != "machine_root_ros":
            raise ValueError("frame_id must be machine_root_ros")
        for name in ("trajectory_id", "mission_id", "map_source", "clock_mode"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"{name} must be non-empty")
        for name in ("trajectory_sha256", "mission_sha256"):
            if not isinstance(getattr(self, name), str) or not _SHA256.fullmatch(getattr(self, name)):
                raise ValueError(f"{name} must be lowercase sha256")
        expected_task_mode = _PHASE_TASK_MODE.get(self.mission_phase)
        if expected_task_mode is None or self.task_mode != expected_task_mode:
            raise ValueError("mission_phase and task_mode mismatch")
        if self.planning_scope not in _PLANNING_SCOPES:
            raise ValueError("planning_scope is invalid")
        if self.control_stage not in {"none", "commissioning", "production"}:
            raise ValueError("control_stage is invalid")
        if self.workspace_constraint not in {
            "none", "disabled_by_operator", "field_validated"
        }:
            raise ValueError("workspace_constraint is invalid")
        if self.input_source not in _INPUT_SOURCES:
            raise ValueError("input_source is invalid")
        if self.map_source not in _MAP_SOURCES[self.input_source]:
            raise ValueError("map_source does not match input_source")
        if self.clock_mode not in _CLOCK_MODES:
            raise ValueError("clock_mode is invalid")
        for name in (
            "created_at_s",
            "source_bucket_tip_stamp_s",
            "source_local_map_stamp_s",
            "inputs_frozen_at_s",
            "valid_until_s",
        ):
            _finite(name, getattr(self, name))
        if self.inputs_frozen_at_s > self.created_at_s + 1e-6:
            raise ValueError("inputs_frozen_at_s must not be after created_at_s")
        if self.valid_until_s <= self.created_at_s:
            raise ValueError("valid_until_s must be after created_at_s")
        for name in ("source_bucket_tip_stamp_s", "source_local_map_stamp_s"):
            source_stamp = getattr(self, name)
            if source_stamp <= 0.0 or source_stamp > self.inputs_frozen_at_s + 1e-6:
                raise ValueError(f"{name} is inconsistent with inputs_frozen_at_s")
            if self.inputs_frozen_at_s - source_stamp > _MAX_SOURCE_AGE_S:
                raise ValueError(f"{name} is stale when planning inputs were frozen")
        if not self.waypoints:
            raise ValueError("waypoints must not be empty")
        for point in self.waypoints:
            _triplet("waypoint", point)
        for name in ("waypoint_tolerance_m", "tracking_timeout_s"):
            value = _finite(name, getattr(self, name))
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if _finite("waypoint_dwell_s", self.waypoint_dwell_s) < 0.0:
            raise ValueError("waypoint_dwell_s must be nonnegative")

    def validate_for_shadow(self, *, expected_input_source: str, now_s: float) -> None:
        current = _finite("now_s", now_s)
        if self.execution_eligible:
            raise ValueError("execution_eligible trajectories are rejected by shadow Follow")
        if self.input_source != expected_input_source:
            raise ValueError("input_source does not match runtime")
        if current > self.valid_until_s:
            raise ValueError("trajectory snapshot expired")
        if current + 1e-6 < self.created_at_s:
            raise ValueError("trajectory snapshot is from the future")
        if self.trajectory_sha256 != self.computed_sha256():
            raise TrajectoryDigestMismatch(
                "trajectory_sha256 does not match Trajectory Snapshot content"
            )

    def validate_for_execution(
        self, *, now_s: float, expected_control_stage: str
    ) -> None:
        """Validate a live trajectory before a motion-capable Follow accepts it."""
        current = _finite("now_s", now_s)
        if not self.execution_eligible:
            raise ValueError("execution_eligible must be true for motion Follow")
        if self.planning_scope != _EXECUTION_SCOPE:
            raise ValueError("planning_scope must be execution_strict for motion Follow")
        if self.input_source != "live" or self.map_source != "live_local_map":
            raise ValueError("input_source/map_source must identify live inputs")
        if self.control_stage != expected_control_stage:
            raise ValueError("trajectory control_stage does not match runtime")
        if expected_control_stage == "production":
            workspace_valid = self.workspace_constraint == "field_validated"
        elif expected_control_stage == "commissioning":
            workspace_valid = self.workspace_constraint in {
                "disabled_by_operator", "field_validated"
            }
        else:
            raise ValueError("expected_control_stage is invalid")
        if not workspace_valid:
            raise ValueError("trajectory workspace_constraint does not match control_stage")
        if current > self.valid_until_s:
            raise ValueError("trajectory snapshot expired")
        if current + 1e-6 < self.created_at_s:
            raise ValueError("trajectory snapshot is from the future")
        if self.trajectory_sha256 != self.computed_sha256():
            raise TrajectoryDigestMismatch(
                "trajectory_sha256 does not match Trajectory Snapshot content"
            )

    def computed_sha256(self) -> str:
        return trajectory_snapshot_sha256(
            frame_id=self.frame_id,
            created_at_s=self.created_at_s,
            mission_id=self.mission_id,
            mission_sha256=self.mission_sha256,
            mission_phase=self.mission_phase,
            task_mode=self.task_mode,
            planning_scope=self.planning_scope,
            control_stage=self.control_stage,
            workspace_constraint=self.workspace_constraint,
            execution_eligible=self.execution_eligible,
            source_bucket_tip_stamp_s=self.source_bucket_tip_stamp_s,
            source_local_map_stamp_s=self.source_local_map_stamp_s,
            inputs_frozen_at_s=self.inputs_frozen_at_s,
            valid_until_s=self.valid_until_s,
            input_source=self.input_source,
            map_source=self.map_source,
            clock_mode=self.clock_mode,
            waypoints=self.waypoints,
            waypoint_tolerance_m=self.waypoint_tolerance_m,
            waypoint_dwell_s=self.waypoint_dwell_s,
            tracking_timeout_s=self.tracking_timeout_s,
        )


@dataclass(frozen=True)
class FollowUpdate:
    sample_accepted: bool
    current_waypoint_index: int
    waypoint_count: int
    distance_m: float
    elapsed_s: float
    advanced: bool
    completed: bool
    timed_out: bool


@dataclass(frozen=True)
class FollowSession:
    snapshot: FollowTrajectorySnapshot
    tracker: TrajectoryTracker
    accepted_at_s: float
    last_sample_stamp_s: float | None = None
    last_distance_m: float = math.inf

    @classmethod
    def start(
        cls,
        snapshot: FollowTrajectorySnapshot,
        *,
        accepted_at_s: float,
    ) -> "FollowSession":
        accepted = _finite("accepted_at_s", accepted_at_s)
        return cls(
            snapshot=snapshot,
            tracker=TrajectoryTracker(
                waypoints=snapshot.waypoints,
                tolerance_m=snapshot.waypoint_tolerance_m,
                dwell_s=snapshot.waypoint_dwell_s,
                timeout_s=snapshot.tracking_timeout_s,
            ),
            accepted_at_s=accepted,
        )

    def observe(
        self,
        bucket_tip_m: Sequence[float],
        *,
        sample_stamp_s: float,
        now_s: float,
    ) -> tuple["FollowSession", FollowUpdate]:
        stamp = _finite("sample_stamp_s", sample_stamp_s)
        current = _finite("now_s", now_s)
        if stamp <= self.accepted_at_s or (
            self.last_sample_stamp_s is not None and stamp <= self.last_sample_stamp_s
        ):
            return self, self._update(False, current, False, self.tracker.completed, False)

        tracker, tracker_update = self.tracker.advance(bucket_tip_m, now_s=current)
        updated = replace(
            self,
            tracker=tracker,
            last_sample_stamp_s=stamp,
            last_distance_m=tracker_update.distance_m,
        )
        return updated, updated._update(
            True,
            current,
            tracker_update.advanced,
            tracker_update.completed,
            tracker_update.timed_out,
        )

    def _update(
        self,
        sample_accepted: bool,
        now_s: float,
        advanced: bool,
        completed: bool,
        timed_out: bool,
    ) -> FollowUpdate:
        return FollowUpdate(
            sample_accepted=sample_accepted,
            current_waypoint_index=self.tracker.current_index,
            waypoint_count=len(self.tracker.waypoints),
            distance_m=self.last_distance_m,
            elapsed_s=max(0.0, now_s - self.accepted_at_s),
            advanced=advanced,
            completed=completed,
            timed_out=timed_out,
        )


def _finite(name: str, value: float) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _triplet(name: str, values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return tuple(_finite(f"{name}[{index}]", value) for index, value in enumerate(values))
