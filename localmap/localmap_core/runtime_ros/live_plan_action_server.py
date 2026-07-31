#!/usr/bin/env python3
"""Live execution-strict Plan Action adapter for the unified PC operator."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
import threading
from pathlib import Path as FsPath


AIRY_ROOT = FsPath(__file__).resolve().parents[3]
sys.path.insert(0, str(AIRY_ROOT))
sys.path.insert(0, str(AIRY_ROOT / "localmap"))

import rclpy
from airy_excavator_interfaces.action import Plan
from airy_excavator_interfaces.msg import TrajectorySnapshot
from airy_excavator_interfaces.snapshot_digest import trajectory_snapshot_message_sha256
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from localmap.apps.planning.run_planning_once import (
    ROS_PYTHON,
    execute_prepared_run,
    invalidate_outputs,
    prepare_planning_snapshot,
)
from localmap_core.live_execution_planning import (
    build_execution_snapshot_fields,
    inject_live_target,
    validate_execution_workspace_provenance,
)
from localmap_core.mission_target_registry import resolve_planning_target
from localmap_core.planning_inputs import LivePlanningInputs, load_live_planning_inputs
from localmap_core.planning_profile import load_planning_profile
from mission.contract import load_mission
from mission.demo import load_demo_program
from runtime_bridge.control_stage import CONTROL_STAGES, control_stage_policy


class LivePlanActionNode(Node):
    def __init__(
        self,
        *,
        profile_path: FsPath,
        mission_path: FsPath,
        demo_path: FsPath | None = None,
        urdf_path: FsPath,
        control_stage: str,
        context=None,
    ) -> None:
        super().__init__("live_plan_server", context=context)
        self._profile = load_planning_profile(profile_path)
        self._mission = load_mission(mission_path)
        self._demo = load_demo_program(demo_path) if demo_path is not None else None
        self._control_policy = control_stage_policy(control_stage)
        self._allowed_target_statuses = self._control_policy.allowed_target_statuses
        workspace_mode = self._profile.planner.execution_workspace_mode
        if workspace_mode == "field_validated":
            validate_execution_workspace_provenance(
                json.loads(self._profile.inputs.reachable_workspace.read_text(encoding="utf-8")),
                urdf_path.read_bytes(),
            )
        else:
            self.get_logger().warning(
                "COMMISSIONING: reachable workspace is diagnostic only; "
                "planning remains global within configured planner bounds"
            )
        self._lock = threading.Lock()
        self._reserved = False
        self._callback_group = ReentrantCallbackGroup()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trajectory_publisher = self.create_publisher(
            TrajectorySnapshot, "/planning/trajectory_snapshot", latched
        )
        self._path_publisher = self.create_publisher(Path, "/planning/preview_path", latched)
        self._marker_publisher = self.create_publisher(
            MarkerArray, "/planning/preview_markers", latched
        )
        self._server = ActionServer(
            self,
            Plan,
            "/planning/plan",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "live Plan ready: scope=execution_strict input=live map=live_local_map "
            f"stage={self._control_policy.name} workspace={workspace_mode} datagrams=0"
        )

    def destroy_node(self):
        self._server.destroy()
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_goal(self, request: Plan.Goal) -> GoalResponse:
        try:
            self._validate_goal(request)
        except ValueError as exc:
            self.get_logger().warning(f"live Plan rejected: {exc}")
            return GoalResponse.REJECT
        with self._lock:
            if self._reserved:
                return GoalResponse.REJECT
            self._reserved = True
        self._clear_preview()
        return GoalResponse.ACCEPT

    def _validate_goal(self, request: Plan.Goal) -> None:
        target = request.target
        if request.planning_scope != "execution_strict":
            raise ValueError("live control only accepts execution_strict")
        if target.header.frame_id != "machine_root_ros":
            raise ValueError("target frame must be machine_root_ros")
        resolved = resolve_planning_target(
            target,
            mission=self._mission,
            demo=self._demo,
        )
        if (
            resolved.target_status not in self._allowed_target_statuses
            or target.target_status != resolved.target_status
        ):
            allowed = ", ".join(sorted(self._allowed_target_statuses))
            raise ValueError(
                f"live supervised control requires Mission target_status in: {allowed}"
            )
        if target.target_kind not in {"dig", "dump"} or target.mission_phase != target.target_kind:
            raise ValueError("target phase mismatch")
        expected = resolved.target
        actual = (target.position.x, target.position.y, target.position.z)
        if math.dist(actual, expected.position_m) > 1e-9 or abs(target.radius_m - expected.radius_m) > 1e-9:
            raise ValueError("target geometry does not match loaded Mission snapshot")
        stamp_s = float(target.header.stamp.sec) + float(target.header.stamp.nanosec) * 1e-9
        age_s = self._now_s() - stamp_s
        if stamp_s <= 0.0 or age_s < 0.0 or age_s > 1.5:
            raise ValueError("target snapshot is stale")

    def _execute(self, goal_handle) -> Plan.Result:
        self._feedback(goal_handle, "freezing_live_inputs", 0)
        try:
            inputs_frozen_at_s = self._now_s()
            live = load_live_planning_inputs(self._profile, now_s=inputs_frozen_at_s)
            target = _target_dict(goal_handle.request.target)
            resolved = resolve_planning_target(
                goal_handle.request.target,
                mission=self._mission,
                demo=self._demo,
            )
            local_map, intent = inject_live_target(live.local_map, target)
            snapshot = LivePlanningInputs(local_map=local_map, bucket_tip=live.bucket_tip)
            invalidate_outputs(self._profile.outputs)
            self._feedback(goal_handle, "planning_execution_strict", 0)
            self._profile.outputs.directory.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(
                prefix=".planning-action-", dir=self._profile.outputs.directory
            ) as directory:
                prepared = prepare_planning_snapshot(
                    self._profile,
                    intent,
                    snapshot,
                    python=ROS_PYTHON,
                    staging_dir=FsPath(directory),
                    final_outputs=self._profile.outputs,
                    planning_scope="execution_strict",
                )
                execute_prepared_run(prepared, dry_run=False)
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, Plan.Result.OUTCOME_CANCELLED, "CANCELLED", "planning cancelled")
            trajectory_data = json.loads(self._profile.outputs.trajectory.read_text(encoding="utf-8"))
            created_at_s = self._now_s()
            fields = build_execution_snapshot_fields(
                trajectory_data,
                target,
                source_bucket_tip_stamp_s=float(live.bucket_tip["stamp_s"]),
                source_local_map_stamp_s=float(live.local_map["timestamp_s"]),
                inputs_frozen_at_s=inputs_frozen_at_s,
                created_at_s=created_at_s,
                waypoint_tolerance_m=resolved.limits.waypoint_tolerance_m,
                waypoint_dwell_s=resolved.limits.waypoint_dwell_s,
                tracking_timeout_s=resolved.limits.tracking_timeout_s,
                control_stage=self._control_policy.name,
            )
            message = _trajectory_message(fields, target["target_id"])
            self._trajectory_publisher.publish(message)
            self._publish_preview(message)
            self._feedback(goal_handle, "published", 0)
            return self._finish(
                goal_handle,
                Plan.Result.OUTCOME_SUCCEEDED,
                "SUCCEEDED",
                "live execution-strict trajectory planned",
                trajectory=message,
            )
        except (OSError, ValueError, subprocess.CalledProcessError) as exc:
            self.get_logger().error(f"live Plan failed: {exc}")
            return self._finish(goal_handle, Plan.Result.OUTCOME_FAILED, "PLANNING_FAILED", str(exc))
        except Exception as exc:
            self.get_logger().error(f"live Plan internal error: {exc}")
            return self._finish(goal_handle, Plan.Result.OUTCOME_FAILED, "INTERNAL_ERROR", str(exc))

    def _finish(self, goal_handle, outcome, reason, message, *, trajectory=None):
        try:
            if outcome == Plan.Result.OUTCOME_SUCCEEDED:
                goal_handle.succeed()
            elif outcome == Plan.Result.OUTCOME_CANCELLED:
                self._clear_preview()
                goal_handle.canceled()
            else:
                self._clear_preview()
                goal_handle.abort()
        finally:
            with self._lock:
                self._reserved = False
        result = Plan.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        if trajectory is not None:
            result.trajectory = trajectory
        result.action_datagrams = 0
        return result

    def _feedback(self, goal_handle, stage: str, iterations: int) -> None:
        feedback = Plan.Feedback()
        feedback.stage = stage
        feedback.iterations = iterations
        goal_handle.publish_feedback(feedback)

    def _clear_preview(self) -> None:
        path = Path()
        path.header.frame_id = "machine_root_ros"
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_publisher.publish(path)
        marker = Marker()
        marker.action = Marker.DELETEALL
        self._marker_publisher.publish(MarkerArray(markers=[marker]))

    def _publish_preview(self, trajectory: TrajectorySnapshot) -> None:
        path = Path()
        path.header = trajectory.header
        for point in trajectory.waypoints:
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose.position = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)
        marker = Marker()
        marker.header = trajectory.header
        marker.ns = "live_execution_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.3, 1.0
        marker.points = list(trajectory.waypoints)
        self._marker_publisher.publish(MarkerArray(markers=[marker]))


def _target_dict(message) -> dict:
    return {
        "target_id": message.target_id,
        "target_kind": message.target_kind,
        "mission_id": message.mission_id,
        "mission_sha256": message.mission_sha256,
        "mission_phase": message.mission_phase,
        "position_m": [message.position.x, message.position.y, message.position.z],
        "normal": [message.normal.x, message.normal.y, message.normal.z],
        "radius_m": message.radius_m,
    }


def _trajectory_message(fields: dict, target_id: str) -> TrajectorySnapshot:
    message = TrajectorySnapshot()
    message.header.frame_id = fields["frame_id"]
    message.header.stamp = _time(fields["created_at_s"])
    for name in (
        "mission_id", "mission_sha256", "mission_phase", "task_mode", "planning_scope",
        "control_stage", "workspace_constraint",
        "input_source", "map_source", "clock_mode",
    ):
        setattr(message, name, fields[name])
    message.execution_eligible = fields["execution_eligible"]
    message.source_bucket_tip_stamp = _time(fields["source_bucket_tip_stamp_s"])
    message.source_local_map_stamp = _time(fields["source_local_map_stamp_s"])
    message.inputs_frozen_at = _time(fields["inputs_frozen_at_s"])
    message.valid_until = _time(fields["valid_until_s"])
    message.waypoints = [Point(x=x, y=y, z=z) for x, y, z in fields["waypoints"]]
    message.waypoint_tolerance_m = fields["waypoint_tolerance_m"]
    message.waypoint_dwell_s = fields["waypoint_dwell_s"]
    message.tracking_timeout_s = fields["tracking_timeout_s"]
    digest = trajectory_snapshot_message_sha256(message)
    message.trajectory_id = f"live-{target_id}-{digest[:12]}"
    message.trajectory_sha256 = digest
    return message


def _time(value: float) -> Time:
    seconds = int(math.floor(value))
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Live execution-strict Plan Action server")
    parser.add_argument("--profile", type=FsPath, required=True)
    parser.add_argument("--mission", type=FsPath, required=True)
    parser.add_argument("--demo", type=FsPath)
    parser.add_argument("--urdf", type=FsPath, required=True)
    parser.add_argument("--control-stage", choices=CONTROL_STAGES, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = LivePlanActionNode(
        profile_path=args.profile,
        mission_path=args.mission,
        demo_path=args.demo,
        urdf_path=args.urdf,
        control_stage=args.control_stage,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
