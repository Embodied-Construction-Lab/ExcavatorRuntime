#!/usr/bin/python3
"""Plan one Mission phase and pass that per-goal Result directly to shadow Follow."""

from __future__ import annotations

import argparse
import copy
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from airy_excavator_interfaces.action import Follow, Plan
from airy_excavator_interfaces.msg import RuntimeStatus
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from mission.contract import ExcavationMission, load_mission


@dataclass(frozen=True)
class PlanFollowOutcome:
    plan_result: Plan.Result
    follow_result: Follow.Result


class PlanFollowShadowClient(Node):
    """Causally bind one Plan Result to one shadow Follow Goal."""

    _NODE_NAME = "plan_follow_shadow_client"
    _RUNTIME_LABEL = "Shadow"
    _PLANNING_SCOPE = "preview_global"
    _EXECUTION_ELIGIBLE = False

    def __init__(self, *, context=None) -> None:
        super().__init__(self._NODE_NAME, context=context)
        self._runtime_status = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_subscription = self.create_subscription(
            RuntimeStatus,
            "/mission/runtime_status",
            self._on_runtime_status,
            status_qos,
        )
        self._plan = ActionClient(self, Plan, "/planning/plan")
        self._follow = ActionClient(self, Follow, "/excavator/follow")
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)

    def run_phase(
        self,
        *,
        mission: ExcavationMission,
        phase: str,
        wait_s: float,
        target_id: str | None = None,
    ) -> PlanFollowOutcome:
        if phase not in ("dig", "dump"):
            raise ValueError("phase must be dig or dump")
        if wait_s <= 0.0:
            raise ValueError("wait_s must be positive")
        self.require_runtime_ready(wait_s)
        plan_result = self.plan_phase(
            mission=mission,
            phase=phase,
            wait_s=wait_s,
            target_id=target_id,
        )
        follow_result = self.follow_trajectory(plan_result.trajectory, wait_s=wait_s)
        return PlanFollowOutcome(plan_result=plan_result, follow_result=follow_result)

    def require_runtime_ready(
        self, wait_s: float, *, expected_input_source: str | None = None
    ) -> None:
        self._require_runtime_status(wait_s, expected_input_source=expected_input_source)

    def plan_phase(
        self,
        *,
        mission: ExcavationMission,
        phase: str,
        wait_s: float,
        target_id: str | None = None,
    ) -> Plan.Result:
        if phase not in ("dig", "dump"):
            raise ValueError("phase must be dig or dump")
        if wait_s <= 0.0:
            raise ValueError("wait_s must be positive")
        plan_handle = None
        plan_result_future = None
        if not self._plan.wait_for_server(timeout_sec=wait_s):
            raise RuntimeError("Plan Action Server is unavailable")
        try:
            plan_send = self._plan.send_goal_async(
                _build_plan_goal(
                    self,
                    mission,
                    phase,
                    planning_scope=self._PLANNING_SCOPE,
                    target_id=target_id,
                ),
                feedback_callback=_print_plan_feedback,
            )
            plan_handle = self._wait(plan_send, wait_s, "Plan goal response")
            if plan_handle is None or not plan_handle.accepted:
                plan_handle = None
                raise RuntimeError("Plan goal was rejected")
            print("plan goal accepted", flush=True)

            plan_result_future = plan_handle.get_result_async()
            plan_response = self._wait(plan_result_future, wait_s, "Plan result")
            plan_handle = None
            plan_result = plan_response.result
            self._validate_action_datagrams("Plan", plan_result.action_datagrams)
            if (
                plan_response.status != GoalStatus.STATUS_SUCCEEDED
                or plan_result.outcome != Plan.Result.OUTCOME_SUCCEEDED
                or plan_result.reason_code != "SUCCEEDED"
            ):
                raise RuntimeError(f"Plan failed: {plan_result.reason_code}")
            trajectory = plan_result.trajectory
            _validate_trajectory(
                self,
                mission,
                phase,
                trajectory,
                execution_eligible=self._EXECUTION_ELIGIBLE,
                planning_scope=self._PLANNING_SCOPE,
            )
            print(
                f"plan result: SUCCEEDED trajectory_id={trajectory.trajectory_id} "
                f"waypoints={len(trajectory.waypoints)} action_datagrams=0",
                flush=True,
            )
            return plan_result
        finally:
            if plan_handle is not None:
                self._cancel_and_wait(
                    plan_handle, plan_result_future, wait_s, "Plan"
                )

    def follow_trajectory(self, trajectory, *, wait_s: float) -> Follow.Result:
        if wait_s <= 0.0:
            raise ValueError("wait_s must be positive")
        follow_handle = None
        follow_result_future = None
        if not self._follow.wait_for_server(timeout_sec=wait_s):
            raise RuntimeError("Follow Action Server is unavailable")
        self.require_runtime_ready(wait_s, expected_input_source=trajectory.input_source)
        try:
            follow_goal = Follow.Goal()
            follow_goal.trajectory = copy.deepcopy(trajectory)
            follow_send = self._follow.send_goal_async(
                follow_goal, feedback_callback=_print_follow_feedback
            )
            follow_handle = self._wait(follow_send, wait_s, "Follow goal response")
            if follow_handle is None or not follow_handle.accepted:
                follow_handle = None
                raise RuntimeError("Follow goal was rejected")
            print(
                f"follow goal accepted: trajectory_id={trajectory.trajectory_id}",
                flush=True,
            )

            result_wait_s = trajectory.tracking_timeout_s + wait_s
            follow_result_future = follow_handle.get_result_async()
            follow_response = self._wait(
                follow_result_future, result_wait_s, "Follow result"
            )
            follow_handle = None
            follow_result = follow_response.result
            self._validate_action_datagrams(
                "Follow", follow_result.action_datagrams
            )
            if not follow_result.quiescence_confirmed:
                raise RuntimeError("Follow Result was published before quiescence")
            if (
                follow_response.status != GoalStatus.STATUS_SUCCEEDED
                or follow_result.outcome != Follow.Result.OUTCOME_SUCCEEDED
                or follow_result.reason_code != "SUCCEEDED"
            ):
                raise RuntimeError(f"Follow failed: {follow_result.reason_code}")
            print(
                "follow result: SUCCEEDED quiescence_confirmed=True "
                f"action_datagrams={follow_result.action_datagrams}",
                flush=True,
            )
            return follow_result
        finally:
            if follow_handle is not None:
                self._cancel_and_wait(
                    follow_handle, follow_result_future, wait_s, "Follow"
                )

    def _wait(self, future, timeout_s: float, operation: str):
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError(f"{operation} timed out")
        value = future.result()
        if value is None:
            raise RuntimeError(f"{operation} returned no value")
        return value

    def _on_runtime_status(self, message: RuntimeStatus) -> None:
        self._runtime_status = message

    def _require_runtime_status(
        self, wait_s: float, *, expected_input_source: str | None = None
    ) -> None:
        deadline = time.monotonic() + wait_s
        status_received = False
        while time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=0.05)
            status = self._runtime_status
            if status is None:
                continue
            status_received = True
            status_stamp_s = (
                status.header.stamp.sec + status.header.stamp.nanosec * 1e-9
            )
            status_age_s = (
                self.get_clock().now().nanoseconds * 1e-9 - status_stamp_s
            )
            valid = self._runtime_status_is_valid(
                status, status_stamp_s, status_age_s
            )
            if expected_input_source is not None:
                valid = valid and status.input_source == expected_input_source
            if valid:
                return
        if not status_received:
            raise RuntimeError(f"{self._RUNTIME_LABEL} RuntimeStatus is unavailable")
        raise RuntimeError(
            f"{self._RUNTIME_LABEL} RuntimeStatus safety contract is not satisfied"
        )

    def _runtime_status_is_valid(
        self, status: RuntimeStatus, status_stamp_s: float, status_age_s: float
    ) -> bool:
        return (
            status.header.frame_id == "machine_root_ros"
            and status_stamp_s > 0.0
            and 0.0 <= status_age_s <= 1.5
            and status.execution_mode == "shadow"
            and status.motion_backend == "none"
            and not status.motion_authorized
            and not status.sender_constructed
            and status.quiescent
            and status.action_datagrams == 0
            and not status.active_behavior
        )

    def _validate_action_datagrams(self, behavior: str, count: int) -> None:
        if count != 0:
            raise RuntimeError(
                f"{behavior} violated shadow no-datagram invariant"
            )

    def destroy_node(self):
        self._executor.remove_node(self)
        self._executor.shutdown(timeout_sec=1.0)
        return super().destroy_node()

    def _cancel_and_wait(self, goal_handle, result_future, wait_s, behavior):
        cancel = self._wait(
            goal_handle.cancel_goal_async(), wait_s, f"{behavior} cancel response"
        )
        if result_future is None:
            result_future = goal_handle.get_result_async()
        terminal = self._wait(
            result_future, wait_s, f"{behavior} cancelled result"
        )
        if not cancel.goals_canceling and terminal.status not in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }:
            raise RuntimeError(f"{behavior} Goal could not be cancelled")
        result = terminal.result
        self._validate_action_datagrams(behavior, result.action_datagrams)
        if behavior == "Follow" and not result.quiescence_confirmed:
            raise RuntimeError("Follow cancel Result was published before quiescence")
        expected = {
            GoalStatus.STATUS_SUCCEEDED: result.OUTCOME_SUCCEEDED,
            GoalStatus.STATUS_CANCELED: result.OUTCOME_CANCELLED,
            GoalStatus.STATUS_ABORTED: result.OUTCOME_FAILED,
        }.get(terminal.status)
        if expected is None or result.outcome != expected:
            raise RuntimeError(f"{behavior} cancel returned an inconsistent terminal state")


class PlanFollowLiveClient(PlanFollowShadowClient):
    """Causally bind one execution-strict Plan Result to live Orin Follow."""

    _NODE_NAME = "plan_follow_live_client"
    _RUNTIME_LABEL = "Live"
    _PLANNING_SCOPE = "execution_strict"
    _EXECUTION_ELIGIBLE = True

    def _runtime_status_is_valid(
        self, status: RuntimeStatus, status_stamp_s: float, status_age_s: float
    ) -> bool:
        return (
            status.header.frame_id == "machine_root_ros"
            and status_stamp_s > 0.0
            and 0.0 <= status_age_s <= 1.5
            and status.execution_mode == "control"
            and status.motion_backend == "orin_edge"
            and status.motion_authorized
            and status.sender_constructed
            and status.quiescent
            and not status.active_behavior
            and status.input_source == "live"
        )

    def _validate_action_datagrams(self, behavior: str, count: int) -> None:
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise RuntimeError(f"{behavior} action_datagrams is invalid")
        if behavior == "Plan" and count != 0:
            raise RuntimeError("Plan must not emit action datagrams")


def _build_plan_goal(
    node: Node,
    mission: ExcavationMission,
    phase: str,
    *,
    planning_scope: str = "preview_global",
    target_id: str | None = None,
) -> Plan.Goal:
    target = mission.targets[phase]
    goal = Plan.Goal()
    goal.planning_scope = planning_scope
    goal.target.header.frame_id = mission.frame_id
    goal.target.header.stamp = node.get_clock().now().to_msg()
    goal.target.target_id = target_id or f"{mission.mission_id}:{phase}"
    goal.target.target_kind = phase
    goal.target.target_status = mission.target_status
    goal.target.mission_id = mission.mission_id
    goal.target.mission_sha256 = mission.sha256
    goal.target.mission_phase = phase
    goal.target.position = Point(
        x=target.position_m[0], y=target.position_m[1], z=target.position_m[2]
    )
    goal.target.normal = Vector3(
        x=target.normal[0], y=target.normal[1], z=target.normal[2]
    )
    goal.target.radius_m = target.radius_m
    return goal


def _validate_trajectory(
    node: Node,
    mission: ExcavationMission,
    phase: str,
    trajectory,
    *,
    execution_eligible: bool = False,
    planning_scope: str = "preview_global",
) -> None:
    if trajectory.trajectory_sha256 != trajectory_snapshot_message_sha256(
        trajectory
    ):
        raise ValueError("Plan Result trajectory digest mismatch")
    if trajectory.execution_eligible is not execution_eligible:
        expected = "true" if execution_eligible else "false"
        raise ValueError(f"Follow requires execution_eligible={expected}")
    if trajectory.planning_scope != planning_scope:
        raise ValueError(
            f"Plan Result planning_scope must be {planning_scope}"
        )
    if (
        trajectory.mission_id != mission.mission_id
        or trajectory.mission_sha256 != mission.sha256
        or trajectory.mission_phase != phase
    ):
        raise ValueError("Plan Result Mission provenance mismatch")
    if not trajectory.waypoints:
        raise ValueError("Plan Result has no waypoints")
    expected_target = mission.targets[phase]
    endpoint = trajectory.waypoints[-1]
    target_error_m = math.dist(
        (endpoint.x, endpoint.y, endpoint.z), expected_target.position_m
    )
    if target_error_m > expected_target.radius_m:
        raise ValueError(
            f"Plan Result endpoint does not match Mission target: "
            f"error_m={target_error_m:.4f}"
        )
    valid_until_s = (
        trajectory.valid_until.sec + trajectory.valid_until.nanosec * 1e-9
    )
    now_s = node.get_clock().now().nanoseconds * 1e-9
    if valid_until_s < now_s:
        raise ValueError("Plan Result trajectory expired before Follow submission")


def _print_plan_feedback(message) -> None:
    feedback = message.feedback
    print(
        f"plan feedback: stage={feedback.stage} iterations={feedback.iterations}",
        flush=True,
    )


def _print_follow_feedback(message) -> None:
    feedback = message.feedback
    print(
        f"follow feedback: waypoint={feedback.current_waypoint_index + 1}/"
        f"{feedback.waypoint_count} distance_m={feedback.distance_m:.4f} "
        f"state={feedback.tracking_state} "
        f"action_datagrams={feedback.action_datagrams}",
        flush=True,
    )


def build_arg_parser(
    description: str = "Run one causally bound Plan→Follow cycle in shadow mode.",
) -> argparse.ArgumentParser:
    default_mission = (
        get_package_share_directory("airy_mission_runtime")
        + "/config/excavation_cycle.json"
    )
    parser = argparse.ArgumentParser(
        description=description
    )
    parser.add_argument("phase", choices=("dig", "dump"))
    parser.add_argument("--mission", default=default_mission)
    parser.add_argument("--wait-s", type=float, default=5.0)
    return parser


def run() -> int:
    args = build_arg_parser().parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PlanFollowShadowClient()
    try:
        node.run_phase(
            mission=load_mission(args.mission),
            phase=args.phase,
            wait_s=args.wait_s,
        )
        return 0
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Plan→Follow shadow failed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Plan→Follow shadow cancelled by operator", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
