#!/usr/bin/python3
"""PC Mission scheduler: Follow DIG -> ExecuteDig -> Follow DUMP -> ExecuteDump."""

from __future__ import annotations

import copy
import threading
import time
import uuid

import rclpy
from airy_excavator_interfaces.action import ExcavationCycle, Plan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from runtime_bridge.orin_behavior_rpc import (
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
    trajectory_snapshot_to_message,
)
from runtime_bridge.orin_cycle_client import CycleLegRejected, OrinCycleClient


_CHILD_FUTURE_POLL_S = 0.01
_FEEDBACK_PERIOD_S = 0.1


class ChildFailure(RuntimeError):
    def __init__(
        self,
        stage: str,
        reason: str,
        message: str,
        datagrams: int = 0,
        quiescent: bool = False,
    ) -> None:
        super().__init__(message)
        self.stage = stage
        self.reason = reason
        self.datagrams = datagrams
        self.quiescent = quiescent


class MissionCancelled(ChildFailure):
    pass


class ExcavationCycleNode(Node):
    def __init__(self, *, context=None, orin_cycle_client=None) -> None:
        super().__init__("excavation_cycle_server", context=context)
        self._group = ReentrantCallbackGroup()
        self._plan = ActionClient(self, Plan, "/planning/plan", callback_group=self._group)
        if orin_cycle_client is None:
            self.declare_parameter("orin_host", "127.0.0.1")
            self.declare_parameter("orin_port", 18083)
            orin_cycle_client = OrinCycleClient(
                str(self.get_parameter("orin_host").value),
                int(self.get_parameter("orin_port").value),
            )
        self._orin_cycle = orin_cycle_client
        self._lock = threading.Lock()
        self._reserved = False
        self._server = ActionServer(
            self,
            ExcavationCycle,
            "/mission/run_cycle",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self.get_logger().info(
            "ExcavationCycle ready: PC PlanDig -> Orin DigLeg -> "
            "PC PlanDump -> Orin DumpLeg"
        )

    def destroy_node(self):
        self._server.destroy()
        return super().destroy_node()

    def _on_goal(self, request: ExcavationCycle.Goal) -> GoalResponse:
        if (
            request.dig_target.target_kind != "dig"
            or request.dump_target.target_kind != "dump"
            or request.dig_target.mission_id != request.dump_target.mission_id
            or request.dig_target.mission_sha256 != request.dump_target.mission_sha256
        ):
            return GoalResponse.REJECT
        with self._lock:
            if self._reserved:
                return GoalResponse.REJECT
            self._reserved = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> ExcavationCycle.Result:
        datagrams = 0
        completed_stage = ""
        cycle_id = "%s:%s" % (
            goal_handle.request.dig_target.mission_id,
            uuid.uuid4().hex,
        )
        try:
            dig_trajectory = self._plan_phase(goal_handle, goal_handle.request.dig_target, "PLAN_DIG")
            completed_stage = "PLAN_DIG"
            dig_leg = self._run_orin_leg(
                goal_handle,
                cycle_id=cycle_id,
                trajectory=dig_trajectory,
                stage="DIG_LEG",
                run=self._orin_cycle.run_cycle_dig_leg,
                expected_reason="DIG_LEG_COMPLETED",
                expected_completed_stage="EXECUTE_DIG",
            )
            datagrams = dig_leg.action_datagrams
            completed_stage = dig_leg.completed_stage
            try:
                dump_trajectory = self._plan_phase(
                    goal_handle,
                    goal_handle.request.dump_target,
                    "PLAN_DUMP",
                )
            except Exception:
                self._cancel_waiting_cycle(cycle_id)
                raise
            completed_stage = "PLAN_DUMP"
            dump_leg = self._run_orin_leg(
                goal_handle,
                cycle_id=cycle_id,
                trajectory=dump_trajectory,
                stage="DUMP_LEG",
                run=self._orin_cycle.run_cycle_dump_leg,
                expected_reason="SEQUENCE_COMPLETED",
                expected_completed_stage="EXECUTE_DUMP",
            )
            datagrams = dump_leg.action_datagrams
            completed_stage = dump_leg.completed_stage
            goal_handle.succeed()
            return self._result(
                ExcavationCycle.Result.OUTCOME_SUCCEEDED,
                "SUCCEEDED",
                "excavation cycle sequence completed",
                completed_stage,
                True,
                datagrams,
            )
        except MissionCancelled as exc:
            datagrams = max(datagrams, exc.datagrams)
            goal_handle.canceled()
            return self._result(
                ExcavationCycle.Result.OUTCOME_CANCELLED,
                "CANCELLED",
                str(exc),
                completed_stage,
                exc.quiescent,
                datagrams,
            )
        except ChildFailure as exc:
            datagrams = max(datagrams, exc.datagrams)
            self.get_logger().error(f"Mission stage {exc.stage} failed: {exc.reason}: {exc}")
            goal_handle.abort()
            return self._result(
                ExcavationCycle.Result.OUTCOME_FAILED,
                exc.reason,
                f"{exc.stage}: {exc}",
                completed_stage,
                exc.quiescent,
                datagrams,
            )
        except Exception as exc:
            self.get_logger().error(f"Mission internal error: {exc}")
            goal_handle.abort()
            return self._result(
                ExcavationCycle.Result.OUTCOME_FAILED,
                "INTERNAL_ERROR",
                str(exc),
                completed_stage,
                False,
                datagrams,
            )
        finally:
            with self._lock:
                self._reserved = False

    def _plan_phase(self, parent, target, stage: str):
        goal = Plan.Goal()
        goal.target = self._fresh_target(target)
        goal.planning_scope = "execution_strict"
        wrapped = self._run_child(parent, self._plan, goal, stage)
        result = wrapped.result
        if result.outcome != Plan.Result.OUTCOME_SUCCEEDED or result.reason_code != "SUCCEEDED" or result.action_datagrams != 0:
            raise ChildFailure(
                stage,
                result.reason_code or "PLAN_FAILED",
                result.message,
                quiescent=result.action_datagrams == 0,
            )
        return result.trajectory

    def _run_orin_leg(
        self,
        parent,
        *,
        cycle_id: str,
        trajectory,
        stage: str,
        run,
        expected_reason: str,
        expected_completed_stage: str,
    ):
        self._feedback(parent, stage, "sending trajectory to Orin", 0)
        try:
            result = run(
                cycle_id,
                trajectory_snapshot_to_message(trajectory),
                feedback_callback=lambda update: self._feedback(
                    parent,
                    update.stage,
                    update.message,
                    update.action_datagrams,
                ),
                cancel_requested=lambda: parent.is_cancel_requested,
            )
        except CycleLegRejected as exc:
            raise ChildFailure(
                stage,
                exc.reason_code,
                exc.message,
                quiescent=True,
            ) from exc
        except (OrinBehaviorConnectionError, OrinBehaviorProtocolError) as exc:
            raise ChildFailure(
                stage,
                "ORIN_RPC_ERROR",
                str(exc),
                quiescent=False,
            ) from exc
        if result.outcome == "CANCELLED":
            raise MissionCancelled(
                stage,
                result.reason_code or "CANCELLED",
                result.message,
                result.action_datagrams,
                result.quiescence_confirmed,
            )
        if (
            result.outcome != "SUCCEEDED"
            or result.reason_code != expected_reason
            or result.completed_stage != expected_completed_stage
            or not result.quiescence_confirmed
        ):
            raise ChildFailure(
                stage,
                result.reason_code or "ORIN_CYCLE_LEG_FAILED",
                result.message,
                result.action_datagrams,
                result.quiescence_confirmed,
            )
        return result

    def _cancel_waiting_cycle(self, cycle_id: str) -> None:
        try:
            result = self._orin_cycle.cancel_cycle(cycle_id)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to cancel Orin cycle {cycle_id} at planning boundary: {exc}"
            )
            return
        if result.outcome != "CANCELLED" or not result.quiescence_confirmed:
            self.get_logger().error(
                "Orin cycle boundary cancel did not confirm quiescence: "
                f"cycle_id={cycle_id} outcome={result.outcome} "
                f"reason={result.reason_code}"
            )

    def _fresh_target(self, target):
        snapshot = copy.deepcopy(target)
        snapshot.header.stamp = self.get_clock().now().to_msg()
        return snapshot

    def _run_child(self, parent, client, goal, stage: str):
        if not client.wait_for_server(timeout_sec=1.0):
            raise ChildFailure(stage, "ACTION_SERVER_UNAVAILABLE", f"{stage} Action Server unavailable")
        self._feedback(parent, stage, "sending goal", 0)
        send_future = client.send_goal_async(goal)
        self._wait_future(parent, send_future, stage, None)
        child = send_future.result()
        if child is None or not child.accepted:
            raise ChildFailure(stage, "GOAL_REJECTED", f"{stage} goal rejected")
        result_future = child.get_result_async()
        self._wait_future(parent, result_future, stage, child)
        wrapped = result_future.result()
        if parent.is_cancel_requested:
            result = wrapped.result
            raise MissionCancelled(
                stage,
                "CANCELLED",
                f"Mission cancelled during {stage}",
                getattr(result, "action_datagrams", 0),
                getattr(result, "quiescence_confirmed", False),
            )
        return wrapped

    def _wait_future(self, parent, future, stage: str, child) -> None:
        cancel_sent = False
        next_feedback_at = 0.0
        while rclpy.ok(context=self.context) and not future.done():
            if parent.is_cancel_requested and child is not None and not cancel_sent:
                child.cancel_goal_async()
                cancel_sent = True
            now_s = time.monotonic()
            if now_s >= next_feedback_at:
                self._feedback(
                    parent,
                    stage,
                    "cancelling" if cancel_sent else "running",
                    0,
                )
                next_feedback_at = now_s + _FEEDBACK_PERIOD_S
            time.sleep(_CHILD_FUTURE_POLL_S)
        if not future.done() or future.exception() is not None:
            raise ChildFailure(stage, "ACTION_TRANSPORT_ERROR", f"{stage} Action future failed")

    def _feedback(self, goal_handle, stage: str, message: str, datagrams: int) -> None:
        feedback = ExcavationCycle.Feedback()
        feedback.stage = stage
        feedback.message = message
        feedback.action_datagrams = datagrams
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(outcome, reason, message, completed_stage, quiescent, datagrams):
        result = ExcavationCycle.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        result.completed_stage = completed_stage
        result.quiescence_confirmed = quiescent
        result.action_datagrams = datagrams
        return result


def main() -> None:
    rclpy.init()
    node = ExcavationCycleNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
