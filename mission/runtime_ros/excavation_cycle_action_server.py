#!/usr/bin/python3
"""PC Mission scheduler: Follow DIG -> ExecuteDig -> Follow DUMP -> ExecuteDump."""

from __future__ import annotations

import copy
import threading
import time

import rclpy
from airy_excavator_interfaces.action import (
    ExcavationCycle,
    ExecuteDig,
    ExecuteDump,
    Follow,
    Plan,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


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
    def __init__(self, *, context=None) -> None:
        super().__init__("excavation_cycle_server", context=context)
        self._group = ReentrantCallbackGroup()
        self._plan = ActionClient(self, Plan, "/planning/plan", callback_group=self._group)
        self._follow = ActionClient(self, Follow, "/excavator/follow", callback_group=self._group)
        self._dig = ActionClient(self, ExecuteDig, "/excavator/execute_dig", callback_group=self._group)
        self._dump = ActionClient(self, ExecuteDump, "/excavator/execute_dump", callback_group=self._group)
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
            "ExcavationCycle ready: PlanDig->Follow->ExecuteDig->PlanDump->Follow->ExecuteDump"
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
        try:
            dig_trajectory = self._plan_phase(goal_handle, goal_handle.request.dig_target, "PLAN_DIG")
            completed_stage = "PLAN_DIG"
            datagrams += self._follow_phase(goal_handle, dig_trajectory, "FOLLOW_DIG")
            completed_stage = "FOLLOW_DIG"
            datagrams += self._fixed_phase(goal_handle, self._dig, ExecuteDig, goal_handle.request.dig_target, "EXECUTE_DIG")
            completed_stage = "EXECUTE_DIG"
            dump_trajectory = self._plan_phase(goal_handle, goal_handle.request.dump_target, "PLAN_DUMP")
            completed_stage = "PLAN_DUMP"
            datagrams += self._follow_phase(goal_handle, dump_trajectory, "FOLLOW_DUMP")
            completed_stage = "FOLLOW_DUMP"
            datagrams += self._fixed_phase(goal_handle, self._dump, ExecuteDump, goal_handle.request.dump_target, "EXECUTE_DUMP")
            completed_stage = "EXECUTE_DUMP"
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
            datagrams += exc.datagrams
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
            datagrams += exc.datagrams
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

    def _follow_phase(self, parent, trajectory, stage: str) -> int:
        goal = Follow.Goal()
        goal.trajectory = trajectory
        wrapped = self._run_child(parent, self._follow, goal, stage)
        result = wrapped.result
        if result.outcome != Follow.Result.OUTCOME_SUCCEEDED or result.reason_code != "SUCCEEDED" or not result.quiescence_confirmed:
            raise ChildFailure(
                stage,
                result.reason_code or "FOLLOW_FAILED",
                result.message,
                result.action_datagrams,
                result.quiescence_confirmed,
            )
        return result.action_datagrams

    def _fixed_phase(self, parent, client, action_type, target, stage: str) -> int:
        goal = action_type.Goal()
        goal.target = self._fresh_target(target)
        wrapped = self._run_child(parent, client, goal, stage)
        result = wrapped.result
        if result.outcome != action_type.Result.OUTCOME_SUCCEEDED or result.reason_code != "SEQUENCE_COMPLETED" or not result.quiescence_confirmed:
            raise ChildFailure(
                stage,
                result.reason_code or "FIXED_ACTION_FAILED",
                result.message,
                result.action_datagrams,
                result.quiescence_confirmed,
            )
        return result.action_datagrams

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
