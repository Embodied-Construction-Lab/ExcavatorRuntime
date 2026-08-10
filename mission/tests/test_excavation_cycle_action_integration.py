import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import ExcavationCycle, Plan
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission.runtime_ros.excavation_cycle_action_server import ExcavationCycleNode


def _wait_future(future, timeout_s=4.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


class _ChildActions(Node):
    def __init__(self, *, context, fail_stage=""):
        super().__init__("excavation_cycle_children", context=context)
        self.calls = []
        self.plan_target_stamps = []
        self.fail_stage = fail_stage
        self.servers = [
            ActionServer(self, Plan, "/planning/plan", execute_callback=self._plan),
        ]

    def destroy_node(self):
        for server in self.servers:
            server.destroy()
        return super().destroy_node()

    def _plan(self, handle):
        phase = handle.request.target.target_kind
        stamp = handle.request.target.header.stamp
        self.plan_target_stamps.append(stamp.sec + stamp.nanosec * 1e-9)
        stage = f"PLAN_{phase.upper()}"
        self.calls.append(stage)
        result = Plan.Result()
        result.action_datagrams = 0
        if self.fail_stage == stage:
            result.outcome = Plan.Result.OUTCOME_FAILED
            result.reason_code = "FIXTURE_FAILURE"
            result.message = "planned fixture failure"
            handle.abort()
            return result
        result.outcome = Plan.Result.OUTCOME_SUCCEEDED
        result.reason_code = "SUCCEEDED"
        result.trajectory.mission_phase = phase
        handle.succeed()
        return result


class _OrinCycleClient:
    def __init__(self, *, fail_stage=""):
        self.calls = []
        self.fail_stage = fail_stage

    def run_cycle_dig_leg(self, cycle_id, trajectory, **kwargs):
        self.calls.extend(["FOLLOW_DIG", "EXECUTE_DIG"])
        if self.fail_stage in {"FOLLOW_DIG", "EXECUTE_DIG"}:
            return self._result(
                outcome="FAILED",
                reason="FIXTURE_FAILURE",
                message="Orin dig leg fixture failure",
                completed_stage=self.fail_stage,
                datagrams=5,
            )
        return self._result(
            outcome="SUCCEEDED",
            reason="DIG_LEG_COMPLETED",
            message="Orin dig leg completed",
            completed_stage="EXECUTE_DIG",
            datagrams=5,
        )

    def run_cycle_dump_leg(self, cycle_id, trajectory, **kwargs):
        self.calls.extend(["FOLLOW_DUMP", "EXECUTE_DUMP"])
        if self.fail_stage in {"FOLLOW_DUMP", "EXECUTE_DUMP"}:
            return self._result(
                outcome="FAILED",
                reason="FIXTURE_FAILURE",
                message="Orin dump leg fixture failure",
                completed_stage=self.fail_stage,
                datagrams=10,
            )
        return self._result(
            outcome="SUCCEEDED",
            reason="SEQUENCE_COMPLETED",
            message="Orin cycle completed",
            completed_stage="EXECUTE_DUMP",
            datagrams=10,
        )

    def cancel_cycle(self, cycle_id):
        self.calls.append("CANCEL_CYCLE")
        return self._result(
            outcome="CANCELLED",
            reason="CANCELLED",
            message="cycle cancelled while waiting for dump trajectory",
            completed_stage="CANCELLED",
            datagrams=5,
        )

    @staticmethod
    def _result(*, outcome, reason, message, completed_stage, datagrams):
        return SimpleNamespace(
            outcome=outcome,
            reason_code=reason,
            message=message,
            completed_stage=completed_stage,
            quiescence_confirmed=True,
            action_datagrams=datagrams,
        )


def _goal():
    goal = ExcavationCycle.Goal()
    for target, phase in ((goal.dig_target, "dig"), (goal.dump_target, "dump")):
        target.target_kind = phase
        target.mission_phase = phase
        target.mission_id = "integration-mission"
        target.mission_sha256 = "a" * 64
    return goal


def _harness(fail_stage=""):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    children = _ChildActions(context=context, fail_stage=fail_stage)
    children.orin_cycle = _OrinCycleClient(fail_stage=fail_stage)
    scheduler = ExcavationCycleNode(
        context=context,
        orin_cycle_client=children.orin_cycle,
    )
    client_node = rclpy.create_node("excavation_cycle_client", context=context)
    executor = MultiThreadedExecutor(num_threads=8, context=context)
    for node in (children, scheduler, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, ExcavationCycle, "/mission/run_cycle")
    assert client.wait_for_server(timeout_sec=2.0)
    return context, children, scheduler, client_node, executor, thread, client


def _stop(harness):
    context, children, scheduler, client_node, executor, thread, _ = harness
    executor.shutdown(timeout_sec=1.0)
    thread.join(timeout=1.0)
    client_node.destroy_node()
    scheduler.destroy_node()
    children.destroy_node()
    rclpy.shutdown(context=context)


def test_cycle_runs_required_order_and_replans_dump_after_dig():
    harness = _harness()
    _, children, _, _, _, _, client = harness
    try:
        started_at = time.monotonic()
        handle = _wait_future(client.send_goal_async(_goal()))
        assert handle.accepted
        wrapped = _wait_future(handle.get_result_async())
        elapsed_s = time.monotonic() - started_at
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.reason_code == "SUCCEEDED"
        assert wrapped.result.quiescence_confirmed
        assert wrapped.result.action_datagrams == 10
        assert elapsed_s < 0.75
        assert children.calls == [
            "PLAN_DIG",
            "PLAN_DUMP",
        ]
        assert children.orin_cycle.calls == [
            "FOLLOW_DIG",
            "EXECUTE_DIG",
            "FOLLOW_DUMP",
            "EXECUTE_DUMP",
        ]
        assert len(children.plan_target_stamps) == 2
        assert all(stamp > 0.0 for stamp in children.plan_target_stamps)
    finally:
        _stop(harness)


def test_cycle_stops_after_quiescent_child_failure():
    harness = _harness(fail_stage="EXECUTE_DIG")
    _, children, _, _, _, _, client = harness
    try:
        handle = _wait_future(client.send_goal_async(_goal()))
        wrapped = _wait_future(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason_code == "FIXTURE_FAILURE"
        assert wrapped.result.quiescence_confirmed
        assert children.calls == ["PLAN_DIG"]
        assert children.orin_cycle.calls == ["FOLLOW_DIG", "EXECUTE_DIG"]
    finally:
        _stop(harness)


def test_dump_leg_failure_reports_orin_cumulative_datagrams_once():
    harness = _harness(fail_stage="EXECUTE_DUMP")
    _, children, _, _, _, _, client = harness
    try:
        handle = _wait_future(client.send_goal_async(_goal()))
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason_code == "FIXTURE_FAILURE"
        assert wrapped.result.action_datagrams == 10
        assert children.calls == ["PLAN_DIG", "PLAN_DUMP"]
    finally:
        _stop(harness)


def test_dump_planning_failure_cancels_orin_cycle_waiting_at_boundary():
    harness = _harness(fail_stage="PLAN_DUMP")
    _, children, _, _, _, _, client = harness
    try:
        handle = _wait_future(client.send_goal_async(_goal()))
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason_code == "FIXTURE_FAILURE"
        assert children.calls == ["PLAN_DIG", "PLAN_DUMP"]
        assert children.orin_cycle.calls == [
            "FOLLOW_DIG",
            "EXECUTE_DIG",
            "CANCEL_CYCLE",
        ]
    finally:
        _stop(harness)


def test_cycle_can_delegate_each_motion_leg_to_the_orin_local_coordinator():
    class RecordingOrinCycleClient:
        def __init__(self):
            self.calls = []

        def run_cycle_dig_leg(
            self,
            cycle_id,
            trajectory,
            *,
            feedback_callback=None,
            status_callback=None,
            cancel_requested=None,
        ):
            self.calls.append(("DIG_LEG", cycle_id, trajectory["mission_phase"]))
            return SimpleNamespace(
                outcome="SUCCEEDED",
                reason_code="DIG_LEG_COMPLETED",
                message="dig leg completed",
                completed_stage="EXECUTE_DIG",
                quiescence_confirmed=True,
                action_datagrams=5,
            )

        def run_cycle_dump_leg(
            self,
            cycle_id,
            trajectory,
            *,
            feedback_callback=None,
            status_callback=None,
            cancel_requested=None,
        ):
            self.calls.append(("DUMP_LEG", cycle_id, trajectory["mission_phase"]))
            return SimpleNamespace(
                outcome="SUCCEEDED",
                reason_code="SEQUENCE_COMPLETED",
                message="cycle completed",
                completed_stage="EXECUTE_DUMP",
                quiescence_confirmed=True,
                action_datagrams=10,
            )

    context = rclpy.context.Context()
    rclpy.init(context=context)
    children = _ChildActions(context=context)
    orin_client = RecordingOrinCycleClient()
    scheduler = ExcavationCycleNode(
        context=context,
        orin_cycle_client=orin_client,
    )
    client_node = rclpy.create_node("orin_cycle_mission_client", context=context)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (children, scheduler, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, ExcavationCycle, "/mission/run_cycle")
    assert client.wait_for_server(timeout_sec=2.0)
    try:
        handle = _wait_future(client.send_goal_async(_goal()))
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.action_datagrams == 10
        assert children.calls == ["PLAN_DIG", "PLAN_DUMP"]
        assert [call[0] for call in orin_client.calls] == ["DIG_LEG", "DUMP_LEG"]
        assert [call[2] for call in orin_client.calls] == ["dig", "dump"]
        assert orin_client.calls[0][1] == orin_client.calls[1][1]
        assert orin_client.calls[0][1].startswith("integration-mission:")
    finally:
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        client_node.destroy_node()
        scheduler.destroy_node()
        children.destroy_node()
        rclpy.shutdown(context=context)
