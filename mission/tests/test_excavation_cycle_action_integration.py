import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import ExcavationCycle, ExecuteDig, ExecuteDump, Follow, Plan
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
            ActionServer(self, Follow, "/excavator/follow", execute_callback=self._follow),
            ActionServer(self, ExecuteDig, "/excavator/execute_dig", execute_callback=self._dig),
            ActionServer(self, ExecuteDump, "/excavator/execute_dump", execute_callback=self._dump),
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

    def _follow(self, handle):
        phase = handle.request.trajectory.mission_phase
        stage = f"FOLLOW_{phase.upper()}"
        self.calls.append(stage)
        result = Follow.Result()
        result.action_datagrams = 2
        result.quiescence_confirmed = True
        if self.fail_stage == stage:
            result.outcome = Follow.Result.OUTCOME_FAILED
            result.reason_code = "FIXTURE_FAILURE"
            result.message = "follow fixture failure"
            handle.abort()
            return result
        result.outcome = Follow.Result.OUTCOME_SUCCEEDED
        result.reason_code = "SUCCEEDED"
        handle.succeed()
        return result

    def _dig(self, handle):
        return self._fixed(handle, ExecuteDig, "EXECUTE_DIG")

    def _dump(self, handle):
        return self._fixed(handle, ExecuteDump, "EXECUTE_DUMP")

    def _fixed(self, handle, action_type, stage):
        self.calls.append(stage)
        result = action_type.Result()
        result.action_datagrams = 3
        result.quiescence_confirmed = True
        if self.fail_stage == stage:
            result.outcome = action_type.Result.OUTCOME_FAILED
            result.reason_code = "FIXTURE_FAILURE"
            result.message = "fixed fixture failure"
            handle.abort()
            return result
        result.outcome = action_type.Result.OUTCOME_SUCCEEDED
        result.reason_code = "SEQUENCE_COMPLETED"
        handle.succeed()
        return result


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
    scheduler = ExcavationCycleNode(context=context)
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
            "FOLLOW_DIG",
            "EXECUTE_DIG",
            "PLAN_DUMP",
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
        assert children.calls == ["PLAN_DIG", "FOLLOW_DIG", "EXECUTE_DIG"]
    finally:
        _stop(harness)
