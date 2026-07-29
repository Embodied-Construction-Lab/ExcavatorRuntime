import json
import sys
import threading
from pathlib import Path

import pytest


rclpy = pytest.importorskip("rclpy")

from airy_excavator_interfaces.action import ExcavationCycle
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.demo import load_demo_program
from mission.demo_runner import DemoCycleFailed, run_demo_cycles
from mission.runtime_ros.run_excavation_demo import ExcavationDemoClient
from mission.tests.test_demo_program import valid_demo_payload


class _CycleServer(Node):
    def __init__(self, *, context, fail_index=0):
        super().__init__("demo_cycle_fixture", context=context)
        self.target_ids = []
        self.fail_index = fail_index
        self.server = ActionServer(
            self,
            ExcavationCycle,
            "/mission/run_cycle",
            execute_callback=self._execute,
        )

    def _execute(self, handle):
        self.target_ids.append(handle.request.dig_target.target_id)
        result = ExcavationCycle.Result()
        result.quiescence_confirmed = True
        result.action_datagrams = 12
        if len(self.target_ids) == self.fail_index:
            result.outcome = ExcavationCycle.Result.OUTCOME_FAILED
            result.reason_code = "FIXTURE_FAILURE"
            result.message = "fixture failure"
            handle.abort()
        else:
            result.outcome = ExcavationCycle.Result.OUTCOME_SUCCEEDED
            result.reason_code = "SUCCEEDED"
            result.message = "ok"
            handle.succeed()
        return result

    def destroy_node(self):
        self.server.destroy()
        return super().destroy_node()


def _program(tmp_path):
    path = tmp_path / "demo.json"
    path.write_text(json.dumps(valid_demo_payload()), encoding="utf-8")
    return load_demo_program(path)


def _harness(fail_index=0):
    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = _CycleServer(context=context, fail_index=fail_index)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ExcavationDemoClient(
        wait_s=2.0,
        cycle_timeout_s=2.0,
        context=context,
    )
    return context, server, executor, thread, client


def _stop(harness):
    context, server, executor, thread, client = harness
    client.destroy_node()
    executor.shutdown(timeout_sec=1.0)
    thread.join(timeout=1.0)
    server.destroy_node()
    rclpy.shutdown(context=context)


def test_ros_client_submits_all_points_in_order(tmp_path):
    harness = _harness()
    _, server, _, _, client = harness
    program = _program(tmp_path)
    try:
        completed = run_demo_cycles(
            program,
            repeat=1,
            run_cycle=lambda point, index, count: client.run_cycle(
                program, point, index, count
            ),
        )
        assert completed == 3
        assert server.target_ids == [
            "field_demo_001:dig:dig_01",
            "field_demo_001:dig:dig_02",
            "field_demo_001:dig:dig_03",
        ]
    finally:
        _stop(harness)


def test_ros_client_does_not_submit_later_points_after_failure(tmp_path):
    harness = _harness(fail_index=2)
    _, server, _, _, client = harness
    program = _program(tmp_path)
    try:
        with pytest.raises(DemoCycleFailed, match="FIXTURE_FAILURE"):
            run_demo_cycles(
                program,
                repeat=1,
                run_cycle=lambda point, index, count: client.run_cycle(
                    program, point, index, count
                ),
            )
        assert server.target_ids == [
            "field_demo_001:dig:dig_01",
            "field_demo_001:dig:dig_02",
        ]
    finally:
        _stop(harness)
