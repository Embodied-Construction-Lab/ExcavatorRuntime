#!/usr/bin/python3
"""Run ordered FollowDig→Dig→FollowDump→Dump cycles from one demo program."""

from __future__ import annotations

import argparse
import sys

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from airy_excavator_interfaces.action import ExcavationCycle
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from mission.demo import (
    DemoDigPoint,
    DemoProgramError,
    ExcavationDemoProgram,
    load_demo_program,
)
from mission.demo_runner import DemoCycleFailed, run_demo_cycles


class ExcavationDemoClient(Node):
    def __init__(
        self,
        *,
        wait_s: float,
        cycle_timeout_s: float,
        context=None,
    ) -> None:
        super().__init__("excavation_demo_client", context=context)
        if wait_s <= 0.0 or cycle_timeout_s <= 0.0:
            raise ValueError("wait_s和cycle_timeout_s必须大于0")
        self._wait_s = wait_s
        self._cycle_timeout_s = cycle_timeout_s
        self._client = ActionClient(self, ExcavationCycle, "/mission/run_cycle")
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)

    def run_cycle(
        self,
        program: ExcavationDemoProgram,
        point: DemoDigPoint,
        cycle_index: int,
        cycle_count: int,
    ) -> None:
        if not self._client.wait_for_server(timeout_sec=self._wait_s):
            raise DemoCycleFailed(point.point_id, "ACTION_SERVER_UNAVAILABLE")
        print(
            f"[cycle {cycle_index}/{cycle_count}] start dig_point={point.point_id} "
            f"position_m={list(point.target.position_m)}",
            flush=True,
        )
        send_future = self._client.send_goal_async(
            _build_cycle_goal(self, program, point),
            feedback_callback=lambda message: _print_feedback(
                point.point_id, message
            ),
        )
        handle = self._wait(send_future, self._wait_s, point, "goal response")
        if handle is None or not handle.accepted:
            raise DemoCycleFailed(point.point_id, "GOAL_REJECTED")
        result_future = handle.get_result_async()
        try:
            wrapped = self._wait(
                result_future,
                self._cycle_timeout_s,
                point,
                "cycle result",
            )
        except (DemoCycleFailed, KeyboardInterrupt):
            self._cancel(handle, result_future, point)
            raise
        result = wrapped.result
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or result.outcome != ExcavationCycle.Result.OUTCOME_SUCCEEDED
            or result.reason_code != "SUCCEEDED"
            or not result.quiescence_confirmed
        ):
            raise DemoCycleFailed(
                point.point_id,
                f"{result.reason_code or 'CYCLE_FAILED'}: {result.message}",
            )
        print(
            f"[cycle {cycle_index}/{cycle_count}] completed "
            f"dig_point={point.point_id} datagrams={result.action_datagrams}",
            flush=True,
        )

    def _wait(self, future, timeout_s, point, operation):
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_s)
        if not future.done():
            raise DemoCycleFailed(point.point_id, f"{operation} timed out")
        if future.exception() is not None or future.result() is None:
            raise DemoCycleFailed(point.point_id, f"{operation} failed")
        return future.result()

    def _cancel(self, handle, result_future, point) -> None:
        cancel_future = handle.cancel_goal_async()
        self._executor.spin_until_future_complete(cancel_future, timeout_sec=self._wait_s)
        self._executor.spin_until_future_complete(result_future, timeout_sec=self._wait_s)
        if not result_future.done():
            raise DemoCycleFailed(
                point.point_id,
                "cycle timed out and cancellation did not reach a terminal result",
            )

    def destroy_node(self):
        self._executor.remove_node(self)
        self._executor.shutdown(timeout_sec=1.0)
        return super().destroy_node()


def _build_cycle_goal(
    node: Node,
    program: ExcavationDemoProgram,
    point: DemoDigPoint,
) -> ExcavationCycle.Goal:
    goal = ExcavationCycle.Goal()
    _fill_target(
        node,
        goal.dig_target,
        program,
        kind="dig",
        target_id=f"{program.demo_id}:dig:{point.point_id}",
        target=point.target,
    )
    _fill_target(
        node,
        goal.dump_target,
        program,
        kind="dump",
        target_id=f"{program.demo_id}:dump",
        target=program.dump_target,
    )
    return goal


def _fill_target(node, message, program, *, kind, target_id, target) -> None:
    message.header.frame_id = program.frame_id
    message.header.stamp = node.get_clock().now().to_msg()
    message.target_id = target_id
    message.target_kind = kind
    message.target_status = program.target_status
    message.mission_id = program.demo_id
    message.mission_sha256 = program.sha256
    message.mission_phase = kind
    message.position = Point(
        x=target.position_m[0],
        y=target.position_m[1],
        z=target.position_m[2],
    )
    message.normal = Vector3(
        x=target.normal[0],
        y=target.normal[1],
        z=target.normal[2],
    )
    message.radius_m = target.radius_m


def _print_feedback(point_id: str, message) -> None:
    feedback = message.feedback
    print(
        f"[{point_id}] stage={feedback.stage} message={feedback.message} "
        f"datagrams={feedback.action_datagrams}",
        flush=True,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    default_program = (
        get_package_share_directory("airy_mission_runtime")
        + "/config/excavation_demo.json"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Execute FollowDig→Dig→FollowDump→Dump for every configured dig point."
        )
    )
    parser.add_argument("--program", default=default_program)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--wait-s", type=float, default=5.0)
    parser.add_argument("--cycle-timeout-s", type=float, default=300.0)
    return parser


def run() -> int:
    args = build_arg_parser().parse_args()
    try:
        program = load_demo_program(args.program)
    except (DemoProgramError, ValueError) as exc:
        print(f"demo configuration failed: {exc}", file=sys.stderr)
        return 2
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    client = ExcavationDemoClient(
        wait_s=args.wait_s,
        cycle_timeout_s=args.cycle_timeout_s,
    )
    try:
        completed = run_demo_cycles(
            program,
            repeat=args.repeat,
            run_cycle=lambda point, index, count: client.run_cycle(
                program, point, index, count
            ),
        )
        print(f"demo completed: cycles={completed}", flush=True)
        return 0
    except (DemoCycleFailed, ValueError) as exc:
        print(f"demo stopped: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("demo cancelled by operator", file=sys.stderr)
        return 130
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
