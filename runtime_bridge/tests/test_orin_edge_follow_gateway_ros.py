import select
import socket
import threading
import time
import uuid
from dataclasses import replace

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import ExecuteDig, ExecuteDump, Follow
from airy_excavator_interfaces.msg import RuntimeStatus
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from runtime_bridge.apps.orin_edge_follow_gateway import (
    LIVE_MOTION_AUTHORIZATION,
    OrinEdgeFollowGatewayNode,
    _remote_goal_admissible,
    build_arg_parser,
)
from runtime_bridge.orin_behavior_rpc import receive_message, send_message
from runtime_bridge.orin_follow_client import OrinRuntimeStatus


class FakeOrinBehaviorServer:
    def __init__(
        self,
        *,
        wait_for_cancel=False,
        quiescence_confirmed=True,
        silent_after_accept=False,
    ):
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(4)
        self._listener.settimeout(0.1)
        self.host, self.port = self._listener.getsockname()
        self._stop = threading.Event()
        self._connections = []
        self._workers = []
        self.start_request = None
        self.cancel_request = None
        self.error = None
        self._closed = False
        self._wait_for_cancel = wait_for_cancel
        self._quiescence_confirmed = quiescence_confirmed
        self._silent_after_accept = silent_after_accept
        self._thread = threading.Thread(target=self._accept, daemon=True)
        self._thread.start()

    def _accept(self):
        try:
            while not self._stop.is_set():
                try:
                    connection, _ = self._listener.accept()
                except socket.timeout:
                    continue
                self._connections.append(connection)
                worker = threading.Thread(
                    target=self._serve_connection,
                    args=(connection,),
                    daemon=True,
                )
                self._workers.append(worker)
                worker.start()
        except OSError:
            pass
        except BaseException as exc:
            self.error = exc

    def _serve_connection(self, connection):
        try:
            sequence = 0
            send_message(connection, self._status(sequence))
            sequence += 1
            readable, _, _ = select.select([connection], [], [], 0.3)
            if readable:
                start = receive_message(connection)
                self.start_request = start
                common = {
                    "schema_version": "orin_behavior_rpc.v1",
                    "session_id": start["session_id"],
                    "request_id": start["request_id"],
                }
                if start["type"] == "start_fixed_action":
                    behavior = start["behavior"]
                    common["behavior"] = behavior
                    send_message(
                        connection,
                        {
                            **common,
                            "type": "accepted",
                            "seq": sequence,
                        },
                    )
                    sequence += 1
                    send_message(
                        connection,
                        {
                            **common,
                            "type": "feedback",
                            "seq": sequence,
                            "step_index": 0,
                            "step_label": "open_bucket",
                            "phase": "running",
                            "max_error": 0.4,
                            "action_datagrams": 5,
                        },
                    )
                    sequence += 1
                    send_message(
                        connection,
                        {
                            **common,
                            "type": "result",
                            "seq": sequence,
                            "outcome": "SUCCEEDED",
                            "reason_code": "SEQUENCE_COMPLETED",
                            "message": f"{behavior} completed",
                            "final_step_index": 1,
                            "final_step_label": "recover_bucket",
                            "final_max_error": 0.01,
                            "quiescence_confirmed": self._quiescence_confirmed,
                            "action_datagrams": 6,
                        },
                    )
                    return
                send_message(
                    connection,
                    {
                        **common,
                        "type": "accepted",
                        "seq": sequence,
                        "trajectory_id": start["trajectory"]["trajectory_id"],
                    },
                )
                sequence += 1
                if self._silent_after_accept:
                    self._stop.wait(2.0)
                    return
                if self._wait_for_cancel:
                    self.cancel_request = receive_message(connection)
                    send_message(
                        connection,
                        {
                            **common,
                            "type": "result",
                            "seq": sequence,
                            "trajectory_id": start["trajectory"]["trajectory_id"],
                            "outcome": "CANCELLED",
                            "reason_code": "CANCELLED",
                            "message": "Follow cancelled",
                            "final_waypoint_index": 0,
                            "final_distance_m": 0.2,
                            "quiescence_confirmed": True,
                            "action_datagrams": 1,
                        },
                    )
                    return
                send_message(
                    connection,
                    {
                        **common,
                        "type": "feedback",
                        "seq": sequence,
                        "trajectory_id": start["trajectory"]["trajectory_id"],
                        "waypoint_index": 0,
                        "waypoint_count": 1,
                        "distance_m": 0.02,
                        "elapsed_s": 0.4,
                        "bucket_tip_stamp_s": 123.25,
                        "bucket_tip": [0.5, 0.1, 0.2],
                        "tracking_state": "ACTIVE",
                        "action_datagrams": 5,
                    },
                )
                sequence += 1
                send_message(
                    connection,
                    {
                        **common,
                        "type": "result",
                        "seq": sequence,
                        "trajectory_id": start["trajectory"]["trajectory_id"],
                        "outcome": "SUCCEEDED",
                        "reason_code": "SUCCEEDED",
                        "message": "trajectory completed",
                        "final_waypoint_index": 0,
                        "final_distance_m": 0.01,
                        "quiescence_confirmed": self._quiescence_confirmed,
                        "action_datagrams": 6,
                    },
                )
                return
            while not self._stop.wait(0.05):
                send_message(connection, self._status(sequence))
                sequence += 1
        except (ConnectionError, OSError):
            pass
        except BaseException as exc:
            self.error = exc
        finally:
            connection.close()

    @staticmethod
    def _status(sequence):
        return {
            "schema_version": "orin_behavior_rpc.v1",
            "type": "status",
            "session_id": "server",
            "seq": sequence,
            "request_id": "status",
            "state_fresh": True,
            "control_enabled": True,
            "sensor_valid": True,
            "stm32_alive": True,
            "estop": False,
            "fault_free": True,
            "quiescent": True,
            "active_behavior": "",
            "action_datagrams": 0,
            "motion_gate_reason": "ready",
            "sender_constructed": True,
            "fixed_actions_available": True,
            "fixed_actions_validated": False,
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._listener.close()
        for connection in self._connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        self._thread.join(timeout=1.0)
        for worker in self._workers:
            worker.join(timeout=1.0)
        if self.error is not None:
            raise self.error


def _wait_future(future, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def _goal(node):
    now = node.get_clock().now()
    goal = Follow.Goal()
    snapshot = goal.trajectory
    snapshot.header.frame_id = "machine_root_ros"
    snapshot.header.stamp = now.to_msg()
    snapshot.trajectory_id = "orin-edge-integration-001"
    snapshot.trajectory_sha256 = "0" * 64
    snapshot.mission_id = "mission-001"
    snapshot.mission_sha256 = "b" * 64
    snapshot.mission_phase = "dig"
    snapshot.task_mode = "MoveToDig"
    snapshot.planning_scope = "execution_strict"
    snapshot.control_stage = "commissioning"
    snapshot.workspace_constraint = "disabled_by_operator"
    snapshot.execution_eligible = True
    snapshot.source_bucket_tip_stamp = now.to_msg()
    snapshot.source_local_map_stamp = now.to_msg()
    snapshot.inputs_frozen_at = now.to_msg()
    snapshot.valid_until = rclpy.time.Time(
        nanoseconds=now.nanoseconds + 5_000_000_000
    ).to_msg()
    snapshot.input_source = "live"
    snapshot.map_source = "live_local_map"
    snapshot.clock_mode = "ros_clock"
    snapshot.waypoints = [Point(x=0.5, y=0.1, z=0.2)]
    snapshot.waypoint_tolerance_m = 0.03
    snapshot.waypoint_dwell_s = 0.05
    snapshot.tracking_timeout_s = 2.0
    snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
    return goal


def _isolated_gateway(remote, context):
    namespace = f"/orin_edge_gateway_test_{uuid.uuid4().hex}"
    follow_action = namespace + "/excavator/follow"
    status_topic = namespace + "/mission/runtime_status"
    gateway = OrinEdgeFollowGatewayNode(
        orin_host=remote.host,
        orin_port=remote.port,
        motion_authorization=LIVE_MOTION_AUTHORIZATION,
        control_stage="commissioning",
        follow_action=follow_action,
        runtime_status_topic=status_topic,
        context=context,
    )
    return gateway, follow_action, status_topic


def _isolated_gateway_with_fixed_actions(remote, context):
    namespace = f"/orin_edge_fixed_gateway_test_{uuid.uuid4().hex}"
    follow_action = namespace + "/excavator/follow"
    dig_action = namespace + "/excavator/execute_dig"
    dump_action = namespace + "/excavator/execute_dump"
    status_topic = namespace + "/mission/runtime_status"
    gateway = OrinEdgeFollowGatewayNode(
        orin_host=remote.host,
        orin_port=remote.port,
        motion_authorization=LIVE_MOTION_AUTHORIZATION,
        control_stage="commissioning",
        follow_action=follow_action,
        execute_dig_action=dig_action,
        execute_dump_action=dump_action,
        runtime_status_topic=status_topic,
        context=context,
    )
    return gateway, dig_action, dump_action, status_topic


def test_gateway_cli_exposes_only_remote_endpoint_authorization_and_control_stage():
    args = build_arg_parser().parse_args(
        [
            "--orin-host",
            "192.0.2.10",
            "--orin-port",
            "19090",
            "--motion-authorization",
            LIVE_MOTION_AUTHORIZATION,
            "--control-stage",
            "commissioning",
        ]
    )

    assert set(vars(args)) == {
        "orin_host",
        "orin_port",
        "motion_authorization",
        "control_stage",
    }


def test_safe_stale_behavior_active_status_allows_next_goal_handoff():
    status = OrinRuntimeStatus(
        state_fresh=True,
        control_enabled=True,
        sensor_valid=True,
        stm32_alive=True,
        estop=False,
        fault_free=True,
        quiescent=False,
        active_behavior="Follow",
        action_datagrams=20,
        motion_gate_reason="behavior_active",
        sender_constructed=True,
        fixed_actions_available=True,
    )

    assert _remote_goal_admissible(status)
    assert not _remote_goal_admissible(replace(status, estop=True))
    assert not _remote_goal_admissible(replace(status, state_fresh=False))


def test_gateway_maps_remote_follow_and_publishes_edge_runtime_status():
    remote = FakeOrinBehaviorServer()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, follow_action, status_topic = _isolated_gateway(remote, context)
    client_node = rclpy.create_node("orin_gateway_integration_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    feedback = []
    client_node.create_subscription(
        RuntimeStatus, status_topic, statuses.append, 10
    )
    action_client = ActionClient(client_node, Follow, follow_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(
            item.motion_backend == "orin_edge"
            and item.follow_control_mode == "edge_onnx"
            and item.motion_gate_reason == "ready"
            for item in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        goal = _goal(client_node)
        handle = _wait_future(
            action_client.send_goal_async(
                goal,
                feedback_callback=lambda wrapped: feedback.append(wrapped.feedback),
            )
        )
        assert handle.accepted
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.reason_code == "SUCCEEDED"
        assert wrapped.result.quiescence_confirmed
        assert feedback[0].bucket_tip.x == pytest.approx(0.5)
        assert feedback[0].action_datagrams == 5
        assert remote.start_request["type"] == "start_follow"
        trajectory = remote.start_request["trajectory"]
        assert trajectory["trajectory_sha256"] == goal.trajectory.trajectory_sha256
        assert "action" not in trajectory
        assert "physical_velocity" not in trajectory
        status_count_before_disconnect = len(statuses)
        remote.close()
        deadline = time.monotonic() + 2.0
        while not any(
            item.motion_gate_reason == "orin_status_unavailable"
            and not item.state_fresh
            and not item.control_enabled
            and item.estop
            for item in statuses[status_count_before_disconnect:]
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()


def test_gateway_maps_existing_execute_dump_action_to_orin_fixed_behavior():
    remote = FakeOrinBehaviorServer()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, _dig_action, dump_action, status_topic = (
        _isolated_gateway_with_fixed_actions(remote, context)
    )
    client_node = rclpy.create_node(
        "orin_gateway_fixed_action_client",
        context=context,
    )
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    feedback = []
    client_node.create_subscription(
        RuntimeStatus,
        status_topic,
        statuses.append,
        10,
    )
    action_client = ActionClient(client_node, ExecuteDump, dump_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(
            item.motion_gate_reason == "ready"
            and item.motion_backend == "orin_edge"
            for item in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        handle = _wait_future(
            action_client.send_goal_async(
                ExecuteDump.Goal(),
                feedback_callback=lambda wrapped: feedback.append(wrapped.feedback),
            )
        )
        assert handle.accepted
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.outcome == ExecuteDump.Result.OUTCOME_SUCCEEDED
        assert wrapped.result.reason_code == "SEQUENCE_COMPLETED"
        assert wrapped.result.quiescence_confirmed
        assert feedback[0].step_label == "open_bucket"
        assert feedback[0].action_datagrams == 5
        assert remote.start_request["type"] == "start_fixed_action"
        assert remote.start_request["behavior"] == "ExecuteDump"
        assert set(remote.start_request) == {
            "schema_version",
            "type",
            "session_id",
            "seq",
            "request_id",
            "behavior",
        }
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()


def test_gateway_rejects_non_live_snapshot_before_opening_remote_follow():
    remote = FakeOrinBehaviorServer()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, follow_action, status_topic = _isolated_gateway(remote, context)
    client_node = rclpy.create_node("orin_gateway_invalid_goal_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    client_node.create_subscription(
        RuntimeStatus, status_topic, statuses.append, 10
    )
    action_client = ActionClient(client_node, Follow, follow_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(item.motion_gate_reason == "ready" for item in statuses):
            assert time.monotonic() < deadline
            time.sleep(0.01)
        goal = _goal(client_node)
        goal.trajectory.input_source = "fixture"
        goal.trajectory.map_source = "fixture_empty"
        goal.trajectory.trajectory_sha256 = trajectory_snapshot_message_sha256(
            goal.trajectory
        )

        handle = _wait_future(action_client.send_goal_async(goal))

        assert not handle.accepted
        assert remote.start_request is None
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()


def test_gateway_maps_ros_cancel_to_remote_cancel_and_waits_for_quiescent_result():
    remote = FakeOrinBehaviorServer(wait_for_cancel=True)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, follow_action, status_topic = _isolated_gateway(remote, context)
    client_node = rclpy.create_node("orin_gateway_cancel_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    client_node.create_subscription(
        RuntimeStatus, status_topic, statuses.append, 10
    )
    action_client = ActionClient(client_node, Follow, follow_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(item.motion_gate_reason == "ready" for item in statuses):
            assert time.monotonic() < deadline
            time.sleep(0.01)
        handle = _wait_future(action_client.send_goal_async(_goal(client_node)))
        assert handle.accepted
        deadline = time.monotonic() + 2.0
        while not any(
            item.active_behavior == "Follow"
            and item.motion_gate_reason == "behavior_active"
            and not item.quiescent
            for item in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        cancel = _wait_future(handle.cancel_goal_async())
        assert cancel.goals_canceling
        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert wrapped.result.outcome == Follow.Result.OUTCOME_CANCELLED
        assert wrapped.result.quiescence_confirmed
        assert remote.cancel_request["type"] == "cancel_follow"
        assert remote.cancel_request["session_id"] == remote.start_request["session_id"]
        assert remote.cancel_request["request_id"] == remote.start_request["request_id"]
        assert remote.cancel_request["seq"] == remote.start_request["seq"] + 1
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()


def test_gateway_never_succeeds_when_remote_quiescence_is_unconfirmed():
    remote = FakeOrinBehaviorServer(quiescence_confirmed=False)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, follow_action, status_topic = _isolated_gateway(remote, context)
    client_node = rclpy.create_node("orin_gateway_quiescence_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    client_node.create_subscription(
        RuntimeStatus, status_topic, statuses.append, 10
    )
    action_client = ActionClient(client_node, Follow, follow_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(item.motion_gate_reason == "ready" for item in statuses):
            assert time.monotonic() < deadline
            time.sleep(0.01)
        handle = _wait_future(action_client.send_goal_async(_goal(client_node)))
        assert handle.accepted

        wrapped = _wait_future(handle.get_result_async())

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.outcome == Follow.Result.OUTCOME_FAILED
        assert wrapped.result.reason_code == "ORIN_QUIESCENCE_UNCONFIRMED"
        assert not wrapped.result.quiescence_confirmed
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()


def test_gateway_aborts_silent_follow_and_releases_owned_operation():
    remote = FakeOrinBehaviorServer(silent_after_accept=True)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    gateway, follow_action, status_topic = _isolated_gateway(remote, context)
    client_node = rclpy.create_node("orin_gateway_silent_follow_client", context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(gateway)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    statuses = []
    client_node.create_subscription(
        RuntimeStatus, status_topic, statuses.append, 10
    )
    action_client = ActionClient(client_node, Follow, follow_action)
    try:
        assert action_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while not any(item.motion_gate_reason == "ready" for item in statuses):
            assert time.monotonic() < deadline
            time.sleep(0.01)

        handle = _wait_future(action_client.send_goal_async(_goal(client_node)))
        assert handle.accepted
        wrapped = _wait_future(handle.get_result_async(), timeout_s=5.0)

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason_code == "ORIN_RPC_ERROR"
        assert not wrapped.result.quiescence_confirmed
        deadline = time.monotonic() + 1.0
        while not any(
            item.active_behavior == "" and item.quiescent
            for item in statuses
        ):
            assert time.monotonic() < deadline
            time.sleep(0.01)
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        gateway.destroy_node()
        rclpy.shutdown(context=context)
        remote.close()
