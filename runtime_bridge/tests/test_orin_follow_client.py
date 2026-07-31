import socket
import threading
import time
import unittest

from runtime_bridge.orin_behavior_rpc import (
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
    receive_message,
    send_message,
)
from runtime_bridge.orin_follow_client import (
    FollowRejected,
    OrinFollowClient,
    OrinStatusMonitor,
)


class FakeBehaviorServer:
    def __init__(self, handler):
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(1)
        self._listener.settimeout(2.0)
        self.host, self.port = self._listener.getsockname()
        self.received = []
        self.error = None
        self._handler = handler
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        try:
            connection, _ = self._listener.accept()
            with connection:
                self._handler(connection, self)
        except BaseException as exc:
            self.error = exc
        finally:
            self._listener.close()

    def receive(self, connection):
        message = receive_message(connection)
        self.received.append(message)
        return message

    def close(self):
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise AssertionError("fake behavior server did not stop")
        if self.error is not None:
            raise self.error


class OrinFollowClientTest(unittest.TestCase):
    def test_start_fixed_action_streams_feedback_and_terminal_result(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "behavior": "ExecuteDump",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0},
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "feedback",
                    "seq": 1,
                    "step_index": 0,
                    "step_label": "open_bucket",
                    "phase": "running",
                    "max_error": 0.4,
                    "action_datagrams": 4,
                },
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 2,
                    "outcome": "SUCCEEDED",
                    "reason_code": "SEQUENCE_COMPLETED",
                    "message": "ExecuteDump completed",
                    "final_step_index": 1,
                    "final_step_label": "recover_bucket",
                    "final_max_error": 0.01,
                    "quiescence_confirmed": True,
                    "action_datagrams": 6,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(
            server.host,
            server.port,
            session_id="fixed-session",
        )
        feedback = []

        result = client.run_fixed_action(
            "ExecuteDump",
            feedback_callback=feedback.append,
        )

        self.assertEqual(
            server.received[0],
            {
                "schema_version": "orin_behavior_rpc.v1",
                "type": "start_fixed_action",
                "session_id": "fixed-session",
                "seq": 0,
                "request_id": server.received[0]["request_id"],
                "behavior": "ExecuteDump",
            },
        )
        self.assertEqual(feedback[0].step_label, "open_bucket")
        self.assertEqual(feedback[0].action_datagrams, 4)
        self.assertEqual(result.behavior, "ExecuteDump")
        self.assertEqual(result.outcome, "SUCCEEDED")
        self.assertEqual(result.reason_code, "SEQUENCE_COMPLETED")
        self.assertTrue(result.quiescence_confirmed)

    def test_fixed_action_early_failure_preserves_remote_reason(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "behavior": "ExecuteDump",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0},
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 1,
                    "outcome": "FAILED",
                    "reason_code": "MOTION_GATE_CLOSED",
                    "message": "state_stale",
                    "final_step_index": 0,
                    "final_step_label": "",
                    "final_max_error": 0.0,
                    "quiescence_confirmed": True,
                    "action_datagrams": 0,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        result = client.run_fixed_action("ExecuteDump")

        self.assertEqual(result.outcome, "FAILED")
        self.assertEqual(result.reason_code, "MOTION_GATE_CLOSED")
        self.assertEqual(result.message, "state_stale")
        self.assertEqual(result.final_step_label, "not_started")

    def test_fixed_action_success_requires_a_real_final_step_label(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "behavior": "ExecuteDump",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0},
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 1,
                    "outcome": "SUCCEEDED",
                    "reason_code": "SEQUENCE_COMPLETED",
                    "message": "ExecuteDump completed",
                    "final_step_index": 0,
                    "final_step_label": "",
                    "final_max_error": 0.0,
                    "quiescence_confirmed": True,
                    "action_datagrams": 0,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        with self.assertRaisesRegex(
            OrinBehaviorProtocolError,
            "final_step_label",
        ):
            client.run_fixed_action("ExecuteDump")

    def test_fixed_action_default_silence_budget_tolerates_one_second_network_gap(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "behavior": "ExecuteDig",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0},
            )
            time.sleep(1.1)
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 1,
                    "outcome": "SUCCEEDED",
                    "reason_code": "SEQUENCE_COMPLETED",
                    "message": "ExecuteDig completed",
                    "final_step_index": 2,
                    "final_step_label": "lift_boom",
                    "final_max_error": 0.0,
                    "quiescence_confirmed": True,
                    "action_datagrams": 12,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        result = client.run_fixed_action("ExecuteDig")

        self.assertEqual(result.outcome, "SUCCEEDED")
        self.assertEqual(result.final_step_label, "lift_boom")

    def test_silent_fixed_action_reports_last_received_event_context(self):
        release = threading.Event()

        def handler(connection, server):
            start = server.receive(connection)
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "accepted",
                    "session_id": start["session_id"],
                    "seq": 0,
                    "request_id": start["request_id"],
                    "behavior": "ExecuteDump",
                },
            )
            release.wait(1.0)

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        self.addCleanup(release.set)
        client = OrinFollowClient(
            server.host,
            server.port,
            stream_silence_timeout_s=0.1,
            poll_interval_s=0.01,
        )

        with self.assertRaises(OrinBehaviorConnectionError) as caught:
            client.run_fixed_action("ExecuteDump")

        diagnostic = str(caught.exception)
        self.assertIn("Orin fixed-action stream was silent", diagnostic)
        self.assertIn(f"endpoint={server.host}:{server.port}", diagnostic)
        self.assertIn(f"session_id={client.session_id}", diagnostic)
        self.assertIn("request_id=fixed-action-", diagnostic)
        self.assertIn("behavior=ExecuteDump", diagnostic)
        self.assertIn("accepted=true", diagnostic)
        self.assertIn("last_event_type=accepted", diagnostic)
        self.assertIn("last_event_seq=0", diagnostic)
        self.assertIn("events_received=1", diagnostic)

    def test_start_follow_streams_feedback_until_quiescent_result(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
            }
            send_message(
                connection,
                {
                    **common,
                    "type": "accepted",
                    "seq": 0,
                    "trajectory_id": "trajectory-001",
                },
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "feedback",
                    "seq": 1,
                    "trajectory_id": "trajectory-001",
                    "waypoint_index": 2,
                    "waypoint_count": 4,
                    "distance_m": 0.12,
                    "elapsed_s": 1.5,
                    "bucket_tip_stamp_s": 100.25,
                    "bucket_tip": [0.1, 0.2, 0.3],
                    "tracking_state": "ACTIVE",
                    "action_datagrams": 7,
                },
            )
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "status",
                    "session_id": "server",
                    "seq": 2,
                    "request_id": "status",
                    "state_fresh": True,
                    "control_enabled": True,
                    "sensor_valid": True,
                    "stm32_alive": True,
                    "estop": False,
                    "fault_free": True,
                    "quiescent": False,
                    "active_behavior": "Follow",
                    "action_datagrams": 7,
                    "motion_gate_reason": "active",
                    "sender_constructed": True,
                },
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 3,
                    "trajectory_id": "trajectory-001",
                    "outcome": "SUCCEEDED",
                    "reason_code": "SUCCEEDED",
                    "message": "trajectory completed",
                    "final_waypoint_index": 3,
                    "final_distance_m": 0.01,
                    "quiescence_confirmed": True,
                    "action_datagrams": 8,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(
            server.host,
            server.port,
            session_id="pc-gateway-session-001",
        )
        feedback = []
        statuses = []

        result = client.run_follow(
            {"trajectory_id": "trajectory-001"},
            feedback_callback=feedback.append,
            status_callback=statuses.append,
        )

        self.assertEqual(server.received[0]["type"], "start_follow")
        self.assertEqual(server.received[0]["schema_version"], "orin_behavior_rpc.v1")
        self.assertEqual(server.received[0]["session_id"], "pc-gateway-session-001")
        self.assertEqual(server.received[0]["seq"], 0)
        self.assertEqual(server.received[0]["trajectory"], {"trajectory_id": "trajectory-001"})
        self.assertEqual(feedback[0].waypoint_index, 2)
        self.assertEqual(feedback[0].bucket_tip, (0.1, 0.2, 0.3))
        self.assertEqual(feedback[0].action_datagrams, 7)
        self.assertEqual(statuses[0].active_behavior, "Follow")
        self.assertEqual(result.outcome, "SUCCEEDED")
        self.assertTrue(result.quiescence_confirmed)
        self.assertEqual(result.action_datagrams, 8)

    def test_rejected_start_surfaces_remote_reason(self):
        def handler(connection, server):
            start = server.receive(connection)
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "rejected",
                    "session_id": start["session_id"],
                    "seq": 0,
                    "request_id": start["request_id"],
                    "reason_code": "BUSY",
                    "message": "another behavior owns the lease",
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        with self.assertRaises(FollowRejected) as caught:
            client.run_follow({"trajectory_id": "trajectory-001"})

        self.assertEqual(caught.exception.reason_code, "BUSY")

    def test_cancel_preserves_ids_and_advances_request_sequence(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
            }
            send_message(
                connection,
                {
                    **common,
                    "type": "accepted",
                    "seq": 0,
                    "trajectory_id": "trajectory-001",
                },
            )
            cancel = server.receive(connection)
            self.assertEqual(cancel["type"], "cancel_follow")
            self.assertEqual(cancel["session_id"], start["session_id"])
            self.assertEqual(cancel["request_id"], start["request_id"])
            self.assertEqual(cancel["seq"], start["seq"] + 1)
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 1,
                    "trajectory_id": "trajectory-001",
                    "outcome": "CANCELLED",
                    "reason_code": "CANCELLED",
                    "message": "Follow cancelled",
                    "final_waypoint_index": 0,
                    "final_distance_m": 1.0,
                    "quiescence_confirmed": True,
                    "action_datagrams": 1,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        result = client.run_follow(
            {"trajectory_id": "trajectory-001"},
            cancel_requested=lambda: True,
        )

        self.assertEqual(result.outcome, "CANCELLED")
        self.assertTrue(result.quiescence_confirmed)

    def test_disconnect_before_result_is_an_explicit_connection_error(self):
        def handler(connection, server):
            server.receive(connection)

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        with self.assertRaisesRegex(
            OrinBehaviorConnectionError, "closed before a complete message"
        ):
            client.run_follow({"trajectory_id": "trajectory-001"})

    def test_silent_follow_stream_fails_instead_of_hanging_forever(self):
        release = threading.Event()

        def handler(connection, server):
            server.receive(connection)
            release.wait(1.0)

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        self.addCleanup(release.set)
        client = OrinFollowClient(
            server.host,
            server.port,
            stream_silence_timeout_s=0.1,
            poll_interval_s=0.01,
        )

        started_at = time.monotonic()
        with self.assertRaises(OrinBehaviorConnectionError) as caught:
            client.run_follow({"trajectory_id": "trajectory-001"})

        self.assertLess(time.monotonic() - started_at, 0.5)
        diagnostic = str(caught.exception)
        self.assertIn("Orin Follow stream was silent", diagnostic)
        self.assertIn(f"endpoint={server.host}:{server.port}", diagnostic)
        self.assertIn(f"session_id={client.session_id}", diagnostic)
        self.assertIn("request_id=follow-", diagnostic)
        self.assertIn("accepted=false", diagnostic)
        self.assertIn("last_event_type=none", diagnostic)
        self.assertIn("last_event_seq=-1", diagnostic)
        self.assertIn("events_received=0", diagnostic)

    def test_out_of_order_remote_event_is_rejected(self):
        def handler(connection, server):
            start = server.receive(connection)
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "accepted",
                    "session_id": start["session_id"],
                    "seq": 1,
                    "request_id": start["request_id"],
                    "trajectory_id": "trajectory-001",
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        with self.assertRaisesRegex(OrinBehaviorProtocolError, "event seq"):
            client.run_follow({"trajectory_id": "trajectory-001"})

    def test_remote_event_for_another_session_is_rejected(self):
        def handler(connection, server):
            start = server.receive(connection)
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "accepted",
                    "session_id": "different-session",
                    "seq": 0,
                    "request_id": start["request_id"],
                    "trajectory_id": "trajectory-001",
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinFollowClient(server.host, server.port)

        with self.assertRaisesRegex(OrinBehaviorProtocolError, "session_id"):
            client.run_follow({"trajectory_id": "trajectory-001"})

    def test_status_monitor_exposes_only_connected_fresh_remote_status(self):
        release = threading.Event()

        def handler(connection, _server):
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "status",
                    "session_id": "server",
                    "seq": 0,
                    "request_id": "status",
                    "state_fresh": True,
                    "control_enabled": True,
                    "sensor_valid": True,
                    "stm32_alive": True,
                    "estop": False,
                    "fault_free": True,
                    "quiescent": True,
                    "active_behavior": "",
                    "action_datagrams": 12,
                    "motion_gate_reason": "ready",
                    "sender_constructed": True,
                },
            )
            release.wait(2.0)

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        monitor = OrinStatusMonitor(
            server.host,
            server.port,
            reconnect_interval_s=0.02,
        )
        monitor.start()
        self.addCleanup(monitor.close)
        deadline = time.monotonic() + 1.0
        status = None
        while status is None and time.monotonic() < deadline:
            status = monitor.latest(max_age_s=0.5)
            time.sleep(0.01)

        self.assertIsNotNone(status)
        self.assertEqual(status.motion_gate_reason, "ready")
        self.assertEqual(status.action_datagrams, 12)
        self.assertTrue(status.sender_constructed)
        release.set()
        deadline = time.monotonic() + 1.0
        while monitor.latest(max_age_s=0.5) is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsNone(monitor.latest(max_age_s=0.5))


if __name__ == "__main__":
    unittest.main()
