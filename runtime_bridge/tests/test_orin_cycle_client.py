import socket
import threading
import unittest

from runtime_bridge.orin_behavior_rpc import receive_message, send_message
from runtime_bridge.orin_cycle_client import (
    CycleLegFeedback,
    CycleLegRejected,
    OrinCycleClient,
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


class OrinCycleClientTest(unittest.TestCase):
    def test_dig_leg_waits_through_execute_dig(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "cycle_id": "cycle-1",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0, "stage": "FOLLOW_DIG"},
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "feedback",
                    "seq": 1,
                    "stage": "EXECUTE_DIG",
                    "behavior": "ExecuteDig",
                    "message": "local behavior running",
                    "action_datagrams": 8,
                },
            )
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 2,
                    "outcome": "SUCCEEDED",
                    "reason_code": "DIG_LEG_COMPLETED",
                    "message": "FollowDig and ExecuteDig completed locally",
                    "completed_stage": "EXECUTE_DIG",
                    "quiescence_confirmed": True,
                    "action_datagrams": 12,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinCycleClient(
            server.host,
            server.port,
            session_id="cycle-session",
        )
        feedback: list[CycleLegFeedback] = []

        result = client.run_cycle_dig_leg(
            "cycle-1",
            {"trajectory_id": "dig-trajectory"},
            feedback_callback=feedback.append,
        )

        self.assertEqual(server.received[0]["type"], "start_cycle")
        self.assertEqual(server.received[0]["seq"], 0)
        self.assertEqual(server.received[0]["cycle_id"], "cycle-1")
        self.assertEqual(feedback[0].stage, "EXECUTE_DIG")
        self.assertEqual(feedback[0].behavior, "ExecuteDig")
        self.assertEqual(result.reason_code, "DIG_LEG_COMPLETED")
        self.assertEqual(result.completed_stage, "EXECUTE_DIG")
        self.assertTrue(result.quiescence_confirmed)
        self.assertEqual(result.action_datagrams, 12)

    def test_rejection_preserves_orin_motion_gate_reason(self):
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
                    "cycle_id": "cycle-1",
                    "reason_code": "MOTION_NOT_READY",
                    "message": "state_stale",
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinCycleClient(server.host, server.port)

        with self.assertRaises(CycleLegRejected) as caught:
            client.run_cycle_dig_leg(
                "cycle-1",
                {"trajectory_id": "dig-trajectory"},
            )

        self.assertEqual(caught.exception.reason_code, "MOTION_NOT_READY")
        self.assertEqual(caught.exception.message, "state_stale")

    def test_bad_request_rejection_does_not_require_cycle_id(self):
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
                    "reason_code": "BAD_REQUEST",
                    "message": "cycle request fields are invalid",
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinCycleClient(server.host, server.port)

        with self.assertRaises(CycleLegRejected) as caught:
            client.run_cycle_dig_leg(
                "cycle-1",
                {"trajectory_id": "dig-trajectory"},
            )

        self.assertEqual(caught.exception.reason_code, "BAD_REQUEST")
        self.assertEqual(
            caught.exception.message,
            "cycle request fields are invalid",
        )

    def test_active_leg_cancel_uses_same_request_identity_and_next_sequence(self):
        def handler(connection, server):
            start = server.receive(connection)
            common = {
                "schema_version": "orin_behavior_rpc.v1",
                "session_id": start["session_id"],
                "request_id": start["request_id"],
                "cycle_id": "cycle-1",
            }
            send_message(
                connection,
                {**common, "type": "accepted", "seq": 0, "stage": "FOLLOW_DIG"},
            )
            cancel = server.receive(connection)
            send_message(
                connection,
                {
                    **common,
                    "type": "result",
                    "seq": 1,
                    "outcome": "CANCELLED",
                    "reason_code": "CANCELLED",
                    "message": "excavation cycle stopped",
                    "completed_stage": "FOLLOW_DIG",
                    "quiescence_confirmed": True,
                    "action_datagrams": 1,
                },
            )
            self.assertEqual(cancel["type"], "cancel_cycle")
            self.assertEqual(cancel["seq"], 1)
            self.assertEqual(cancel["request_id"], start["request_id"])

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinCycleClient(server.host, server.port)

        result = client.run_cycle_dig_leg(
            "cycle-1",
            {"trajectory_id": "dig-trajectory"},
            cancel_requested=lambda: True,
        )

        self.assertEqual(result.outcome, "CANCELLED")
        self.assertTrue(result.quiescence_confirmed)

    def test_cancel_waiting_cycle_uses_a_standalone_rpc(self):
        def handler(connection, server):
            request = server.receive(connection)
            send_message(
                connection,
                {
                    "schema_version": "orin_behavior_rpc.v1",
                    "type": "result",
                    "session_id": request["session_id"],
                    "seq": 0,
                    "request_id": request["request_id"],
                    "cycle_id": "cycle-1",
                    "outcome": "CANCELLED",
                    "reason_code": "CANCELLED",
                    "message": "excavation cycle cancelled while quiescent",
                    "completed_stage": "CANCELLED",
                    "quiescence_confirmed": True,
                    "action_datagrams": 5,
                },
            )

        server = FakeBehaviorServer(handler)
        self.addCleanup(server.close)
        client = OrinCycleClient(server.host, server.port)

        result = client.cancel_cycle("cycle-1")

        self.assertEqual(server.received[0]["type"], "cancel_cycle")
        self.assertEqual(server.received[0]["cycle_id"], "cycle-1")
        self.assertEqual(result.outcome, "CANCELLED")
        self.assertTrue(result.quiescence_confirmed)


if __name__ == "__main__":
    unittest.main()
