import json
import socket
import struct
import unittest
from types import SimpleNamespace

from runtime_bridge.orin_behavior_rpc import (
    MAX_MESSAGE_BYTES,
    OrinBehaviorProtocolError,
    receive_message,
    send_message,
    trajectory_snapshot_to_message,
)


class OrinBehaviorRpcTest(unittest.TestCase):
    def test_receive_message_reassembles_a_fragmented_length_prefixed_json_frame(self):
        reader, writer = socket.socketpair()
        self.addCleanup(reader.close)
        self.addCleanup(writer.close)
        message = {
            "schema_version": "orin_behavior_rpc.v1",
            "type": "accepted",
            "session_id": "session-001",
            "seq": 0,
            "request_id": "request-001",
            "trajectory_id": "trajectory-001",
        }
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        frame = struct.pack(">I", len(payload)) + payload

        for byte in frame:
            writer.sendall(bytes([byte]))

        self.assertEqual(receive_message(reader), message)

    def test_send_message_uses_big_endian_length_and_rejects_oversized_json(self):
        reader, writer = socket.socketpair()
        self.addCleanup(reader.close)
        self.addCleanup(writer.close)
        message = {"schema_version": "orin_behavior_rpc.v1", "type": "ping"}

        send_message(writer, message)

        (length,) = struct.unpack(">I", reader.recv(4))
        self.assertEqual(json.loads(reader.recv(length).decode("utf-8")), message)
        with self.assertRaisesRegex(OrinBehaviorProtocolError, "too large"):
            send_message(writer, {"payload": "x" * MAX_MESSAGE_BYTES})

    def test_trajectory_snapshot_serializes_every_canonical_field_without_velocity(self):
        stamp = lambda sec, nanosec: SimpleNamespace(sec=sec, nanosec=nanosec)
        snapshot = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="machine_root_ros", stamp=stamp(100, 250_000_000)
            ),
            trajectory_id="trajectory-001",
            trajectory_sha256="a" * 64,
            mission_id="mission-001",
            mission_sha256="b" * 64,
            mission_phase="dig",
            task_mode="MoveToDig",
            planning_scope="execution_strict",
            control_stage="production",
            workspace_constraint="mission_workspace",
            execution_eligible=True,
            source_bucket_tip_stamp=stamp(99, 100_000_000),
            source_local_map_stamp=stamp(99, 200_000_000),
            inputs_frozen_at=stamp(99, 300_000_000),
            valid_until=stamp(110, 0),
            input_source="live",
            map_source="local_map",
            clock_mode="system",
            waypoints=[
                SimpleNamespace(x=1.0, y=2.0, z=3.0),
                SimpleNamespace(x=4.0, y=5.0, z=6.0),
            ],
            waypoint_tolerance_m=0.03,
            waypoint_dwell_s=0.2,
            tracking_timeout_s=12.0,
        )

        message = trajectory_snapshot_to_message(snapshot)

        self.assertEqual(
            set(message),
            {
                "trajectory_id",
                "trajectory_sha256",
                "frame_id",
                "created_at_s",
                "mission_id",
                "mission_sha256",
                "mission_phase",
                "task_mode",
                "planning_scope",
                "control_stage",
                "workspace_constraint",
                "execution_eligible",
                "source_bucket_tip_stamp_s",
                "source_local_map_stamp_s",
                "inputs_frozen_at_s",
                "valid_until_s",
                "input_source",
                "map_source",
                "clock_mode",
                "waypoints",
                "waypoint_tolerance_m",
                "waypoint_dwell_s",
                "tracking_timeout_s",
            },
        )
        self.assertEqual(message["created_at_s"], 100.25)
        self.assertEqual(message["waypoints"], [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        self.assertNotIn("action", message)
        self.assertNotIn("physical_velocity", message)


if __name__ == "__main__":
    unittest.main()
