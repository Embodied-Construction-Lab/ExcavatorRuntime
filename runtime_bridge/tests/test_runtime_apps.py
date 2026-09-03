import os
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import mock

from runtime_bridge.apps.inspect_orin_packets import extract_machine_state_packets, format_machine_state_packet
from runtime_bridge.apps.mock_orin_relay import build_arg_parser as build_mock_orin_parser
from runtime_bridge.apps.pc_runtime_bridge import (
    JointStatePublisher,
    build_arg_parser,
    should_print_state,
    write_latest_state,
)
from runtime_bridge.ros_provenance import epoch_ms_to_ros_time_fields
from runtime_bridge.runtime_config import load_runtime_config


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class RuntimeAppsTest(unittest.TestCase):
    def test_latest_state_snapshot_is_published_with_atomic_replace(self):
        class FakeState:
            def to_dict(self):
                return {"type": "machine_state_v1", "seq": 7}

        with self.subTest("atomic snapshot"), mock.patch(
            "runtime_bridge.apps.pc_runtime_bridge.os.replace",
            wraps=os.replace,
        ) as replace:
            with TemporaryDirectory() as directory:
                path = Path(directory) / "latest_state.json"
                write_latest_state(path, FakeState())

                self.assertEqual(
                    path.read_text(encoding="utf-8"),
                    '{\n  "type": "machine_state_v1",\n  "seq": 7\n}\n',
                )
                self.assertEqual(replace.call_count, 1)
                self.assertEqual(replace.call_args.args[1], path)

    def test_joint_state_publisher_close_is_idempotent_after_ros_signal_shutdown(self):
        class FakeNode:
            destroyed = False

            def destroy_node(self):
                self.destroyed = True

        class AlreadyShutdownRclpy:
            def ok(self):
                return False

            def shutdown(self):
                raise AssertionError("shutdown must not be called twice")

        publisher = object.__new__(JointStatePublisher)
        publisher.node = FakeNode()
        publisher.rclpy = AlreadyShutdownRclpy()

        publisher.close()

        self.assertTrue(publisher.node.destroyed)

    def test_diagnostic_bridge_accepts_packet_print_interval_override(self):
        args = build_arg_parser().parse_args(
            ["--print-every", "100", "--write-every", "1"]
        )

        self.assertEqual(args.print_every, 100)
        self.assertEqual(args.write_every, 1)

    def test_diagnostic_bridge_allows_zero_but_rejects_negative_print_interval(self):
        parser = build_arg_parser()

        self.assertEqual(parser.parse_args(["--print-every", "0"]).print_every, 0)
        with self.assertRaises(SystemExit):
            parser.parse_args(["--print-every", "-1"])

    def test_diagnostic_bridge_prints_only_on_requested_packet_boundaries(self):
        self.assertFalse(should_print_state(9, 10))
        self.assertTrue(should_print_state(10, 10))
        self.assertTrue(should_print_state(100, 100))
        self.assertFalse(should_print_state(100, 0))

    def test_diagnostic_bridge_exposes_only_runtime_intent(self):
        parser = build_arg_parser()
        defaults = parser.parse_args([])

        self.assertEqual(
            set(vars(defaults)),
            {"config", "publish_joint_states", "print_every", "write_every"},
        )
        self.assertIsNone(defaults.print_every)
        self.assertIsNone(defaults.write_every)

    def test_mock_orin_relay_is_state_only(self):
        defaults = build_mock_orin_parser().parse_args([])

        self.assertEqual(
            set(vars(defaults)),
            {"pc_host", "state_port", "rate_hz"},
        )

    def test_shipped_mock_config_uses_loopback_network(self):
        config = load_runtime_config(
            PROJECT_ROOT / "runtime_bridge" / "config" / "runtime.mock.json"
        )

        self.assertEqual(config.network.state_endpoint, ("127.0.0.1", 18081))

    def test_packet_inspector_extracts_and_formats_machine_state_from_tcpdump_text(self):
        capture = (
            "14:00:01 IP 192.168.2.88.18081 > 192.168.2.127.18081: UDP, length 42\n"
            '{"type":"machine_state_v1","schema_version":"1.0","seq":63,"stamp_ms":1783666906735,'
            '"source":"orin","machine_id":"scale_excavator_v1","stm32_stamp_ms":850104,'
            '"safety":{"estop":false,"stm32_alive":true,"sensor_valid":true,"control_enabled":true,"fault_flags":[]},'
            '"actuator_state":{"boom":{"position_m":0.13103,"velocity_mps":0.0},"stick":{"position_m":0.15075,"velocity_mps":0.0},"bucket":{"position_m":0.05643,"velocity_mps":0.0},"swing":{"position_rad":0.2870717553680273,"velocity_rad_s":0.001780235837034216}},'
            '"joint_state":{"position_rad":{"swing":0.2870717553680273,"boom":0.7749261878854823,"arm":1.8750072154175084,"bucket":3.815289744859604},"velocity_rad_s":{"swing":0.001780235837034216,"boom":0.0,"arm":0.0,"bucket":0.0}}}\n'
        )

        packets = extract_machine_state_packets(capture)

        self.assertEqual(len(packets), 1)
        self.assertIn("seq=63", format_machine_state_packet(packets[0]))
        self.assertIn("boom: pos=0.13103 m, vel=0.00000 m/s", format_machine_state_packet(packets[0]))
        self.assertIn("safety: estop=False, stm32_alive=True, sensor_valid=True, control_enabled=True", format_machine_state_packet(packets[0]))

    def test_orin_epoch_stamp_converts_to_ros_header_time_without_pc_arrival_time(self):
        self.assertEqual(
            epoch_ms_to_ros_time_fields(1783666906735),
            (1783666906, 735000000),
        )


if __name__ == "__main__":
    unittest.main()
