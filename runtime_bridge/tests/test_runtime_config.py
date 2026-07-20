import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from runtime_bridge.runtime_config import load_runtime_config


def valid_config_payload():
    return {
        "schema": "runtime_bridge_config_v10",
        "network": {
            "state_bind_host": "0.0.0.0",
            "state_port": 18081,
            "orin_host": "192.168.2.88",
            "action_port": 18082,
            "action_valid_ms": 100,
            "action_time_source": "orin",
        },
        "artifacts": {
            "onnx": "model.onnx",
            "machine_profile": "machine_profile.json",
            "fixed_action_profile": "fixed_actions.json",
            "urdf": "machine.urdf",
            "waypoint_slice": "waypoint_slice.json",
            "latest_observation": "exports/latest_observation.json",
        },
        "action_journal": {
            "directory": "exports/action_journal",
            "max_file_bytes": 67108864,
            "retained_files": 16,
        },
        "policy": {
            "bucket_tip_timeout_ms": 500,
            "machine_state_timeout_ms": 500,
        },
        "fixed_action": {
            "expected_profile_sha256": "a" * 64,
        },
        "manual_jog": {
            "enabled": True,
            "allowed_actuators": ["boom", "stick", "bucket"],
            "speed_fraction": 0.1,
            "command_period_ms": 50,
            "heartbeat_timeout_ms": 175,
            "max_hold_ms": 3000,
        },
        "follow_control": {
            "mode": "supervised_canary",
            "allowed_actuators": ["boom", "stick", "bucket", "swing"],
            "heartbeat_timeout_ms": 175,
        },
        "diagnostics": {"print_every": 1, "write_every": 5},
    }


class RuntimeConfigTest(unittest.TestCase):
    def test_shipped_fixed_action_digest_matches_fixed_action_template(self):
        runtime_root = Path(__file__).resolve().parents[1]
        for config_name in ("runtime.json", "runtime.mock.json"):
            with self.subTest(config_name=config_name):
                config_path = runtime_root / "config" / config_name
                config = load_runtime_config(config_path, project_root=runtime_root.parent)

                actual_digest = hashlib.sha256(
                    config.artifacts.fixed_action_profile.read_bytes()
                ).hexdigest()

                self.assertEqual(config.fixed_action.expected_profile_sha256, actual_digest)

    def test_live_manual_jog_is_hard_limited_to_one_second_field_probe(self):
        live_config = load_runtime_config(
            Path(__file__).resolve().parents[1] / "config/runtime.json"
        )

        self.assertEqual(live_config.manual_jog.max_hold_ms, 1000)

    def test_resolves_local_action_journal_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            config = load_runtime_config(config_path, project_root=project_root)

        self.assertEqual(
            config.action_journal.directory,
            project_root / "exports/action_journal",
        )
        self.assertEqual(config.action_journal.max_file_bytes, 67108864)
        self.assertEqual(config.action_journal.retained_files, 16)

    def test_loads_runtime_settings_and_resolves_artifact_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_path = project_root / "runtime.json"
            config_path.write_text(
                json.dumps(valid_config_payload()),
                encoding="utf-8",
            )

            config = load_runtime_config(config_path, project_root=project_root)

        self.assertEqual(config.network.state_endpoint, ("0.0.0.0", 18081))
        self.assertEqual(config.network.action_endpoint, ("192.168.2.88", 18082))
        self.assertEqual(config.network.action_valid_ms, 100)
        self.assertEqual(config.network.action_time_source, "orin")
        self.assertEqual(config.artifacts.onnx, project_root / "model.onnx")
        self.assertEqual(config.artifacts.latest_observation, project_root / "exports/latest_observation.json")
        self.assertEqual(config.policy.bucket_tip_timeout_ms, 500)
        self.assertEqual(config.policy.machine_state_timeout_ms, 500)
        self.assertEqual(config.fixed_action.expected_profile_sha256, "a" * 64)
        self.assertTrue(config.manual_jog.enabled)
        self.assertEqual(config.manual_jog.allowed_actuators, ("boom", "stick", "bucket"))
        self.assertEqual(config.manual_jog.speed_fraction, 0.1)
        self.assertEqual(config.manual_jog.heartbeat_timeout_ms, 175)
        self.assertEqual(config.follow_control.mode, "supervised_canary")
        self.assertEqual(
            config.follow_control.allowed_actuators,
            ("boom", "stick", "bucket", "swing"),
        )
        self.assertEqual(config.follow_control.heartbeat_timeout_ms, 175)
        self.assertEqual(config.artifacts.fixed_action_profile, project_root / "fixed_actions.json")
        self.assertEqual(config.artifacts.urdf, project_root / "machine.urdf")
        self.assertEqual(config.diagnostics.write_every, 5)

    def test_rejects_unknown_fields_instead_of_silently_ignoring_them(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            config_data["network"]["state_poort"] = 18081
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.*未知字段"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_unsupported_config_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            config_data["schema"] = "runtime_bridge_config_v0"
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "schema"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_v5_config_after_follow_canary_contract_was_added(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_data = valid_config_payload()
            config_data["schema"] = "runtime_bridge_config_v5"
            del config_data["follow_control"]
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "schema.*runtime_bridge_config_v10"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_boolean_or_out_of_range_network_numbers(self):
        invalid_values = (True, 0, 65536)
        for invalid_value in invalid_values:
            with self.subTest(state_port=invalid_value), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                config_data = valid_config_payload()
                config_data["network"]["state_port"] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, "network.state_port"):
                    load_runtime_config(config_path, project_root=project_root)

    def test_reports_missing_required_field(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_data = valid_config_payload()
            del config_data["network"]["action_port"]
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.*缺少字段.*action_port"):
                load_runtime_config(config_path, project_root=project_root)

    def test_policy_artifacts_are_checked_only_when_policy_runtime_requests_them(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(valid_config_payload()), encoding="utf-8")

            config = load_runtime_config(config_path, project_root=project_root)

            with self.assertRaisesRegex(ValueError, "artifacts.onnx.*不存在"):
                config.artifacts.require_policy_inputs()

    def test_fixed_action_requires_only_machine_profile_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            (project_root / "machine_profile.json").touch()
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(valid_config_payload()), encoding="utf-8")

            config = load_runtime_config(config_path, project_root=project_root)

            config.artifacts.require_machine_profile()

    def test_rejects_unknown_action_time_source(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                (project_root / relative_path).touch()
            config_data = valid_config_payload()
            config_data["network"]["action_time_source"] = "automatic"
            config_path = project_root / "runtime.json"
            config_path.write_text(json.dumps(config_data), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "network.action_time_source"):
                load_runtime_config(config_path, project_root=project_root)

    def test_rejects_invalid_timeout_and_diagnostic_intervals(self):
        invalid_cases = (
            (("network", "action_valid_ms"), True, "network.action_valid_ms"),
            (("policy", "bucket_tip_timeout_ms"), 0, "policy.bucket_tip_timeout_ms"),
            (("policy", "machine_state_timeout_ms"), 99, "policy.machine_state_timeout_ms"),
            (("diagnostics", "print_every"), -1, "diagnostics.print_every"),
            (("action_journal", "max_file_bytes"), 0, "action_journal.max_file_bytes"),
            (("action_journal", "retained_files"), 0, "action_journal.retained_files"),
        )
        for field_path, invalid_value, expected_error in invalid_cases:
            with self.subTest(field=expected_error), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data[field_path[0]][field_path[1]] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_empty_or_non_string_network_hosts(self):
        invalid_cases = (
            ("state_bind_host", ""),
            ("orin_host", 1234),
        )
        for field, invalid_value in invalid_cases:
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data["network"][field] = invalid_value
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, f"network.{field}"):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_invalid_fixed_action_tuning(self):
        invalid_cases = (
            ({"expected_profile_sha256": "bad"}, "expected_profile_sha256"),
        )
        for changes, expected_error in invalid_cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                for relative_path in ("model.onnx", "machine_profile.json", "waypoint_slice.json"):
                    (project_root / relative_path).touch()
                config_data = valid_config_payload()
                config_data["fixed_action"].update(changes)
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_unsafe_or_inconsistent_manual_jog_configuration(self):
        invalid_cases = (
            ({"speed_fraction": 0.0}, "speed_fraction"),
            ({"speed_fraction": 0.21}, "speed_fraction"),
            ({"command_period_ms": 100}, "heartbeat_timeout_ms"),
            ({"heartbeat_timeout_ms": 100}, "heartbeat_timeout_ms"),
            ({"max_hold_ms": 6000}, "max_hold_ms"),
            ({"position_margin_m": 0.0}, "position_margin_m"),
            ({"allowed_actuators": ["boom", "swing"]}, "allowed_actuators"),
        )
        for changes, expected_error in invalid_cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                config_data = valid_config_payload()
                config_data["manual_jog"].update(changes)
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)

    def test_rejects_unsafe_or_inconsistent_follow_canary_configuration(self):
        invalid_cases = (
            ({"mode": "field"}, "mode"),
            ({"heartbeat_timeout_ms": 74}, "heartbeat_timeout_ms"),
            ({"allowed_actuators": []}, "allowed_actuators"),
            ({"allowed_actuators": ["boom", "boom"]}, "allowed_actuators"),
            ({"allowed_actuators": ["boom", "stick", "bucket"]}, "allowed_actuators"),
        )
        for changes, expected_error in invalid_cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as directory:
                project_root = Path(directory)
                config_data = valid_config_payload()
                config_data["follow_control"].update(changes)
                config_path = project_root / "runtime.json"
                config_path.write_text(json.dumps(config_data), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, expected_error):
                    load_runtime_config(config_path, project_root=project_root)


if __name__ == "__main__":
    unittest.main()
