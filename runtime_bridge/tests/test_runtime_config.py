import json
import tempfile
import unittest
from pathlib import Path

from runtime_bridge.runtime_config import load_runtime_config


def valid_config_payload():
    return {
        "schema": "pc_state_bridge_config_v1",
        "network": {
            "state_bind_host": "0.0.0.0",
            "state_port": 18081,
        },
        "diagnostics": {
            "print_every": 1,
            "write_every": 5,
        },
    }


class RuntimeConfigTest(unittest.TestCase):
    def test_loads_receive_only_state_bridge_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.json"
            path.write_text(json.dumps(valid_config_payload()), encoding="utf-8")

            config = load_runtime_config(path)

        self.assertEqual(config.network.state_endpoint, ("0.0.0.0", 18081))
        self.assertEqual(config.diagnostics.print_every, 1)
        self.assertEqual(config.diagnostics.write_every, 5)
        self.assertFalse(hasattr(config.network, "action_endpoint"))
        self.assertFalse(hasattr(config, "artifacts"))

    def test_rejects_unknown_or_missing_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.json"
            unknown = valid_config_payload()
            unknown["network"]["action_port"] = 18082
            path.write_text(json.dumps(unknown), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "network.*未知字段"):
                load_runtime_config(path)

            missing = valid_config_payload()
            del missing["network"]["state_port"]
            path.write_text(json.dumps(missing), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "network.*缺少字段"):
                load_runtime_config(path)

    def test_rejects_old_pc_motion_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.json"
            payload = valid_config_payload()
            payload["schema"] = "runtime_bridge_config_v10"
            path.write_text(json.dumps(payload), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "pc_state_bridge_config_v1"):
                load_runtime_config(path)

    def test_rejects_boolean_or_out_of_range_numbers(self):
        for field, value in (
            ("state_port", True),
            ("state_port", 0),
            ("state_port", 65536),
        ):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "runtime.json"
                payload = valid_config_payload()
                payload["network"][field] = value
                path.write_text(json.dumps(payload), encoding="utf-8")

                with self.assertRaisesRegex(ValueError, f"network.{field}"):
                    load_runtime_config(path)


if __name__ == "__main__":
    unittest.main()
