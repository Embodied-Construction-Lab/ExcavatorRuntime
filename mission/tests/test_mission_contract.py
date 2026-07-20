import json
import copy
import sys
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MISSION = AIRY_ROOT / "mission" / "config" / "excavation_cycle.json"
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import MissionContractError, load_mission


def valid_mission_payload() -> dict:
    return {
        "schema_version": "excavation_mission.v1",
        "mission_id": "field_cycle_001",
        "mission_type": "dig_transport_dump",
        "frame_id": "machine_root_ros",
        "target_status": "placeholder",
        "targets": {
            "dig": {
                "position_m": [0.6, -0.3, -0.4],
                "normal": [0.0, 0.0, 1.0],
                "radius_m": 0.22,
            },
            "dump": {
                "position_m": [0.45, 0.3, 0.1],
                "normal": [0.0, 0.0, 1.0],
                "radius_m": 0.45,
            },
        },
        "limits": {
            "waypoint_tolerance_m": 0.05,
            "waypoint_dwell_s": 0.3,
            "tracking_timeout_s": 20.0,
            "settle_s": 0.5,
        },
    }


class MissionContractTest(unittest.TestCase):
    def test_accepts_zero_waypoint_dwell_for_immediate_progression(self):
        payload = valid_mission_payload()
        payload["targets"]["dig"]["radius_m"] = 0.25
        payload["limits"]["waypoint_tolerance_m"] = 0.25
        payload["limits"]["waypoint_dwell_s"] = 0.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            mission = load_mission(path)

        self.assertEqual(mission.limits.waypoint_tolerance_m, 0.25)
        self.assertEqual(mission.limits.waypoint_dwell_s, 0.0)

    def test_rejects_waypoint_tolerance_larger_than_a_target_radius(self):
        payload = valid_mission_payload()
        payload["limits"]["waypoint_tolerance_m"] = 0.25
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            with self.assertRaisesRegex(MissionContractError, "目标半径"):
                load_mission(path)

    def test_active_mission_is_a_valid_right_handed_snapshot(self):
        mission = load_mission(DEFAULT_MISSION)

        self.assertEqual(mission.frame_id, "machine_root_ros")
        self.assertIn(mission.target_status, {"placeholder", "rviz_adjusted", "field_validated"})
        self.assertEqual(mission.limits.waypoint_tolerance_m, 0.25)
        self.assertEqual(mission.limits.waypoint_dwell_s, 0.0)
        self.assertGreaterEqual(
            min(target.radius_m for target in mission.targets.values()),
            mission.limits.waypoint_tolerance_m,
        )

    def test_loads_file_as_immutable_right_handed_mission_snapshot(self):
        payload = valid_mission_payload()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.json"
            path.write_text(json.dumps(payload), encoding="utf-8")

            mission = load_mission(path)

        self.assertEqual(mission.frame_id, "machine_root_ros")
        self.assertEqual(mission.targets["dig"].position_m, (0.6, -0.3, -0.4))
        self.assertEqual(mission.targets["dump"].position_m, (0.45, 0.3, 0.1))
        self.assertEqual(len(mission.sha256), 64)
        with self.assertRaises(FrozenInstanceError):
            mission.frame_id = "machine_root"

    def test_rejects_unsafe_or_ambiguous_mission_files(self):
        cases = []

        wrong_frame = copy.deepcopy(valid_mission_payload())
        wrong_frame["frame_id"] = "machine_root"
        cases.append(("wrong_frame", wrong_frame, "machine_root_ros"))

        non_finite = copy.deepcopy(valid_mission_payload())
        non_finite["targets"]["dig"]["position_m"][0] = float("nan")
        cases.append(("non_finite", non_finite, "有限"))

        missing_target = copy.deepcopy(valid_mission_payload())
        del missing_target["targets"]["dump"]
        cases.append(("missing_target", missing_target, "dump"))

        unknown_field = copy.deepcopy(valid_mission_payload())
        unknown_field["enable_motion"] = True
        cases.append(("unknown_field", unknown_field, "未知字段"))

        invalid_radius = copy.deepcopy(valid_mission_payload())
        invalid_radius["targets"]["dig"]["radius_m"] = -0.1
        cases.append(("invalid_radius", invalid_radius, "radius_m"))

        with tempfile.TemporaryDirectory() as directory:
            for name, payload, error in cases:
                with self.subTest(name=name):
                    path = Path(directory) / f"{name}.json"
                    path.write_text(json.dumps(payload), encoding="utf-8")
                    with self.assertRaisesRegex(MissionContractError, error):
                        load_mission(path)


if __name__ == "__main__":
    unittest.main()
