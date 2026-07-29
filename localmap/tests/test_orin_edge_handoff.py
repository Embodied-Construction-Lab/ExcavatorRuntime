import json
import sys
import tempfile
import unittest
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
AIRY_ROOT = LOCALMAP_DIR.parent
sys.path.insert(0, str(LOCALMAP_DIR))
sys.path.insert(0, str(AIRY_ROOT))

from localmap_core.orin_edge_handoff import (
    build_orin_edge_handoff,
    write_orin_edge_handoff,
)
from mission.contract import load_mission


MISSION_PATH = AIRY_ROOT / "mission" / "config" / "excavation_cycle.json"


def trajectory(mission_sha256: str) -> dict:
    return {
        "schema_version": "trajectory_command.v1",
        "timestamp_s": 100.0,
        "frame_id": "machine_root_ros",
        "task_mode": "MoveToDig",
        "waypoints_base": [[0.1, 0.2, 0.3], [0.8, -0.1, 0.0]],
        "waypoint_count": 2,
        "target_threshold": 0.03,
        "tube_radius": 0.04,
        "planning_scope": "execution_strict",
        "execution_eligible": True,
        "mission": {
            "id": "field_cycle_001",
            "sha256": mission_sha256,
            "phase": "dig",
        },
        "planner": {
            "workspace_constraint": "disabled_by_operator",
            "workspace_disable_reason": "operator_temporary_workspace_invalid",
            "reachable_workspace": None,
        },
    }


class OrinEdgeHandoffTest(unittest.TestCase):
    def test_builds_auditable_manifest_for_current_execution_trajectory(self):
        mission = load_mission(MISSION_PATH)
        document = trajectory(mission.sha256)
        document["waypoints_base"][-1] = list(mission.targets["dig"].position_m)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.json"
            path.write_text(json.dumps(document), encoding="utf-8")

            manifest = build_orin_edge_handoff(
                path,
                mission=mission,
                phase="dig",
                source_bucket_tip={
                    "stamp_s": 99.9,
                    "frame_id": "machine_root_ros",
                    "position_m": [0.1, 0.2, 0.3],
                },
                created_at_s=100.1,
            )

        self.assertEqual(manifest["schema_version"], "orin_edge_trajectory_handoff.v1")
        self.assertEqual(manifest["mission"]["sha256"], mission.sha256)
        self.assertEqual(manifest["trajectory"]["frame_id"], "machine_root_ros")
        self.assertEqual(manifest["trajectory"]["phase"], "dig")
        self.assertEqual(len(manifest["trajectory"]["sha256"]), 64)
        self.assertEqual(manifest["start_distance_m"], 0.0)
        self.assertEqual(
            manifest["waypoint_tolerance_m"],
            mission.limits.waypoint_tolerance_m,
        )

    def test_rejects_a_trajectory_that_does_not_start_at_the_frozen_tip(self):
        mission = load_mission(MISSION_PATH)
        document = trajectory(mission.sha256)
        document["waypoints_base"][-1] = list(mission.targets["dig"].position_m)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.json"
            path.write_text(json.dumps(document), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "first waypoint"):
                build_orin_edge_handoff(
                    path,
                    mission=mission,
                    phase="dig",
                    source_bucket_tip={
                        "stamp_s": 99.9,
                        "frame_id": "machine_root_ros",
                        "position_m": [0.8, 0.8, 0.8],
                    },
                    created_at_s=100.1,
                )

    def test_writes_manifest_atomically_as_canonical_json(self):
        manifest = {
            "schema_version": "orin_edge_trajectory_handoff.v1",
            "trajectory": {"sha256": "a" * 64},
        }
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "handoff.json"

            write_orin_edge_handoff(destination, manifest)

            self.assertEqual(
                json.loads(destination.read_text(encoding="utf-8")),
                manifest,
            )
            self.assertEqual(tuple(Path(directory).glob(".handoff.json.*")), ())

if __name__ == "__main__":
    unittest.main()
