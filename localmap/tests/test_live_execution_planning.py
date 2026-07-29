import sys
import unittest
import hashlib
from pathlib import Path
from types import MappingProxyType

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.live_execution_planning import (
    build_execution_snapshot_fields,
    inject_live_target,
    validate_execution_workspace_provenance,
)


class LiveExecutionPlanningTests(unittest.TestCase):
    def test_execution_workspace_must_be_field_validated_for_current_urdf(self):
        urdf = b"<robot name='measured'/>"
        workspace = {
            "coordinate_frame": "machine_root_ros",
            "source": {
                "status": "field_validated",
                "urdf_sha256": hashlib.sha256(urdf).hexdigest(),
            },
        }
        validate_execution_workspace_provenance(workspace, urdf)

        with self.assertRaisesRegex(ValueError, "field_validated"):
            validate_execution_workspace_provenance(
                {**workspace, "source": {**workspace["source"], "status": "derived"}},
                urdf,
            )
        with self.assertRaisesRegex(ValueError, "URDF"):
            validate_execution_workspace_provenance(workspace, b"changed")

    def test_target_injection_is_immutable_and_preserves_mission_provenance(self):
        local_map = MappingProxyType({
            "frame_id": "machine_root_ros",
            "dig_targets": (),
            "dump_targets": (),
            "obstacles": (
                MappingProxyType({"type": "box", "center_m": (0.1, 0.2, 0.3)}),
            ),
        })
        target = {
            "target_id": "cycle:dig",
            "target_kind": "dig",
            "mission_id": "cycle",
            "mission_sha256": "a" * 64,
            "mission_phase": "dig",
            "position_m": [1.0, 0.0, 0.2],
            "normal": [0.0, 0.0, 1.0],
            "radius_m": 0.1,
        }

        injected, intent = inject_live_target(local_map, target)

        self.assertEqual(local_map["dig_targets"], ())
        self.assertEqual(injected["dig_targets"][0]["mission"]["sha256"], "a" * 64)
        self.assertEqual(
            injected["obstacles"],
            [{"type": "box", "center_m": [0.1, 0.2, 0.3]}],
        )
        self.assertEqual(intent.target_id, "cycle:dig")
        self.assertEqual(intent.task_mode, "MoveToDig")

    def test_execution_snapshot_rejects_non_strict_or_wrong_endpoint(self):
        trajectory = {
            "frame_id": "machine_root_ros",
            "task_mode": "MoveToDig",
            "planning_scope": "execution_strict",
            "execution_eligible": True,
            "waypoints_base": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2]],
            "target_threshold": 0.03,
            "mission": {"id": "cycle", "sha256": "a" * 64, "phase": "dig"},
            "planner": {
                "reachable_workspace": None,
                "workspace_constraint": "disabled_by_operator",
                "workspace_disable_reason": "operator_temporary_workspace_invalid",
            },
        }
        target = {
            "target_id": "cycle:dig",
            "target_kind": "dig",
            "mission_id": "cycle",
            "mission_sha256": "a" * 64,
            "mission_phase": "dig",
            "position_m": [1.0, 0.0, 0.2],
            "radius_m": 0.1,
        }

        fields = build_execution_snapshot_fields(
            trajectory,
            target,
            source_bucket_tip_stamp_s=9.8,
            source_local_map_stamp_s=9.7,
            inputs_frozen_at_s=10.0,
            created_at_s=10.0,
            waypoint_tolerance_m=0.25,
            waypoint_dwell_s=0.3,
            tracking_timeout_s=20.0,
            control_stage="commissioning",
        )
        self.assertTrue(fields["execution_eligible"])
        self.assertEqual(fields["map_source"], "live_local_map")
        self.assertEqual(fields["waypoint_tolerance_m"], 0.25)

        with self.assertRaisesRegex(ValueError, "workspace constraint provenance"):
            build_execution_snapshot_fields(
                {**trajectory, "planner": {"reachable_workspace": None}},
                target,
                source_bucket_tip_stamp_s=9.8,
                source_local_map_stamp_s=9.7,
                inputs_frozen_at_s=10.0,
                created_at_s=10.0,
                waypoint_tolerance_m=0.25,
                waypoint_dwell_s=0.3,
                tracking_timeout_s=20.0,
                control_stage="commissioning",
            )

        with self.assertRaisesRegex(ValueError, "execution_strict"):
            build_execution_snapshot_fields(
                {**trajectory, "planning_scope": "preview_global", "execution_eligible": False},
                target,
                source_bucket_tip_stamp_s=9.8,
                source_local_map_stamp_s=9.7,
                inputs_frozen_at_s=10.0,
                created_at_s=10.0,
                waypoint_tolerance_m=0.25,
                waypoint_dwell_s=0.3,
                tracking_timeout_s=20.0,
                control_stage="commissioning",
            )
        with self.assertRaisesRegex(ValueError, "endpoint"):
            build_execution_snapshot_fields(
                {**trajectory, "waypoints_base": [[0.0, 0.0, 0.0], [2.0, 0.0, 0.2]]},
                target,
                source_bucket_tip_stamp_s=9.8,
                source_local_map_stamp_s=9.7,
                inputs_frozen_at_s=10.0,
                created_at_s=10.0,
                waypoint_tolerance_m=0.25,
                waypoint_dwell_s=0.3,
                tracking_timeout_s=20.0,
                control_stage="commissioning",
            )

    def test_execution_snapshot_remains_valid_after_slow_planning(self):
        trajectory = {
            "frame_id": "machine_root_ros",
            "task_mode": "MoveToDig",
            "planning_scope": "execution_strict",
            "execution_eligible": True,
            "waypoints_base": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.2]],
            "target_threshold": 0.03,
            "mission": {"id": "cycle", "sha256": "a" * 64, "phase": "dig"},
            "planner": {
                "reachable_workspace": None,
                "workspace_constraint": "disabled_by_operator",
                "workspace_disable_reason": "operator_temporary_workspace_invalid",
            },
        }
        target = {
            "target_id": "cycle:dig",
            "target_kind": "dig",
            "mission_id": "cycle",
            "mission_sha256": "a" * 64,
            "mission_phase": "dig",
            "position_m": [1.0, 0.0, 0.2],
            "radius_m": 0.1,
        }

        fields = build_execution_snapshot_fields(
            trajectory,
            target,
            source_bucket_tip_stamp_s=9.8,
            source_local_map_stamp_s=9.7,
            inputs_frozen_at_s=10.0,
            created_at_s=15.0,
            waypoint_tolerance_m=0.25,
            waypoint_dwell_s=0.3,
            tracking_timeout_s=20.0,
            control_stage="commissioning",
        )

        self.assertEqual(fields["source_bucket_tip_stamp_s"], 9.8)
        self.assertEqual(fields["created_at_s"], 15.0)


if __name__ == "__main__":
    unittest.main()
