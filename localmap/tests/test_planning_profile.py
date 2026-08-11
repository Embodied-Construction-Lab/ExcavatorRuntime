import json
import sys
import tempfile
import unittest
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_profile import load_planning_profile


def valid_planning_payload():
    return {
        "schema": "planning_profile_v2",
        "profile_id": "scale_excavator_live",
        "inputs": {
            "perception_profile": "localmap/config/perception.json",
            "machine_profile": "../shared/machine_profile.json",
            "reachable_workspace": "../shared/reachable_workspaces/scale_excavator_workspace.json",
        },
        "freshness": {
            "local_map_max_age_ms": 500,
            "bucket_tip_max_age_ms": 500,
            "octomap_timeout_s": 5.0,
        },
        "obstacle_adapter": {
            "box_size_m": 0.2,
            "max_obstacles": 1000,
        },
        "planner": {
            "execution_workspace_mode": "disabled_by_operator",
            "collision_radius_m": 0.05,
            "step_size_m": 0.2,
            "edge_check_step_m": 0.04,
            "max_iterations": 6000,
            "goal_sample_rate": 0.15,
            "start_mask_radius_m": 0.15,
            "goal_mask_radius_m": 0.45,
            "waypoint_count": 5,
            "seed": 8,
        },
        "task_mode_by_target_kind": {
            "dig": "MoveToDig",
            "dump": "CarryMaterial",
        },
    }


def valid_perception_payload():
    return {
        "schema": "perception_profile_v1",
        "profile_id": "test_live",
        "expected_frame": "machine_root_ros",
        "inputs": {
            "rslidar_config": "runtime/lidar.yaml",
            "extrinsics": "localmap/config/extrinsics.json",
            "targets": "localmap/config/targets.json",
            "bucket_tip_bridge": "localmap/config/bridge.json",
        },
        "outputs": {
            "live_local_map": "runtime/live/local_map.json",
            "live_bucket_tip": "runtime/live/bucket_tip.json",
            "log_dir": "runtime/logs",
        },
        "topics": {
            "raw_cloud": "/raw_cloud",
            "machine_cloud": "/machine_cloud",
            "octomap_cells": "/octomap_cells",
            "bucket_tip_fk": "/bucket_tip_fk",
            "bucket_tip_machine_root": "/bucket_tip_machine_root",
        },
        "local_map": {
            "bounds": [-2.0, 3.0, -0.8, 1.0, -0.5, 4.0],
            "write_every": 5,
            "publish_every": 10,
        },
        "octomap": {
            "resolution_m": 0.05,
            "max_range_m": 4.0,
            "filter_ground_plane": False,
            "reset_interval_s": 1.0,
            "crop_bounds": [-1.5, 3.0, -0.42, 1.0, -0.5, 4.0],
        },
    }


def write_profile_pair(project_root: Path, planning_data=None, perception_data=None) -> Path:
    perception_path = project_root / "localmap/config/perception.json"
    perception_path.parent.mkdir(parents=True)
    perception_path.write_text(
        json.dumps(perception_data or valid_perception_payload()),
        encoding="utf-8",
    )
    planning_path = project_root / "planning.json"
    planning_path.write_text(
        json.dumps(planning_data or valid_planning_payload()),
        encoding="utf-8",
    )
    return planning_path


class PlanningProfileTest(unittest.TestCase):
    def test_deployed_profile_outputs_only_start_and_goal_waypoints(self):
        profile = load_planning_profile()

        self.assertEqual(profile.planner.waypoint_count, 2)
        self.assertEqual(profile.obstacle_adapter.max_obstacles, 0)

    def test_allows_zero_obstacles_for_demo_planning(self):
        data = valid_planning_payload()
        data["obstacle_adapter"]["max_obstacles"] = 0

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            profile = load_planning_profile(
                write_profile_pair(project_root, planning_data=data),
                project_root=project_root,
            )

        self.assertEqual(profile.obstacle_adapter.max_obstacles, 0)

    def test_derives_shared_contract_from_perception_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            profile = load_planning_profile(
                write_profile_pair(project_root),
                project_root=project_root,
            )

        self.assertEqual(profile.expected_frame, "machine_root_ros")
        self.assertEqual(profile.inputs.live_local_map, project_root / "runtime/live/local_map.json")
        self.assertEqual(profile.inputs.live_bucket_tip, project_root / "runtime/live/bucket_tip.json")
        self.assertEqual(profile.inputs.octomap_topic, "/octomap_cells")
        self.assertEqual(profile.outputs.directory, project_root / "runtime/live")
        self.assertEqual(profile.obstacle_adapter.bounds, (-1.5, 3.0, -0.42, 1.0, -0.5, 4.0))
        self.assertEqual(profile.planner.bounds, (-2.0, 3.0, -0.8, 1.0, -0.5, 4.0))
        self.assertEqual(profile.planner.execution_workspace_mode, "disabled_by_operator")

    def test_rejects_unknown_execution_workspace_mode(self):
        data = valid_planning_payload()
        data["planner"]["execution_workspace_mode"] = "ignore_everything"

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "execution_workspace_mode"):
                load_planning_profile(
                    write_profile_pair(project_root, planning_data=data),
                    project_root=project_root,
                )

    def test_rejects_unknown_profile_fields(self):
        data = valid_planning_payload()
        data["planner"]["max_iteration"] = 10

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "planner.*未知字段.*max_iteration"):
                load_planning_profile(
                    write_profile_pair(project_root, planning_data=data),
                    project_root=project_root,
                )

    def test_rejects_unsupported_profile_schema(self):
        data = valid_planning_payload()
        data["schema"] = "planning_profile_v1"

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "schema.*planning_profile_v2"):
                load_planning_profile(
                    write_profile_pair(project_root, planning_data=data),
                    project_root=project_root,
                )

    def test_rejects_invalid_numeric_settings(self):
        invalid_cases = (
            (("freshness", "local_map_max_age_ms"), True, "freshness.local_map_max_age_ms"),
            (("obstacle_adapter", "box_size_m"), 0.0, "obstacle_adapter.box_size_m"),
            (("planner", "goal_sample_rate"), 1.5, "planner.goal_sample_rate"),
            (("planner", "max_iterations"), 0, "planner.max_iterations"),
            (("planner", "waypoint_count"), True, "planner.waypoint_count"),
        )
        for field_path, invalid_value, expected_error in invalid_cases:
            with self.subTest(field=expected_error), tempfile.TemporaryDirectory() as directory:
                data = valid_planning_payload()
                data[field_path[0]][field_path[1]] = invalid_value
                project_root = Path(directory)
                with self.assertRaisesRegex(ValueError, expected_error):
                    load_planning_profile(
                        write_profile_pair(project_root, planning_data=data),
                        project_root=project_root,
                    )

    def test_rejects_wrong_task_mapping(self):
        data = valid_planning_payload()
        data["task_mode_by_target_kind"]["dig"] = "CarryMaterial"

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "task_mode_by_target_kind"):
                load_planning_profile(
                    write_profile_pair(project_root, planning_data=data),
                    project_root=project_root,
                )

    def test_rejects_live_input_path_collisions(self):
        perception = valid_perception_payload()
        perception["outputs"]["live_bucket_tip"] = perception["outputs"]["live_local_map"]

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "live输入路径必须互不相同"):
                load_planning_profile(
                    write_profile_pair(project_root, perception_data=perception),
                    project_root=project_root,
                )

    def test_rejects_live_inputs_in_different_directories(self):
        perception = valid_perception_payload()
        perception["outputs"]["live_bucket_tip"] = "runtime/other/bucket_tip.json"

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "同一输出目录"):
                load_planning_profile(
                    write_profile_pair(project_root, perception_data=perception),
                    project_root=project_root,
                )

    def test_rejects_live_input_that_uses_planning_artifact_name(self):
        perception = valid_perception_payload()
        perception["outputs"]["live_local_map"] = "runtime/live/local_map.octomap_obstacles.json"

        with tempfile.TemporaryDirectory() as directory:
            project_root = Path(directory)
            with self.assertRaisesRegex(ValueError, "规划产物路径冲突"):
                load_planning_profile(
                    write_profile_pair(project_root, perception_data=perception),
                    project_root=project_root,
                )


if __name__ == "__main__":
    unittest.main()
