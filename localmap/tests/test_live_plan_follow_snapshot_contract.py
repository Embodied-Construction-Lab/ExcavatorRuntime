import pytest


pytest.importorskip("rclpy")

from localmap_core.live_execution_planning import build_execution_snapshot_fields
from localmap_core.runtime_ros.live_plan_action_server import _trajectory_message
from mission.runtime_ros.follow_action_server import _snapshot_from_message


def test_slow_live_plan_snapshot_is_accepted_by_follow_contract():
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
        waypoint_dwell_s=0.0,
        tracking_timeout_s=20.0,
        control_stage="commissioning",
    )

    message = _trajectory_message(fields, target["target_id"])
    snapshot = _snapshot_from_message(message)

    snapshot.validate_for_execution(
        now_s=15.1, expected_control_stage="commissioning"
    )
    assert snapshot.inputs_frozen_at_s == 10.0
