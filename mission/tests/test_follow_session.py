import unittest
from dataclasses import replace

from mission.follow import (
    FollowSession,
    FollowTrajectorySnapshot,
    TrajectoryDigestMismatch,
)


def valid_snapshot(**overrides):
    digest_overridden = "trajectory_sha256" in overrides
    values = {
        "frame_id": "machine_root_ros",
        "created_at_s": 10.0,
        "trajectory_id": "trajectory-001",
        "trajectory_sha256": "0" * 64,
        "mission_id": "mission-001",
        "mission_sha256": "b" * 64,
        "mission_phase": "dig",
        "task_mode": "MoveToDig",
        "planning_scope": "preview_global",
        "control_stage": "none",
        "workspace_constraint": "none",
        "execution_eligible": False,
        "source_bucket_tip_stamp_s": 9.8,
        "source_local_map_stamp_s": 9.7,
        "inputs_frozen_at_s": 10.0,
        "valid_until_s": 30.0,
        "input_source": "fixture",
        "map_source": "fixture_empty",
        "clock_mode": "ros_clock",
        "waypoints": ((0.0, 0.0, 0.0), (0.5, 0.0, 0.0)),
        "waypoint_tolerance_m": 0.02,
        "waypoint_dwell_s": 0.1,
        "tracking_timeout_s": 5.0,
    }
    values.update(overrides)
    snapshot = FollowTrajectorySnapshot(**values)
    if digest_overridden:
        return snapshot
    return replace(snapshot, trajectory_sha256=snapshot.computed_sha256())


class FollowSessionTests(unittest.TestCase):
    def test_new_bucket_tip_samples_complete_an_immutable_trajectory(self):
        session = FollowSession.start(valid_snapshot(), accepted_at_s=10.1)

        session, first = session.observe((0.0, 0.0, 0.0), sample_stamp_s=10.2, now_s=10.2)
        session, advanced = session.observe((0.0, 0.0, 0.0), sample_stamp_s=10.3, now_s=10.3)
        session, third = session.observe((0.5, 0.0, 0.0), sample_stamp_s=10.4, now_s=10.4)
        session, completed = session.observe((0.5, 0.0, 0.0), sample_stamp_s=10.5, now_s=10.5)

        self.assertTrue(first.sample_accepted)
        self.assertTrue(advanced.advanced)
        self.assertEqual(advanced.current_waypoint_index, 1)
        self.assertFalse(third.completed)
        self.assertTrue(completed.completed)
        self.assertFalse(completed.timed_out)

    def test_repeated_source_stamp_does_not_accumulate_dwell(self):
        session = FollowSession.start(valid_snapshot(), accepted_at_s=10.1)

        session, first = session.observe((0.0, 0.0, 0.0), sample_stamp_s=10.2, now_s=10.2)
        unchanged, duplicate = session.observe((0.0, 0.0, 0.0), sample_stamp_s=10.2, now_s=11.0)

        self.assertTrue(first.sample_accepted)
        self.assertFalse(duplicate.sample_accepted)
        self.assertEqual(unchanged, session)

    def test_tracking_timeout_is_a_failure_not_completion(self):
        session = FollowSession.start(valid_snapshot(tracking_timeout_s=0.5), accepted_at_s=10.1)
        session, _ = session.observe((1.0, 1.0, 1.0), sample_stamp_s=10.2, now_s=10.2)
        _, update = session.observe((1.0, 1.0, 1.0), sample_stamp_s=10.8, now_s=10.8)

        self.assertTrue(update.timed_out)
        self.assertFalse(update.completed)

    def test_snapshot_rejects_unsafe_or_mismatched_metadata(self):
        with self.assertRaisesRegex(ValueError, "execution_eligible"):
            valid_snapshot(execution_eligible=True).validate_for_shadow(
                expected_input_source="fixture", now_s=10.1
            )
        with self.assertRaisesRegex(ValueError, "input_source"):
            valid_snapshot().validate_for_shadow(expected_input_source="replay", now_s=10.1)
        with self.assertRaisesRegex(ValueError, "expired"):
            valid_snapshot(valid_until_s=10.05).validate_for_shadow(
                expected_input_source="fixture", now_s=10.1
            )
        with self.assertRaisesRegex(ValueError, "map_source"):
            valid_snapshot(input_source="live", map_source="fixture_empty")

    def test_snapshot_digest_rejects_waypoint_or_timeout_tampering(self):
        snapshot = valid_snapshot()

        with self.assertRaises(TrajectoryDigestMismatch):
            replace(snapshot, waypoints=((9.0, 9.0, 9.0),)).validate_for_shadow(
                expected_input_source="fixture", now_s=10.1
            )
        with self.assertRaises(TrajectoryDigestMismatch):
            replace(snapshot, tracking_timeout_s=99.0).validate_for_shadow(
                expected_input_source="fixture", now_s=10.1
            )

    def test_execution_validation_accepts_only_fresh_live_strict_trajectory(self):
        snapshot = valid_snapshot(
            planning_scope="execution_strict",
            execution_eligible=True,
            input_source="live",
            map_source="live_local_map",
            control_stage="commissioning",
            workspace_constraint="disabled_by_operator",
        )

        snapshot.validate_for_execution(now_s=10.1, expected_control_stage="commissioning")

        with self.assertRaisesRegex(ValueError, "execution_eligible"):
            replace(snapshot, execution_eligible=False).validate_for_execution(
                now_s=10.1, expected_control_stage="commissioning"
            )
        with self.assertRaisesRegex(ValueError, "planning_scope"):
            replace(snapshot, planning_scope="workspace_strict").validate_for_execution(
                now_s=10.1, expected_control_stage="commissioning"
            )
        with self.assertRaisesRegex(ValueError, "input_source"):
            replace(snapshot, input_source="fixture", map_source="fixture_empty").validate_for_execution(
                now_s=10.1, expected_control_stage="commissioning"
            )

    def test_snapshot_accepts_fresh_frozen_inputs_after_slow_planning(self):
        snapshot = valid_snapshot(
            created_at_s=15.0,
            inputs_frozen_at_s=10.0,
            valid_until_s=25.0,
            planning_scope="execution_strict",
            execution_eligible=True,
            input_source="live",
            map_source="live_local_map",
            control_stage="commissioning",
            workspace_constraint="disabled_by_operator",
        )

        snapshot.validate_for_execution(now_s=15.1, expected_control_stage="commissioning")

    def test_execution_validation_allows_only_bounded_clock_skew(self):
        snapshot = valid_snapshot(
            planning_scope="execution_strict",
            execution_eligible=True,
            input_source="live",
            map_source="live_local_map",
            control_stage="commissioning",
            workspace_constraint="disabled_by_operator",
        )

        snapshot.validate_for_execution(
            now_s=9.8,
            expected_control_stage="commissioning",
        )

        with self.assertRaisesRegex(ValueError, "from the future"):
            snapshot.validate_for_execution(
                now_s=9.4,
                expected_control_stage="commissioning",
            )

    def test_retired_production_stage_is_not_a_valid_snapshot_contract(self):
        with self.assertRaisesRegex(ValueError, "control_stage"):
            valid_snapshot(control_stage="production")


if __name__ == "__main__":
    unittest.main()
