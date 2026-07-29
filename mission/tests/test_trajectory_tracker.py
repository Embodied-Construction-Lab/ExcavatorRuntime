import sys
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.trajectory_tracker import TrajectoryTracker


class TrajectoryTrackerTest(unittest.TestCase):
    def test_zero_dwell_advances_on_first_sample_inside_tolerance(self):
        tracker = TrajectoryTracker(
            waypoints=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            tolerance_m=0.25,
            dwell_s=0.0,
            timeout_s=5.0,
        )

        tracker, update = tracker.advance((0.2, 0.0, 0.0), now_s=0.0)

        self.assertTrue(update.advanced)
        self.assertEqual(update.current_index, 1)
        self.assertFalse(update.completed)
        self.assertEqual(tracker.current_index, 1)

    def test_advances_waypoints_only_after_tip_dwells_inside_tolerance(self):
        tracker = TrajectoryTracker(
            waypoints=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            tolerance_m=0.05,
            dwell_s=0.3,
            timeout_s=5.0,
        )

        tracker, first = tracker.advance((0.0, 0.0, 0.0), now_s=0.0)
        tracker, advanced = tracker.advance((0.0, 0.0, 0.0), now_s=0.3)
        tracker, outside = tracker.advance((0.5, 0.0, 0.0), now_s=0.4)
        tracker, goal_entered = tracker.advance((1.0, 0.0, 0.0), now_s=0.5)
        tracker, completed = tracker.advance((1.0, 0.0, 0.0), now_s=0.8)

        self.assertEqual(first.current_index, 0)
        self.assertFalse(first.advanced)
        self.assertEqual(advanced.current_index, 1)
        self.assertTrue(advanced.advanced)
        self.assertAlmostEqual(advanced.distance_m, 1.0)
        self.assertFalse(outside.completed)
        self.assertFalse(goal_entered.completed)
        self.assertTrue(completed.completed)

    def test_timeout_fails_without_skipping_to_next_waypoint(self):
        tracker = TrajectoryTracker(
            waypoints=((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            tolerance_m=0.05,
            dwell_s=0.3,
            timeout_s=1.0,
        )

        tracker, _ = tracker.advance((0.0, 0.0, 0.0), now_s=0.0)
        tracker, timed_out = tracker.advance((0.0, 0.0, 0.0), now_s=1.1)

        self.assertTrue(timed_out.timed_out)
        self.assertEqual(tracker.current_index, 0)
        self.assertFalse(tracker.completed)


if __name__ == "__main__":
    unittest.main()
