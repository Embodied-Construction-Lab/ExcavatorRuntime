import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localmap_core.simple_bucket_tip_planner import PlanningBounds, plan_bucket_tip_path


class SimpleBucketTipPlannerTest(unittest.TestCase):
    def test_returns_straight_path_when_no_obstacles_block_goal(self):
        path = plan_bucket_tip_path(
            start=np.array([0.0, 0.2, 0.0]),
            goal=np.array([1.0, 0.2, 0.0]),
            obstacles=[],
            bounds=PlanningBounds.from_values([-0.5, 1.5, 0.0, 1.0, -0.5, 0.5]),
            waypoint_count=2,
            seed=7,
        )

        self.assertTrue(path.success)
        self.assertEqual(len(path.waypoints), 2)
        np.testing.assert_allclose(path.waypoints[0], [0.0, 0.2, 0.0])
        np.testing.assert_allclose(path.waypoints[-1], [1.0, 0.2, 0.0])
        self.assertEqual(path.reason, "straight_line")

    def test_routes_around_box_obstacle_between_start_and_goal(self):
        obstacle = {
            "id": "box_1",
            "shape": "box",
            "center_m": [0.5, 0.2, 0.0],
            "size_m": [0.25, 0.5, 0.5],
            "confidence": 1.0,
        }

        path = plan_bucket_tip_path(
            start=np.array([0.0, 0.2, 0.0]),
            goal=np.array([1.0, 0.2, 0.0]),
            obstacles=[obstacle],
            bounds=PlanningBounds.from_values([-0.2, 1.2, 0.0, 0.8, -0.8, 0.8]),
            collision_radius_m=0.03,
            step_size_m=0.18,
            edge_check_step_m=0.03,
            max_iterations=2500,
            goal_sample_rate=0.2,
            seed=11,
        )

        self.assertTrue(path.success, path.reason)
        np.testing.assert_allclose(path.waypoints[0], [0.0, 0.2, 0.0])
        np.testing.assert_allclose(path.waypoints[-1], [1.0, 0.2, 0.0])

        # 关键行为：路径不应穿过障碍物膨胀盒。
        for point in path.waypoints:
            inside_x = 0.5 - 0.125 - 0.03 <= point[0] <= 0.5 + 0.125 + 0.03
            inside_y = 0.2 - 0.25 - 0.03 <= point[1] <= 0.2 + 0.25 + 0.03
            inside_z = -0.25 - 0.03 <= point[2] <= 0.25 + 0.03
            self.assertFalse(inside_x and inside_y and inside_z, point.tolist())

    def test_endpoint_only_mode_rejects_a_blocked_straight_path(self):
        obstacle = {
            "id": "box_1",
            "shape": "box",
            "center_m": [0.5, 0.2, 0.0],
            "size_m": [0.25, 0.5, 0.5],
            "confidence": 1.0,
        }

        path = plan_bucket_tip_path(
            start=np.array([0.0, 0.2, 0.0]),
            goal=np.array([1.0, 0.2, 0.0]),
            obstacles=[obstacle],
            bounds=PlanningBounds.from_values(
                [-0.2, 1.2, 0.0, 0.8, -0.8, 0.8]
            ),
            collision_radius_m=0.03,
            edge_check_step_m=0.03,
            waypoint_count=2,
            seed=11,
        )

        self.assertFalse(path.success)
        self.assertEqual(path.reason, "endpoint_only_path_blocked")


if __name__ == "__main__":
    unittest.main()
