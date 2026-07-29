import sys
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import load_mission
from mission.demo import load_demo_program
from mission.markers import build_demo_marker_specs, build_mission_marker_specs


class MissionMarkersTest(unittest.TestCase):
    def test_builds_distinct_dig_and_dump_marker_specs_from_same_snapshot(self):
        mission = load_mission(AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json")

        specs = build_mission_marker_specs(mission)

        self.assertEqual([spec.phase for spec in specs], ["dig", "dump"])
        self.assertEqual(specs[0].position_m, (0.6, -0.3, -0.4))
        self.assertEqual(specs[1].position_m, (0.45, 0.3, 0.1))
        self.assertNotEqual(specs[0].color_rgba, specs[1].color_rgba)
        self.assertIn("PLACEHOLDER", specs[0].label)
        self.assertEqual(specs[0].frame_id, "machine_root_ros")

    def test_builds_all_demo_dig_points_and_one_common_dump_marker(self):
        program = load_demo_program(
            AIRY_ROOT / "mission" / "config" / "excavation_demo.json"
        )

        specs = build_demo_marker_specs(program)

        self.assertEqual(
            [spec.phase for spec in specs],
            [f"dig:{point.point_id}" for point in program.dig_points] + ["dump"],
        )
        self.assertIn(program.dig_points[0].point_id, specs[0].label)
        self.assertEqual(specs[-1].position_m, program.dump_target.position_m)


if __name__ == "__main__":
    unittest.main()
