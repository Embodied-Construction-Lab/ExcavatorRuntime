import sys
import tempfile
import unittest
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import load_mission
from mission.demo import load_demo_program
from mission.markers import (
    build_demo_marker_specs,
    build_mission_marker_specs,
    MissionMarkerStyleError,
    load_mission_marker_style,
)


MARKER_STYLE = AIRY_ROOT / "mission" / "config" / "marker_style.json"


class MissionMarkersTest(unittest.TestCase):
    def test_builds_distinct_dig_and_dump_marker_specs_from_same_snapshot(self):
        mission = load_mission(AIRY_ROOT / "mission" / "replays" / "mission.placeholder.json")
        style = load_mission_marker_style(MARKER_STYLE)

        specs = build_mission_marker_specs(mission, style)

        self.assertEqual([spec.phase for spec in specs], ["dig", "dump"])
        self.assertEqual(specs[0].position_m, (0.6, -0.3, -0.4))
        self.assertEqual(specs[1].position_m, (0.45, 0.3, 0.1))
        self.assertNotEqual(specs[0].color_rgba, specs[1].color_rgba)
        self.assertIn("PLACEHOLDER", specs[0].label)
        self.assertEqual(specs[0].frame_id, "machine_root_ros")

    def test_display_diameter_is_independent_from_follow_target_radius(self):
        mission = load_mission(AIRY_ROOT / "mission" / "config" / "excavation_cycle.json")
        style = load_mission_marker_style(MARKER_STYLE)

        specs = build_mission_marker_specs(mission, style)

        self.assertEqual(mission.targets["dig"].radius_m, 0.25)
        self.assertEqual(mission.limits.waypoint_tolerance_m, 0.25)
        self.assertEqual(specs[0].diameter_m, 0.08)
        self.assertNotEqual(specs[0].diameter_m, mission.targets["dig"].radius_m * 2.0)

    def test_invalid_marker_style_is_reported_as_a_style_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "marker_style.json"
            path.write_text("{not-json", encoding="utf-8")

            with self.assertRaises(MissionMarkerStyleError):
                load_mission_marker_style(path)

    def test_builds_all_demo_dig_points_and_one_common_dump_marker(self):
        program = load_demo_program(
            AIRY_ROOT / "mission" / "config" / "excavation_demo.json"
        )
        style = load_mission_marker_style(MARKER_STYLE)

        specs = build_demo_marker_specs(program, style)

        self.assertEqual(
            [spec.phase for spec in specs],
            [f"dig:{point.point_id}" for point in program.dig_points] + ["dump"],
        )
        self.assertIn(program.dig_points[0].point_id, specs[0].label)
        self.assertEqual(specs[-1].position_m, program.dump_target.position_m)


if __name__ == "__main__":
    unittest.main()
