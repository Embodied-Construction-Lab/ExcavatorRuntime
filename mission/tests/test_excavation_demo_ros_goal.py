import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


rclpy = pytest.importorskip("rclpy")

from builtin_interfaces.msg import Time


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.demo import load_demo_program
from mission.runtime_ros.run_excavation_demo import _build_cycle_goal
from mission.tests.test_demo_program import valid_demo_payload


class _Node:
    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123, nanosec=4))
        )


def test_cycle_goal_binds_selected_dig_and_common_dump_to_program_hash(tmp_path):
    path = tmp_path / "demo.json"
    path.write_text(json.dumps(valid_demo_payload()), encoding="utf-8")
    program = load_demo_program(path)

    goal = _build_cycle_goal(_Node(), program, program.dig_points[1])

    assert goal.dig_target.target_id == "field_demo_001:dig:dig_02"
    assert goal.dig_target.position.x == pytest.approx(0.9)
    assert goal.dump_target.target_id == "field_demo_001:dump"
    assert goal.dump_target.position.y == pytest.approx(-0.7)
    for target, kind in (
        (goal.dig_target, "dig"),
        (goal.dump_target, "dump"),
    ):
        assert target.header.frame_id == "machine_root_ros"
        assert target.header.stamp.sec == 123
        assert target.target_kind == kind
        assert target.mission_phase == kind
        assert target.mission_id == program.demo_id
        assert target.mission_sha256 == program.sha256
