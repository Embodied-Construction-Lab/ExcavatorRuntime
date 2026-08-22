import copy
import json
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest


AIRY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DEMO = AIRY_ROOT / "mission" / "config" / "excavation_demo.json"
DEFAULT_MISSION = AIRY_ROOT / "mission" / "config" / "excavation_cycle.json"
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import load_mission
from mission.demo import DemoProgramError, load_demo_program


def valid_demo_payload() -> dict:
    return {
        "schema_version": "excavation_demo.v1",
        "demo_id": "field_demo_001",
        "frame_id": "machine_root_ros",
        "target_status": "rviz_adjusted",
        "dig_points": [
            {
                "point_id": "dig_01",
                "position_m": [0.8, 0.2, -0.1],
                "normal": [0.0, 0.0, 1.0],
                "radius_m": 0.25,
            },
            {
                "point_id": "dig_02",
                "position_m": [0.9, 0.0, -0.1],
                "normal": [0.0, 0.0, 1.0],
                "radius_m": 0.25,
            },
            {
                "point_id": "dig_03",
                "position_m": [0.8, -0.2, -0.1],
                "normal": [0.0, 0.0, 1.0],
                "radius_m": 0.25,
            },
        ],
        "dump_target": {
            "position_m": [0.0, -0.7, 0.1],
            "normal": [0.0, 0.0, 1.0],
            "radius_m": 0.25,
        },
        "limits": {
            "waypoint_tolerance_m": 0.25,
            "waypoint_dwell_s": 0.0,
            "tracking_timeout_s": 60.0,
            "settle_s": 0.5,
        },
    }


def write_demo(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / "excavation_demo.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_loads_ordered_immutable_multi_point_program(tmp_path):
    program = load_demo_program(write_demo(tmp_path, valid_demo_payload()))

    assert program.demo_id == "field_demo_001"
    assert [point.point_id for point in program.dig_points] == [
        "dig_01",
        "dig_02",
        "dig_03",
    ]
    assert program.dig_points[1].target.position_m == (0.9, 0.0, -0.1)
    assert program.dump_target.position_m == (0.0, -0.7, 0.1)
    assert len(program.sha256) == 64
    with pytest.raises(FrozenInstanceError):
        program.frame_id = "machine_root"


def test_active_demo_program_uses_current_right_handed_target_contract():
    program = load_demo_program(DEFAULT_DEMO)
    mission = load_mission(DEFAULT_MISSION)
    center_dig = next(point for point in program.dig_points if point.point_id == "dig_02")

    assert program.frame_id == "machine_root_ros"
    assert program.target_status in {"rviz_adjusted", "field_validated"}
    assert len(program.dig_points) >= 1
    assert center_dig.target == mission.targets["dig"]
    assert program.dump_target == mission.targets["dump"]
    assert program.limits == mission.limits
    assert program.limits.waypoint_tolerance_m == pytest.approx(0.25)


@pytest.mark.parametrize(
    ("mutate", "error"),
    [
        (lambda value: value.update(frame_id="machine_root"), "machine_root_ros"),
        (lambda value: value.update(dig_points=[]), "至少包含一个"),
        (
            lambda value: value["dig_points"][1].update(point_id="dig_01"),
            "point_id必须唯一",
        ),
        (
            lambda value: value["dig_points"][0].update(normal=[0.0, 0.0, 2.0]),
            "单位向量",
        ),
        (
            lambda value: value["dig_points"][0].update(radius_m=0.1),
            "目标半径",
        ),
        (lambda value: value.update(enable_motion=True), "未知字段"),
    ],
)
def test_rejects_ambiguous_or_unsafe_programs(tmp_path, mutate, error):
    payload = copy.deepcopy(valid_demo_payload())
    mutate(payload)

    with pytest.raises(DemoProgramError, match=error):
        load_demo_program(write_demo(tmp_path, payload))
