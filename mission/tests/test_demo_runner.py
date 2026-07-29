import json
import sys
from pathlib import Path

import pytest


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.demo import load_demo_program
from mission.demo_runner import DemoCycleFailed, run_demo_cycles
from mission.tests.test_demo_program import valid_demo_payload


def load_fixture(tmp_path: Path):
    path = tmp_path / "demo.json"
    path.write_text(json.dumps(valid_demo_payload()), encoding="utf-8")
    return load_demo_program(path)


def test_runs_every_dig_point_in_file_order_for_each_repeat(tmp_path):
    program = load_fixture(tmp_path)
    calls = []

    completed = run_demo_cycles(
        program,
        repeat=2,
        run_cycle=lambda point, cycle_index, cycle_count: calls.append(
            (point.point_id, cycle_index, cycle_count)
        ),
    )

    assert completed == 6
    assert calls == [
        ("dig_01", 1, 6),
        ("dig_02", 2, 6),
        ("dig_03", 3, 6),
        ("dig_01", 4, 6),
        ("dig_02", 5, 6),
        ("dig_03", 6, 6),
    ]


def test_stops_immediately_when_one_cycle_fails(tmp_path):
    program = load_fixture(tmp_path)
    calls = []

    def fail_second(point, cycle_index, cycle_count):
        calls.append(point.point_id)
        if cycle_index == 2:
            raise DemoCycleFailed(point.point_id, "FOLLOW_DIG_FAILED")

    with pytest.raises(DemoCycleFailed, match="dig_02.*FOLLOW_DIG_FAILED"):
        run_demo_cycles(program, repeat=2, run_cycle=fail_second)

    assert calls == ["dig_01", "dig_02"]


@pytest.mark.parametrize("repeat", [0, -1])
def test_rejects_non_positive_repeat(tmp_path, repeat):
    with pytest.raises(ValueError, match="repeat"):
        run_demo_cycles(
            load_fixture(tmp_path),
            repeat=repeat,
            run_cycle=lambda *_args: None,
        )
