import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))
sys.path.insert(0, str(AIRY_ROOT / "localmap"))

from localmap_core.mission_target_registry import resolve_planning_target
from mission.contract import load_mission
from mission.demo import load_demo_program
from mission.tests.test_demo_program import valid_demo_payload
from mission.tests.test_mission_contract import valid_mission_payload


def write_json(tmp_path: Path, name: str, payload: dict) -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def target_message(**overrides):
    values = {
        "mission_id": "field_demo_001",
        "mission_sha256": "",
        "target_kind": "dig",
        "target_id": "field_demo_001:dig:dig_02",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_resolves_only_a_declared_demo_dig_point(tmp_path):
    mission = load_mission(
        write_json(tmp_path, "mission.json", valid_mission_payload())
    )
    demo = load_demo_program(
        write_json(tmp_path, "demo.json", valid_demo_payload())
    )

    resolved = resolve_planning_target(
        target_message(mission_sha256=demo.sha256),
        mission=mission,
        demo=demo,
    )

    assert resolved.target.position_m == (0.9, 0.0, -0.1)
    assert resolved.limits == demo.limits
    assert resolved.target_status == "rviz_adjusted"


def test_rejects_undeclared_or_mismatched_demo_target(tmp_path):
    mission = load_mission(
        write_json(tmp_path, "mission.json", valid_mission_payload())
    )
    demo = load_demo_program(
        write_json(tmp_path, "demo.json", valid_demo_payload())
    )

    with pytest.raises(ValueError, match="未声明"):
        resolve_planning_target(
            target_message(
                mission_sha256=demo.sha256,
                target_id="field_demo_001:dig:dig_99",
            ),
            mission=mission,
            demo=demo,
        )

    with pytest.raises(ValueError, match="snapshot"):
        resolve_planning_target(
            target_message(mission_sha256="f" * 64),
            mission=mission,
            demo=demo,
        )


def test_keeps_existing_single_mission_targets_valid(tmp_path):
    mission = load_mission(
        write_json(tmp_path, "mission.json", valid_mission_payload())
    )
    demo = load_demo_program(
        write_json(tmp_path, "demo.json", valid_demo_payload())
    )

    resolved = resolve_planning_target(
        target_message(
            mission_id=mission.mission_id,
            mission_sha256=mission.sha256,
            target_kind="dump",
            target_id=f"{mission.mission_id}:dump",
        ),
        mission=mission,
        demo=demo,
    )

    assert resolved.target == mission.targets["dump"]
    assert resolved.limits == mission.limits
