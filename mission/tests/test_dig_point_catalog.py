import copy
import json
import sys
from pathlib import Path

import pytest


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

from mission.dig_point_catalog import (
    DigPointCatalogError,
    load_dig_point_catalog,
)
CATALOG_PATH = (
    AIRY_ROOT / "mission/config/excavation_dig_point_catalog.v1.json"
)


def _valid_catalog() -> dict:
    return json.loads(CATALOG_PATH.read_text(encoding="utf-8"))


def _write_catalog(tmp_path: Path, value: dict) -> Path:
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_loads_the_current_ordered_points_and_groups():
    source = _valid_catalog()
    catalog = load_dig_point_catalog(CATALOG_PATH)

    assert tuple(catalog.points) == tuple(source["dig_points"])
    assert dict(catalog.points) == {
        point_id: tuple(position)
        for point_id, position in source["dig_points"].items()
    }
    assert tuple(catalog.groups) == tuple(source["dig_groups"])
    assert catalog.groups["all"] == tuple(catalog.points)
    with pytest.raises(TypeError):
        catalog.points["another"] = (1.0, 0.0, 0.0)


@pytest.mark.parametrize(
    ("mutate", "error"),
    [
        (lambda value: value.update(extra=True), "fields"),
        (lambda value: value.update(frame_id="map"), "machine_root_ros"),
        (
            lambda value: value["dig_points"].update(dig_bad=[True, 0, 0]),
            "numeric",
        ),
        (
            lambda value: value["dig_groups"].update(all=["dig_near_01"]),
            "exactly match",
        ),
        (
            lambda value: value["dig_groups"].update(near=["missing"]),
            "unknown point",
        ),
    ],
)
def test_rejects_ambiguous_catalogs(tmp_path, mutate, error):
    value = copy.deepcopy(_valid_catalog())
    mutate(value)

    with pytest.raises(DigPointCatalogError, match=error):
        load_dig_point_catalog(_write_catalog(tmp_path, value))
