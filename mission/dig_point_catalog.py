"""Strict authoritative catalog of fixed excavation points and groups."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping


SCHEMA_VERSION = "excavation_dig_point_catalog.v1"
_FIELDS = {
    "schema_version",
    "frame_id",
    "dig_points",
    "default_dig_group",
    "dig_groups",
}
_SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}")


class DigPointCatalogError(ValueError):
    """The fixed dig-point catalog is unavailable or invalid."""


@dataclass(frozen=True)
class DigPointCatalog:
    frame_id: str
    points: Mapping[str, tuple[float, float, float]]
    groups: Mapping[str, tuple[str, ...]]
    default_group_id: str


def load_dig_point_catalog(path: Path) -> DigPointCatalog:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DigPointCatalogError(f"cannot load dig point catalog: {exc}") from exc
    if not isinstance(value, dict) or set(value) != _FIELDS:
        raise DigPointCatalogError("dig point catalog fields are invalid")
    if value["schema_version"] != SCHEMA_VERSION:
        raise DigPointCatalogError("unsupported dig point catalog schema")
    if value["frame_id"] != "machine_root_ros":
        raise DigPointCatalogError("dig point catalog frame must be machine_root_ros")

    points = _load_points(value["dig_points"])
    groups = _load_groups(value["dig_groups"], tuple(points))
    default_group_id = _identifier(value["default_dig_group"], "default group")
    if default_group_id not in groups:
        raise DigPointCatalogError("default dig group is not defined")
    return DigPointCatalog(
        frame_id=value["frame_id"],
        points=MappingProxyType(points),
        groups=MappingProxyType(groups),
        default_group_id=default_group_id,
    )


def _load_points(value: object) -> dict[str, tuple[float, float, float]]:
    if not isinstance(value, dict) or not value:
        raise DigPointCatalogError("dig_points must be a non-empty object")
    points: dict[str, tuple[float, float, float]] = {}
    for raw_id, raw_position in value.items():
        point_id = _identifier(raw_id, "dig point id")
        if not isinstance(raw_position, list) or len(raw_position) != 3:
            raise DigPointCatalogError("dig point position must contain three numbers")
        if any(isinstance(axis, bool) or not isinstance(axis, (int, float)) for axis in raw_position):
            raise DigPointCatalogError("dig point position must be numeric")
        position = tuple(float(axis) for axis in raw_position)
        if not all(math.isfinite(axis) for axis in position):
            raise DigPointCatalogError("dig point position must be finite")
        points[point_id] = position
    return points


def _load_groups(
    value: object,
    point_ids: tuple[str, ...],
) -> dict[str, tuple[str, ...]]:
    if not isinstance(value, dict) or not value:
        raise DigPointCatalogError("dig_groups must be a non-empty object")
    groups: dict[str, tuple[str, ...]] = {}
    known = frozenset(point_ids)
    for raw_id, raw_members in value.items():
        group_id = _identifier(raw_id, "dig group id")
        if not isinstance(raw_members, list) or not raw_members:
            raise DigPointCatalogError("each dig group must be a non-empty list")
        members = tuple(_identifier(member, "dig group member") for member in raw_members)
        if len(members) != len(set(members)):
            raise DigPointCatalogError("dig group members must be unique")
        if not set(members) <= known:
            raise DigPointCatalogError("dig group references an unknown point")
        groups[group_id] = members
    if groups.get("all") != point_ids:
        raise DigPointCatalogError("dig_groups.all must exactly match dig_points order")
    return groups


def _identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or _SAFE_ID.fullmatch(value) is None:
        raise DigPointCatalogError(f"{field} is invalid")
    return value
