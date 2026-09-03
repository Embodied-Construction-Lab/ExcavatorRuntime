"""Offline export of complete selector candidate traces for TADPS benchmarks.

The input boundary is deliberately narrower than a generic selector log.  Each
JSONL row must attest that it contains the complete preselection candidate set;
a selected point or an RViz marker stream cannot be used to reconstruct it.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping


class TadpsReplayExportError(ValueError):
    """The selector trace cannot support an honest candidate replay."""


_FRAME_FIELDS = frozenset(
    {
        "schema_version",
        "record_type",
        "sequence_id",
        "frame_id",
        "frame_index",
        "stamp_s",
        "map_sha256",
        "candidate_set_complete",
        "candidates",
    }
)
_CANDIDATE_FIELDS = frozenset(
    {
        "candidate_id",
        "position_m",
        "features",
        "downstream_planner_accepted",
    }
)
_FEATURE_FIELDS = frozenset(
    {
        "terrain_valid",
        "surface_height_m",
        "local_soil_height_score",
        "confidence_score",
        "edge_clearance_score",
        "relative_height_score",
        "roughness_penalty",
    }
)
_NORMALIZED_FEATURES = _FEATURE_FIELDS - {"terrain_valid", "surface_height_m"}


def export_tadps_candidate_replay(source: Path) -> dict[str, Any]:
    """Convert a complete selector-candidate JSONL trace into replay schema v1."""

    records = tuple(_parse_frame_record(item) for item in _load_jsonl(source))
    if not records:
        raise TadpsReplayExportError(
            "selector trace must contain at least one candidate frame"
        )
    frame_id = records[0]["frame_id"]
    if any(record["frame_id"] != frame_id for record in records[1:]):
        raise TadpsReplayExportError("frame_id must be the same in every frame")

    sequences: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        frames = sequences.setdefault(record["sequence_id"], [])
        if frames and (
            record["frame_index"] <= frames[-1]["frame_index"]
            or record["stamp_s"] <= frames[-1]["stamp_s"]
        ):
            raise TadpsReplayExportError(
                "frame_index and stamp_s must be strictly increasing per sequence"
            )
        frames.append(
            {
                "frame_index": record["frame_index"],
                "stamp_s": record["stamp_s"],
                "map_sha256": record["map_sha256"],
                "candidates": record["candidates"],
            }
        )
    return {
        "schema_version": "tadps_candidate_replay.v1",
        "frame_id": frame_id,
        "sequences": [
            {"sequence_id": sequence_id, "frames": frames}
            for sequence_id, frames in sequences.items()
        ],
    }


def _load_jsonl(source: Path) -> tuple[Any, ...]:
    resolved = source.expanduser().resolve(strict=True)
    if not resolved.is_file():
        raise TadpsReplayExportError("selector trace must be a regular file")
    rows: list[Any] = []
    for line_number, line in enumerate(
        resolved.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            raise TadpsReplayExportError(
                f"selector trace line {line_number} must not be blank"
            )
        try:
            rows.append(json.loads(line, object_pairs_hook=_unique_object))
        except json.JSONDecodeError as exc:
            raise TadpsReplayExportError(
                f"selector trace line {line_number} is not valid JSON"
            ) from exc
    return tuple(rows)


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise TadpsReplayExportError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def _parse_frame_record(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TadpsReplayExportError("selector candidate frame must be an object")
    if (
        value.get("schema_version") != "tadps_selector_candidate_frame.v1"
        or value.get("record_type") != "complete_preselection_candidate_set"
        or value.get("candidate_set_complete") is not True
        or "candidates" not in value
    ):
        raise TadpsReplayExportError(
            "selected-point logs cannot reconstruct candidates; future logger must "
            "record complete preselection candidate sets"
        )
    if set(value) != _FRAME_FIELDS:
        raise TadpsReplayExportError("selector candidate frame fields are invalid")

    sequence_id = _nonempty_string(value["sequence_id"], "sequence_id")
    frame_id = _nonempty_string(value["frame_id"], "frame_id")
    frame_index = _integer(value["frame_index"], "frame_index", 0, 2**63 - 1)
    stamp_s = _number(value["stamp_s"], "stamp_s", 0.0, float("inf"))
    map_sha256 = value["map_sha256"]
    if (
        not isinstance(map_sha256, str)
        or len(map_sha256) != 64
        or any(character not in "0123456789abcdef" for character in map_sha256)
    ):
        raise TadpsReplayExportError("map_sha256 must be a lowercase SHA-256 digest")
    raw_candidates = value["candidates"]
    if not isinstance(raw_candidates, list):
        raise TadpsReplayExportError("candidates must be a list")
    candidates = [_parse_candidate(candidate) for candidate in raw_candidates]
    candidate_ids = [candidate["candidate_id"] for candidate in candidates]
    if len(candidate_ids) != len(set(candidate_ids)):
        raise TadpsReplayExportError(
            "candidate_id values must be unique within each frame"
        )
    return {
        "sequence_id": sequence_id,
        "frame_id": frame_id,
        "frame_index": frame_index,
        "stamp_s": stamp_s,
        "map_sha256": map_sha256,
        "candidates": candidates,
    }


def _parse_candidate(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TadpsReplayExportError("candidate must be an object")
    if "downstream_planner_accepted" not in value:
        raise TadpsReplayExportError(
            "each candidate must record downstream_planner_accepted"
        )
    if set(value) != _CANDIDATE_FIELDS:
        raise TadpsReplayExportError("candidate fields are invalid")
    candidate_id = _nonempty_string(value["candidate_id"], "candidate_id")
    raw_position = value["position_m"]
    if not isinstance(raw_position, list) or len(raw_position) != 3:
        raise TadpsReplayExportError("position_m must contain exactly three numbers")
    position_m = [
        _number(component, "position_m", -1e6, 1e6) for component in raw_position
    ]
    raw_features = value["features"]
    if not isinstance(raw_features, Mapping) or set(raw_features) != _FEATURE_FIELDS:
        raise TadpsReplayExportError("candidate feature fields are invalid")
    if not isinstance(raw_features["terrain_valid"], bool):
        raise TadpsReplayExportError("terrain_valid must be boolean")
    features = {
        "terrain_valid": raw_features["terrain_valid"],
        "surface_height_m": _number(
            raw_features["surface_height_m"],
            "surface_height_m",
            -1e6,
            1e6,
        ),
        **{
            field: _number(raw_features[field], field, 0.0, 1.0)
            for field in _NORMALIZED_FEATURES
        },
    }
    planner_accepted = value["downstream_planner_accepted"]
    if planner_accepted is not None and not isinstance(planner_accepted, bool):
        raise TadpsReplayExportError(
            "downstream_planner_accepted must be boolean or null"
        )
    return {
        "candidate_id": candidate_id,
        "position_m": position_m,
        "features": features,
        "downstream_planner_accepted": planner_accepted,
    }


def _nonempty_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise TadpsReplayExportError(f"{field} must be a non-empty string")
    return value


def _number(value: Any, field: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TadpsReplayExportError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise TadpsReplayExportError(f"{field} must be a finite number")
    if not minimum <= number <= maximum:
        raise TadpsReplayExportError(f"{field} is out of range")
    return number


def _integer(value: Any, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TadpsReplayExportError(f"{field} must be an integer")
    if not minimum <= value <= maximum:
        raise TadpsReplayExportError(f"{field} is out of range")
    return value
