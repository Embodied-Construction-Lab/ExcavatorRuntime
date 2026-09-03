"""Offline-only benchmark for frozen TADPS candidate replays.

This module evaluates target selectors in memory.  It has no ROS, network, or
motion-control dependency and cannot emit actuator commands.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Mapping


METHODS = ("highest_point", "score_only", "full_tadps")


class TadpsBenchmarkError(ValueError):
    """The frozen replay or benchmark configuration is invalid."""


def evaluate_tadps_replay(
    replay: Mapping[str, Any],
    config: Mapping[str, Any],
) -> dict[str, Any]:
    """Evaluate all selectors over a frozen replay without motion side effects."""

    parsed_config = _parse_config(config)
    sequences = _parse_replay(replay, expected_frame_id=parsed_config.expected_frame_id)
    per_frame: list[dict[str, Any]] = []
    for sequence_id, frames in sequences:
        full = _FullTadpsSelector(parsed_config)
        previous_selected: dict[str, dict[str, Any] | None] = {
            method: None for method in METHODS
        }
        for frame in frames:
            for method in METHODS:
                started_ns = time.perf_counter_ns()
                selected, status = _select(method, frame["candidates"], parsed_config, full)
                runtime_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
                target = _selected_target(selected)
                previous = previous_selected[method]
                displacement_m = (
                    math.dist(previous["position_m"], target["position_m"])
                    if previous is not None and target is not None
                    else None
                )
                per_frame.append(
                    {
                        "sequence_id": sequence_id,
                        "frame_index": frame["frame_index"],
                        "stamp_s": frame["stamp_s"],
                        "map_sha256": frame["map_sha256"],
                        "method": method,
                        "candidate_count": len(frame["candidates"]),
                        "selected_target": target,
                        "terrain_valid_output": bool(
                            target is not None and target["terrain_valid"] is True
                        ),
                        "dropout": target is None,
                        "interframe_displacement_m": displacement_m,
                        "switched": bool(
                            displacement_m is not None
                            and displacement_m > parsed_config.switch_distance_m
                        ),
                        "downstream_planner_evaluated": bool(
                            target is not None
                            and target["downstream_planner_accepted"] is not None
                        ),
                        "downstream_planner_accepted": (
                            None
                            if target is None
                            else target["downstream_planner_accepted"]
                        ),
                        "status": status,
                        "runtime_ms": runtime_ms,
                    }
                )
                previous_selected[method] = target
    return {
        "manifest": {
            "schema_version": "tadps_benchmark_manifest.v1",
            "input_replay_sha256": _canonical_sha256(replay),
            "config_sha256": _canonical_sha256(config),
            "methods": list(METHODS),
            "motion_commands_emitted": 0,
        },
        "per_frame": per_frame,
        "summary": _build_summary(per_frame, parsed_config),
    }


@dataclass(frozen=True)
class _Config:
    expected_frame_id: str
    minimum_candidate_score: float
    minimum_stable_frames: int
    maximum_target_jump_m: float
    maximum_candidate_dropout_frames: int
    switch_score_margin: float
    switch_distance_m: float
    score_weights: Mapping[str, float]


class _FullTadpsSelector:
    def __init__(self, config: _Config) -> None:
        self.config = config
        self.current: dict[str, Any] | None = None
        self.pending: dict[str, Any] | None = None
        self.pending_frames = 0
        self.dropout_frames = 0

    def select(self, candidates: tuple[dict[str, Any], ...]) -> tuple[dict[str, Any] | None, str]:
        scored = _scored_candidates(candidates, self.config)
        if not scored:
            self.dropout_frames += 1
            if self.dropout_frames <= self.config.maximum_candidate_dropout_frames:
                if self.current is not None:
                    return self.current, "valid_held"
                if self.pending is not None:
                    return None, "stabilizing_dropout"
            self.current = None
            self.pending = None
            self.pending_frames = 0
            return None, "no_valid_candidate"
        self.dropout_frames = 0
        best = scored[0]
        if self.current is None:
            self._update_pending(best)
            if self.pending_frames < self.config.minimum_stable_frames:
                return None, "stabilizing"
            self.current = self.pending
            self.pending = None
            self.pending_frames = 0
            return self.current, "valid"
        nearest = min(scored, key=lambda item: _distance(item, self.current))
        if _distance(nearest, self.current) > self.config.maximum_target_jump_m:
            self.current = None
            self.pending = None
            self.pending_frames = 0
            self._update_pending(best)
            return None, "stabilizing"
        if (
            _distance(best, nearest) > self.config.maximum_target_jump_m
            and best["score"] >= nearest["score"] + self.config.switch_score_margin
        ):
            self._update_pending(best)
            if self.pending_frames >= self.config.minimum_stable_frames:
                self.current = self.pending
                self.pending = None
                self.pending_frames = 0
            else:
                self.current = nearest
        else:
            self.current = nearest
            self.pending = None
            self.pending_frames = 0
        return self.current, "valid"

    def _update_pending(self, candidate: dict[str, Any]) -> None:
        if self.pending is not None and _distance(candidate, self.pending) <= self.config.maximum_target_jump_m:
            self.pending = candidate
            self.pending_frames += 1
        else:
            self.pending = candidate
            self.pending_frames = 1


def _select(
    method: str,
    candidates: tuple[dict[str, Any], ...],
    config: _Config,
    full: _FullTadpsSelector,
) -> tuple[dict[str, Any] | None, str]:
    if method == "highest_point":
        if not candidates:
            return None, "no_candidate"
        return (
            max(candidates, key=lambda item: item["features"]["surface_height_m"]),
            "selected",
        )
    if method == "score_only":
        scored = _scored_candidates(candidates, config)
        return (scored[0], "valid") if scored else (None, "no_valid_candidate")
    return full.select(candidates)


def _scored_candidates(
    candidates: tuple[dict[str, Any], ...], config: _Config
) -> list[dict[str, Any]]:
    scored: list[dict[str, Any]] = []
    for candidate in candidates:
        features = candidate["features"]
        if not features["terrain_valid"]:
            continue
        weights = config.score_weights
        score = max(
            0.0,
            min(
                1.0,
                weights["local_soil_height"] * features["local_soil_height_score"]
                + weights["confidence"] * features["confidence_score"]
                + weights["edge_clearance"] * features["edge_clearance_score"]
                + weights["relative_height"] * features["relative_height_score"]
                + weights["roughness_penalty"] * features["roughness_penalty"],
            ),
        )
        if score >= config.minimum_candidate_score:
            scored.append({**candidate, "score": score})
    return sorted(scored, key=lambda item: (-item["score"], item["candidate_id"]))


def _selected_target(candidate: dict[str, Any] | None) -> dict[str, Any] | None:
    if candidate is None:
        return None
    return {
        "candidate_id": candidate["candidate_id"],
        "position_m": list(candidate["position_m"]),
        "terrain_valid": candidate["features"]["terrain_valid"],
        "score": candidate.get("score"),
        "downstream_planner_accepted": candidate["downstream_planner_accepted"],
    }


def _parse_config(value: Mapping[str, Any]) -> _Config:
    expected = {
        "schema_version",
        "expected_frame_id",
        "minimum_candidate_score",
        "minimum_stable_frames",
        "maximum_target_jump_m",
        "maximum_candidate_dropout_frames",
        "switch_score_margin",
        "switch_distance_m",
        "score_weights",
    }
    if not isinstance(value, Mapping) or set(value) != expected:
        raise TadpsBenchmarkError("benchmark config fields are invalid")
    if value["schema_version"] != "tadps_benchmark_config.v1":
        raise TadpsBenchmarkError("unsupported benchmark config schema")
    frame = value["expected_frame_id"]
    if not isinstance(frame, str) or not frame:
        raise TadpsBenchmarkError("expected_frame_id is invalid")
    raw_weights = value["score_weights"]
    weight_fields = {
        "local_soil_height",
        "confidence",
        "edge_clearance",
        "relative_height",
        "roughness_penalty",
    }
    if not isinstance(raw_weights, Mapping) or set(raw_weights) != weight_fields:
        raise TadpsBenchmarkError("score_weights fields are invalid")
    weights = {
        field: _raw_number(field, raw_weights[field], -10.0, 10.0)
        for field in weight_fields
    }
    return _Config(
        expected_frame_id=frame,
        minimum_candidate_score=_number(value, "minimum_candidate_score", 0.0, 1.0),
        minimum_stable_frames=_integer(value, "minimum_stable_frames", 1, 1000),
        maximum_target_jump_m=_number(value, "maximum_target_jump_m", 1e-9, 100.0),
        maximum_candidate_dropout_frames=_integer(
            value, "maximum_candidate_dropout_frames", 0, 1000
        ),
        switch_score_margin=_number(value, "switch_score_margin", 0.0, 1.0),
        switch_distance_m=_number(value, "switch_distance_m", 1e-9, 100.0),
        score_weights=weights,
    )


def _parse_replay(
    value: Mapping[str, Any], *, expected_frame_id: str
) -> tuple[tuple[str, tuple[dict[str, Any], ...]], ...]:
    if not isinstance(value, Mapping) or set(value) != {"schema_version", "frame_id", "sequences"}:
        raise TadpsBenchmarkError("candidate replay fields are invalid")
    if value["schema_version"] != "tadps_candidate_replay.v1":
        raise TadpsBenchmarkError("unsupported candidate replay schema")
    if value["frame_id"] != expected_frame_id:
        raise TadpsBenchmarkError("candidate replay frame does not match config")
    raw_sequences = value["sequences"]
    if not isinstance(raw_sequences, list) or not raw_sequences:
        raise TadpsBenchmarkError("candidate replay sequences must be non-empty")
    result = []
    sequence_ids: set[str] = set()
    for sequence in raw_sequences:
        if not isinstance(sequence, Mapping) or set(sequence) != {"sequence_id", "frames"}:
            raise TadpsBenchmarkError("candidate replay sequence fields are invalid")
        sequence_id = sequence["sequence_id"]
        if not isinstance(sequence_id, str) or not sequence_id or sequence_id in sequence_ids:
            raise TadpsBenchmarkError("candidate replay sequence_id is invalid")
        sequence_ids.add(sequence_id)
        raw_frames = sequence["frames"]
        if not isinstance(raw_frames, list) or not raw_frames:
            raise TadpsBenchmarkError("candidate replay frames must be non-empty")
        frames = tuple(_parse_frame(item) for item in raw_frames)
        indices = [item["frame_index"] for item in frames]
        stamps = [item["stamp_s"] for item in frames]
        if indices != sorted(indices) or len(indices) != len(set(indices)):
            raise TadpsBenchmarkError("frame indices must be strictly increasing")
        if stamps != sorted(stamps) or len(stamps) != len(set(stamps)):
            raise TadpsBenchmarkError("frame stamps must be strictly increasing")
        result.append((sequence_id, frames))
    return tuple(result)


def _parse_frame(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != {"frame_index", "stamp_s", "map_sha256", "candidates"}:
        raise TadpsBenchmarkError("candidate replay frame fields are invalid")
    index = _raw_integer("frame_index", value["frame_index"], 0, 2**63 - 1)
    stamp = _raw_number("stamp_s", value["stamp_s"], 0.0, float("inf"))
    digest = value["map_sha256"]
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        raise TadpsBenchmarkError("map_sha256 is invalid")
    raw_candidates = value["candidates"]
    if not isinstance(raw_candidates, list):
        raise TadpsBenchmarkError("candidates must be a list")
    candidates = tuple(_parse_candidate(item) for item in raw_candidates)
    ids = [item["candidate_id"] for item in candidates]
    if len(ids) != len(set(ids)):
        raise TadpsBenchmarkError("candidate ids must be unique within a frame")
    return {"frame_index": index, "stamp_s": stamp, "map_sha256": digest, "candidates": candidates}


def _parse_candidate(value: Any) -> dict[str, Any]:
    expected = {"candidate_id", "position_m", "features", "downstream_planner_accepted"}
    if not isinstance(value, Mapping) or set(value) != expected:
        raise TadpsBenchmarkError("candidate fields are invalid")
    candidate_id = value["candidate_id"]
    if not isinstance(candidate_id, str) or not candidate_id:
        raise TadpsBenchmarkError("candidate_id is invalid")
    position = value["position_m"]
    if not isinstance(position, list) or len(position) != 3:
        raise TadpsBenchmarkError("candidate position_m is invalid")
    position_m = tuple(_raw_number("position_m", item, -1e6, 1e6) for item in position)
    features = value["features"]
    feature_fields = {
        "terrain_valid",
        "surface_height_m",
        "local_soil_height_score",
        "confidence_score",
        "edge_clearance_score",
        "relative_height_score",
        "roughness_penalty",
    }
    if not isinstance(features, Mapping) or set(features) != feature_fields:
        raise TadpsBenchmarkError("candidate feature fields are invalid")
    if not isinstance(features["terrain_valid"], bool):
        raise TadpsBenchmarkError("terrain_valid must be boolean")
    parsed_features = {"terrain_valid": features["terrain_valid"]}
    parsed_features["surface_height_m"] = _raw_number(
        "surface_height_m", features["surface_height_m"], -1e6, 1e6
    )
    for field in feature_fields - {"terrain_valid", "surface_height_m"}:
        parsed_features[field] = _raw_number(field, features[field], 0.0, 1.0)
    planner = value["downstream_planner_accepted"]
    if planner is not None and not isinstance(planner, bool):
        raise TadpsBenchmarkError("downstream_planner_accepted must be boolean or null")
    return {
        "candidate_id": candidate_id,
        "position_m": position_m,
        "features": parsed_features,
        "downstream_planner_accepted": planner,
    }


def _number(value: Mapping[str, Any], field: str, minimum: float, maximum: float) -> float:
    return _raw_number(field, value[field], minimum, maximum)


def _integer(value: Mapping[str, Any], field: str, minimum: int, maximum: int) -> int:
    return _raw_integer(field, value[field], minimum, maximum)


def _raw_number(field: str, value: Any, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TadpsBenchmarkError(f"{field} must be a finite number")
    number = float(value)
    if not minimum <= number <= maximum:
        raise TadpsBenchmarkError(f"{field} is out of range")
    return number


def _raw_integer(field: str, value: Any, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise TadpsBenchmarkError(f"{field} is out of range")
    return value


def _distance(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    return math.dist(first["position_m"], second["position_m"])


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _build_summary(rows: list[dict[str, Any]], config: _Config) -> dict[str, Any]:
    methods: dict[str, Any] = {}
    sequence_ids = tuple(dict.fromkeys(row["sequence_id"] for row in rows))
    for method in METHODS:
        sequence_metrics = [
            _sequence_metrics(
                sequence_id,
                [
                    row
                    for row in rows
                    if row["method"] == method and row["sequence_id"] == sequence_id
                ],
                config,
            )
            for sequence_id in sequence_ids
        ]
        runtimes = [
            row["runtime_ms"] for row in rows if row["method"] == method
        ]
        methods[method] = {
            "aggregation_unit": "sequence",
            "sequence_count": len(sequence_metrics),
            "frame_count": sum(item["frame_count"] for item in sequence_metrics),
            "output_count": sum(item["output_count"] for item in sequence_metrics),
            "terrain_valid_output_count": sum(
                item["terrain_valid_output_count"] for item in sequence_metrics
            ),
            "terrain_valid_output_rate": _optional_mean(
                [
                    item["terrain_valid_output_rate"]
                    for item in sequence_metrics
                    if item["terrain_valid_output_rate"] is not None
                ]
            ),
            "terrain_valid_frame_rate": _mean(
                [item["terrain_valid_frame_rate"] for item in sequence_metrics]
            ),
            "dropout_frame_count": sum(
                item["dropout_frame_count"] for item in sequence_metrics
            ),
            "dropout_rate": _mean(
                [item["dropout_rate"] for item in sequence_metrics]
            ),
            "longest_dropout_frames": max(
                (item["longest_dropout_frames"] for item in sequence_metrics),
                default=0,
            ),
            "mean_interframe_displacement_m": _optional_mean(
                [
                    item["mean_interframe_displacement_m"]
                    for item in sequence_metrics
                    if item["mean_interframe_displacement_m"] is not None
                ]
            ),
            "switch_count": sum(item["switch_count"] for item in sequence_metrics),
            "switch_rate": _optional_mean(
                [
                    item["switch_rate"]
                    for item in sequence_metrics
                    if item["switch_rate"] is not None
                ]
            ),
            "downstream_planner_evaluated_count": sum(
                item["downstream_planner_evaluated_count"]
                for item in sequence_metrics
            ),
            "downstream_planner_accept_count": sum(
                item["downstream_planner_accept_count"]
                for item in sequence_metrics
            ),
            "downstream_planner_accept_rate": _optional_mean(
                [
                    item["downstream_planner_accept_rate"]
                    for item in sequence_metrics
                    if item["downstream_planner_accept_rate"] is not None
                ]
            ),
            "runtime_ms": {
                "p50": _percentile(runtimes, 0.50),
                "p95": _percentile(runtimes, 0.95),
                "max": max(runtimes, default=0.0),
            },
            "sequences": sequence_metrics,
        }
    return {
        "schema_version": "tadps_benchmark_summary.v1",
        "aggregation_unit": "sequence",
        "methods": methods,
    }


def _sequence_metrics(
    sequence_id: str,
    rows: list[dict[str, Any]],
    config: _Config,
) -> dict[str, Any]:
    selected = [row["selected_target"] for row in rows]
    valid_count = sum(
        target is not None and target["terrain_valid"] is True for target in selected
    )
    dropout_flags = [target is None for target in selected]
    longest_dropout = 0
    current_dropout = 0
    for dropout in dropout_flags:
        current_dropout = current_dropout + 1 if dropout else 0
        longest_dropout = max(longest_dropout, current_dropout)

    displacements = []
    for previous, current in zip(selected, selected[1:]):
        if previous is not None and current is not None:
            displacements.append(
                math.dist(previous["position_m"], current["position_m"])
            )
    switches = sum(
        distance > config.switch_distance_m for distance in displacements
    )
    planner_values = [
        target["downstream_planner_accepted"]
        for target in selected
        if target is not None
        and target["downstream_planner_accepted"] is not None
    ]
    frame_count = len(rows)
    output_count = frame_count - sum(dropout_flags)
    return {
        "sequence_id": sequence_id,
        "frame_count": frame_count,
        "output_count": output_count,
        "terrain_valid_output_count": valid_count,
        "terrain_valid_output_rate": (
            valid_count / output_count if output_count else None
        ),
        "terrain_valid_frame_rate": valid_count / frame_count,
        "dropout_frame_count": sum(dropout_flags),
        "dropout_rate": sum(dropout_flags) / frame_count,
        "longest_dropout_frames": longest_dropout,
        "interframe_displacement_count": len(displacements),
        "mean_interframe_displacement_m": _optional_mean(displacements),
        "switch_count": switches,
        "switch_rate": (
            switches / len(displacements) if displacements else None
        ),
        "downstream_planner_evaluated_count": len(planner_values),
        "downstream_planner_accept_count": sum(planner_values),
        "downstream_planner_accept_rate": (
            sum(planner_values) / len(planner_values) if planner_values else None
        ),
    }


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _optional_mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]
