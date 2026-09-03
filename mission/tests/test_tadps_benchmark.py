import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from mission.tadps_benchmark import TadpsBenchmarkError, evaluate_tadps_replay
from mission.scripts.evaluate_tadps_replay import main as evaluate_cli_main


HEX_A = "a" * 64


def _config() -> dict:
    return {
        "schema_version": "tadps_benchmark_config.v1",
        "expected_frame_id": "world",
        "minimum_candidate_score": 0.45,
        "minimum_stable_frames": 2,
        "maximum_target_jump_m": 0.15,
        "maximum_candidate_dropout_frames": 1,
        "switch_score_margin": 0.10,
        "switch_distance_m": 0.10,
        "score_weights": {
            "local_soil_height": 0.35,
            "confidence": 0.25,
            "edge_clearance": 0.20,
            "relative_height": 0.10,
            "roughness_penalty": -0.10,
        },
    }


def _candidate(
    candidate_id: str,
    *,
    x: float,
    height: float,
    score_features: tuple[float, float, float, float, float],
    terrain_valid: bool = True,
    planner_accepted: bool | None = True,
) -> dict:
    soil, confidence, edge, relative, roughness = score_features
    return {
        "candidate_id": candidate_id,
        "position_m": [x, 0.0, height],
        "features": {
            "terrain_valid": terrain_valid,
            "surface_height_m": height,
            "local_soil_height_score": soil,
            "confidence_score": confidence,
            "edge_clearance_score": edge,
            "relative_height_score": relative,
            "roughness_penalty": roughness,
        },
        "downstream_planner_accepted": planner_accepted,
    }


def _frame(index: int, candidates: list[dict]) -> dict:
    return {
        "frame_index": index,
        "stamp_s": float(index),
        "map_sha256": HEX_A,
        "candidates": candidates,
    }


def _replay(frames: list[dict]) -> dict:
    return {
        "schema_version": "tadps_candidate_replay.v1",
        "frame_id": "world",
        "sequences": [{"sequence_id": "pile-a", "frames": frames}],
    }


def test_repository_benchmark_config_is_valid() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config" / "tadps_benchmark.v1.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    candidate = _candidate(
        "candidate",
        x=0.0,
        height=0.5,
        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
    )

    result = evaluate_tadps_replay(_replay([_frame(0, [candidate])]), config)

    assert result["manifest"]["config_sha256"]


def test_one_frozen_frame_runs_all_three_offline_selectors_without_motion() -> None:
    replay = _replay(
        [
            _frame(
                0,
                [
                    _candidate(
                        "low-good",
                        x=0.0,
                        height=0.4,
                        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
                    ),
                    _candidate(
                        "high-rough",
                        x=0.5,
                        height=0.9,
                        score_features=(0.5, 0.5, 0.5, 0.5, 1.0),
                    ),
                ],
            )
        ]
    )

    result = evaluate_tadps_replay(replay, _config())

    assert result["manifest"]["schema_version"] == "tadps_benchmark_manifest.v1"
    assert result["manifest"]["motion_commands_emitted"] == 0
    assert result["manifest"]["methods"] == [
        "highest_point",
        "score_only",
        "full_tadps",
    ]
    assert result["per_frame"][0]["selected_target"]["candidate_id"] == "high-rough"
    assert result["per_frame"][1]["selected_target"]["candidate_id"] == "low-good"
    assert result["per_frame"][2]["selected_target"] is None


def test_score_only_uses_frozen_config_weights() -> None:
    config = _config()
    config["score_weights"] = {
        "local_soil_height": 0.0,
        "confidence": 1.0,
        "edge_clearance": 0.0,
        "relative_height": 0.0,
        "roughness_penalty": 0.0,
    }
    result = evaluate_tadps_replay(
        _replay(
            [
                _frame(
                    0,
                    [
                        _candidate(
                            "soil",
                            x=0.0,
                            height=0.5,
                            score_features=(1.0, 0.5, 1.0, 1.0, 0.0),
                        ),
                        _candidate(
                            "confidence",
                            x=0.1,
                            height=0.4,
                            score_features=(0.0, 0.9, 0.0, 0.0, 0.0),
                        ),
                    ],
                )
            ]
        ),
        config,
    )

    score_row = next(row for row in result["per_frame"] if row["method"] == "score_only")
    assert score_row["selected_target"]["candidate_id"] == "confidence"
    assert score_row["selected_target"]["score"] == pytest.approx(0.9)


def test_highest_point_baseline_does_not_apply_tadps_terrain_filter() -> None:
    result = evaluate_tadps_replay(
        _replay(
            [
                _frame(
                    0,
                    [
                        _candidate(
                            "highest-invalid",
                            x=0.0,
                            height=0.9,
                            score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
                            terrain_valid=False,
                        ),
                        _candidate(
                            "lower-valid",
                            x=0.1,
                            height=0.5,
                            score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
                        ),
                    ],
                )
            ]
        ),
        _config(),
    )

    highest = next(
        row for row in result["per_frame"] if row["method"] == "highest_point"
    )
    score_only = next(
        row for row in result["per_frame"] if row["method"] == "score_only"
    )
    assert highest["selected_target"]["candidate_id"] == "highest-invalid"
    assert highest["terrain_valid_output"] is False
    assert score_only["selected_target"]["candidate_id"] == "lower-valid"
    assert (
        result["summary"]["methods"]["highest_point"]["terrain_valid_output_count"]
        == 0
    )


def test_full_tadps_requires_confirmation_and_hysteresis_before_switching() -> None:
    stable = _candidate(
        "stable",
        x=0.0,
        height=0.5,
        score_features=(0.7, 0.7, 0.7, 0.7, 0.0),
    )
    challenger = _candidate(
        "challenger",
        x=0.5,
        height=0.6,
        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
    )
    result = evaluate_tadps_replay(
        _replay(
            [
                _frame(0, [stable]),
                _frame(1, [stable]),
                _frame(2, [stable, challenger]),
                _frame(3, [stable, challenger]),
            ]
        ),
        _config(),
    )
    full_rows = [row for row in result["per_frame"] if row["method"] == "full_tadps"]

    assert [
        None if row["selected_target"] is None else row["selected_target"]["candidate_id"]
        for row in full_rows
    ] == [None, "stable", "stable", "challenger"]
    assert [row["switched"] for row in full_rows] == [False, False, False, True]
    assert full_rows[-1]["interframe_displacement_m"] == pytest.approx(
        (0.5**2 + 0.1**2) ** 0.5
    )
    assert all(row["candidate_count"] in {1, 2} for row in full_rows)


def test_full_tadps_preserves_pending_confirmation_across_bounded_dropout() -> None:
    config = _config()
    config["minimum_stable_frames"] = 3
    candidate = _candidate(
        "stable",
        x=0.0,
        height=0.5,
        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
    )
    result = evaluate_tadps_replay(
        _replay(
            [
                _frame(0, [candidate]),
                _frame(1, []),
                _frame(2, [candidate]),
                _frame(3, [candidate]),
            ]
        ),
        config,
    )
    full_rows = [row for row in result["per_frame"] if row["method"] == "full_tadps"]

    assert [row["status"] for row in full_rows] == [
        "stabilizing",
        "stabilizing_dropout",
        "stabilizing",
        "valid",
    ]
    assert full_rows[-1]["selected_target"]["candidate_id"] == "stable"


def test_summary_is_sequence_aware_and_separates_terrain_validity_from_planner_acceptance() -> None:
    first = _candidate(
        "a",
        x=0.0,
        height=0.5,
        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
        planner_accepted=False,
    )
    second = _candidate(
        "b",
        x=0.5,
        height=0.6,
        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
        planner_accepted=True,
    )
    result = evaluate_tadps_replay(
        _replay(
            [
                _frame(0, [first]),
                _frame(1, [first]),
                _frame(2, []),
                _frame(3, []),
                _frame(4, [second]),
                _frame(5, [second]),
            ]
        ),
        _config(),
    )

    score = result["summary"]["methods"]["score_only"]
    assert score["aggregation_unit"] == "sequence"
    assert score["frame_count"] == 6
    assert score["output_count"] == 4
    assert score["terrain_valid_output_count"] == 4
    assert score["terrain_valid_output_rate"] == pytest.approx(1.0)
    assert score["terrain_valid_frame_rate"] == pytest.approx(4 / 6)
    assert score["dropout_frame_count"] == 2
    assert score["longest_dropout_frames"] == 2
    assert score["downstream_planner_evaluated_count"] == 4
    assert score["downstream_planner_accept_count"] == 2
    assert score["downstream_planner_accept_rate"] == pytest.approx(0.5)
    assert score["sequences"][0]["sequence_id"] == "pile-a"

    full = result["summary"]["methods"]["full_tadps"]
    assert full["terrain_valid_output_count"] == 3
    assert full["dropout_frame_count"] == 3
    assert full["longest_dropout_frames"] == 2
    assert full["downstream_planner_evaluated_count"] == 3
    assert full["downstream_planner_accept_count"] == 1


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda replay: replay.update({"unexpected": True}), "replay fields"),
        (lambda replay: replay.update({"frame_id": "map"}), "frame does not match"),
        (
            lambda replay: replay["sequences"][0]["frames"][0].update(
                {"map_sha256": "not-a-digest"}
            ),
            "map_sha256",
        ),
        (
            lambda replay: replay["sequences"][0]["frames"][0]["candidates"][0][
                "features"
            ].update({"confidence_score": float("nan")}),
            "confidence_score",
        ),
    ],
)
def test_replay_boundary_rejects_unfrozen_or_nonfinite_inputs(mutate, message) -> None:
    replay = _replay(
        [
            _frame(
                0,
                [
                    _candidate(
                        "candidate",
                        x=0.0,
                        height=0.5,
                        score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
                    )
                ],
            )
        ]
    )
    mutate(replay)

    with pytest.raises(TadpsBenchmarkError, match=message):
        evaluate_tadps_replay(replay, _config())


def test_cli_writes_manifest_per_frame_and_summary_without_motion_surface(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    root = Path(__file__).resolve().parents[2]
    replay_path = tmp_path / "replay.json"
    config_path = tmp_path / "config.json"
    output_dir = tmp_path / "benchmark"
    replay_path.write_text(
        json.dumps(
            _replay(
                [
                    _frame(
                        0,
                        [
                            _candidate(
                                "candidate",
                                x=0.0,
                                height=0.5,
                                score_features=(1.0, 1.0, 1.0, 1.0, 0.0),
                            )
                        ],
                    )
                ]
            )
        ),
        encoding="utf-8",
    )
    config_path.write_text(json.dumps(_config()), encoding="utf-8")
    script = root / "mission" / "scripts" / "evaluate_tadps_replay.py"

    return_code = evaluate_cli_main(
        [
            "--replay",
            str(replay_path),
            "--config",
            str(config_path),
            "--output-dir",
            str(output_dir),
        ],
    )

    assert return_code == 0, capsys.readouterr().err
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    frame_rows = [
        json.loads(line)
        for line in (output_dir / "per_frame.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert manifest["input_file_sha256"] == hashlib.sha256(
        replay_path.read_bytes()
    ).hexdigest()
    assert manifest["config_file_sha256"] == hashlib.sha256(
        config_path.read_bytes()
    ).hexdigest()
    assert manifest["motion_commands_emitted"] == 0
    assert summary["aggregation_unit"] == "sequence"
    assert len(frame_rows) == 3

    help_result = subprocess.run(
        [sys.executable, str(script), "--help"],
        cwd=root,
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0
    assert "motion" not in help_result.stdout.lower()
