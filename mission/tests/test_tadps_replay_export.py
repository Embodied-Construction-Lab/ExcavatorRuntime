import copy
import json
from pathlib import Path

import pytest

from mission.tadps_replay_export import (
    TadpsReplayExportError,
    export_tadps_candidate_replay,
)
from mission.scripts.export_tadps_candidate_replay import main as export_cli_main


HEX_A = "a" * 64
HEX_B = "b" * 64


def _candidate(candidate_id: str, *, accepted: bool | None) -> dict:
    return {
        "candidate_id": candidate_id,
        "position_m": [1.0, 0.2, 0.4],
        "features": {
            "terrain_valid": True,
            "surface_height_m": 0.4,
            "local_soil_height_score": 0.8,
            "confidence_score": 0.9,
            "edge_clearance_score": 0.7,
            "relative_height_score": 0.6,
            "roughness_penalty": 0.1,
        },
        "downstream_planner_accepted": accepted,
    }


def _record(
    sequence_id: str,
    frame_index: int,
    *,
    map_sha256: str,
    candidates: list[dict],
) -> dict:
    return {
        "schema_version": "tadps_selector_candidate_frame.v1",
        "record_type": "complete_preselection_candidate_set",
        "sequence_id": sequence_id,
        "frame_id": "world",
        "frame_index": frame_index,
        "stamp_s": float(frame_index + 1),
        "map_sha256": map_sha256,
        "candidate_set_complete": True,
        "candidates": candidates,
    }


def _write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


def test_complete_candidate_trace_exports_strict_replay_without_losing_evidence(
    tmp_path: Path,
) -> None:
    trace = tmp_path / "selector.jsonl"
    first = _candidate("pile-1-cell-10-20", accepted=True)
    second = _candidate("pile-1-cell-11-20", accepted=False)
    _write_jsonl(
        trace,
        [
            _record("pile-a", 0, map_sha256=HEX_A, candidates=[first, second]),
            _record("pile-b", 0, map_sha256=HEX_B, candidates=[]),
        ],
    )

    replay = export_tadps_candidate_replay(trace)

    assert replay == {
        "schema_version": "tadps_candidate_replay.v1",
        "frame_id": "world",
        "sequences": [
            {
                "sequence_id": "pile-a",
                "frames": [
                    {
                        "frame_index": 0,
                        "stamp_s": 1.0,
                        "map_sha256": HEX_A,
                        "candidates": [first, second],
                    }
                ],
            },
            {
                "sequence_id": "pile-b",
                "frames": [
                    {
                        "frame_index": 0,
                        "stamp_s": 1.0,
                        "map_sha256": HEX_B,
                        "candidates": [],
                    }
                ],
            },
        ],
    }


def test_final_selected_point_log_is_rejected_instead_of_inventing_candidates(
    tmp_path: Path,
) -> None:
    trace = tmp_path / "selected-only.jsonl"
    _write_jsonl(
        trace,
        [
            {
                "schema_version": "entry_point.v1",
                "sequence_id": "pile-a",
                "frame_id": "world",
                "frame_index": 0,
                "stamp_s": 1.0,
                "map_sha256": HEX_A,
                "selected_point_m": [1.0, 0.2, 0.4],
            }
        ],
    )

    with pytest.raises(
        TadpsReplayExportError,
        match="future logger must record complete preselection candidate sets",
    ):
        export_tadps_candidate_replay(trace)


def test_candidate_without_planner_result_is_rejected(tmp_path: Path) -> None:
    trace = tmp_path / "missing-planner-result.jsonl"
    candidate = _candidate("candidate", accepted=True)
    del candidate["downstream_planner_accepted"]
    _write_jsonl(
        trace,
        [_record("pile-a", 0, map_sha256=HEX_A, candidates=[candidate])],
    )

    with pytest.raises(
        TadpsReplayExportError,
        match="downstream_planner_accepted",
    ):
        export_tadps_candidate_replay(trace)


def test_explicitly_incomplete_candidate_set_is_rejected(tmp_path: Path) -> None:
    trace = tmp_path / "truncated-candidates.jsonl"
    record = _record(
        "pile-a",
        0,
        map_sha256=HEX_A,
        candidates=[_candidate("visible-only", accepted=None)],
    )
    record["candidate_set_complete"] = False
    _write_jsonl(trace, [record])

    with pytest.raises(
        TadpsReplayExportError,
        match="complete preselection candidate sets",
    ):
        export_tadps_candidate_replay(trace)


def test_duplicate_candidate_ids_in_one_frame_are_rejected(tmp_path: Path) -> None:
    trace = tmp_path / "duplicate-candidates.jsonl"
    _write_jsonl(
        trace,
        [
            _record(
                "pile-a",
                0,
                map_sha256=HEX_A,
                candidates=[
                    _candidate("same", accepted=True),
                    _candidate("same", accepted=False),
                ],
            )
        ],
    )

    with pytest.raises(TadpsReplayExportError, match="candidate_id.*unique"):
        export_tadps_candidate_replay(trace)


def test_frames_must_be_strictly_ordered_within_each_sequence(tmp_path: Path) -> None:
    trace = tmp_path / "out-of-order.jsonl"
    _write_jsonl(
        trace,
        [
            _record("pile-a", 1, map_sha256=HEX_A, candidates=[]),
            _record("pile-a", 0, map_sha256=HEX_B, candidates=[]),
        ],
    )

    with pytest.raises(TadpsReplayExportError, match="strictly increasing"):
        export_tadps_candidate_replay(trace)


def test_empty_trace_is_rejected_with_domain_error(tmp_path: Path) -> None:
    trace = tmp_path / "empty.jsonl"
    trace.write_text("", encoding="utf-8")

    with pytest.raises(TadpsReplayExportError, match="must contain at least one"):
        export_tadps_candidate_replay(trace)


def test_all_frames_must_use_one_coordinate_frame(tmp_path: Path) -> None:
    trace = tmp_path / "mixed-frames.jsonl"
    second = _record("pile-a", 1, map_sha256=HEX_B, candidates=[])
    second["frame_id"] = "map"
    _write_jsonl(
        trace,
        [
            _record("pile-a", 0, map_sha256=HEX_A, candidates=[]),
            second,
        ],
    )

    with pytest.raises(TadpsReplayExportError, match="frame_id.*same"):
        export_tadps_candidate_replay(trace)


def test_unknown_frame_fields_are_rejected(tmp_path: Path) -> None:
    trace = tmp_path / "unknown-field.jsonl"
    record = _record("pile-a", 0, map_sha256=HEX_A, candidates=[])
    record["selected_point_m"] = [1.0, 0.2, 0.4]
    _write_jsonl(trace, [record])

    with pytest.raises(TadpsReplayExportError, match="frame fields are invalid"):
        export_tadps_candidate_replay(trace)


def test_candidate_fields_and_normalized_features_are_strict(tmp_path: Path) -> None:
    trace = tmp_path / "invalid-candidate.jsonl"
    candidate = _candidate("candidate", accepted=True)
    candidate["features"]["confidence_score"] = 1.01
    _write_jsonl(
        trace,
        [_record("pile-a", 0, map_sha256=HEX_A, candidates=[candidate])],
    )

    with pytest.raises(TadpsReplayExportError, match="confidence_score.*range"):
        export_tadps_candidate_replay(trace)


def test_cli_writes_replay_and_source_bound_export_manifest(tmp_path: Path) -> None:
    trace = tmp_path / "selector.jsonl"
    _write_jsonl(
        trace,
        [
            _record(
                "pile-a",
                0,
                map_sha256=HEX_A,
                candidates=[_candidate("candidate", accepted=None)],
            )
        ],
    )
    output_dir = tmp_path / "frozen-replay"

    assert (
        export_cli_main(
            ["--input-jsonl", str(trace), "--output-dir", str(output_dir)]
        )
        == 0
    )

    replay = json.loads(
        (output_dir / "candidate_replay.json").read_text(encoding="utf-8")
    )
    manifest = json.loads(
        (output_dir / "export_manifest.json").read_text(encoding="utf-8")
    )
    assert replay["sequences"][0]["frames"][0]["candidates"][0][
        "downstream_planner_accepted"
    ] is None
    assert manifest["schema_version"] == "tadps_replay_export_manifest.v1"
    assert manifest["input_jsonl_sha256"]
    assert manifest["output_replay_sha256"]
    assert manifest["sequence_count"] == 1
    assert manifest["frame_count"] == 1
    assert manifest["candidate_count"] == 1
    assert manifest["reconstructed_from_selected_points"] is False
    assert manifest["motion_commands_emitted"] == 0


def test_cli_fails_closed_without_leaving_partial_output(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    trace = tmp_path / "selected-only.jsonl"
    _write_jsonl(trace, [{"selected_point_m": [1.0, 0.2, 0.4]}])
    output_dir = tmp_path / "must-not-exist"

    assert (
        export_cli_main(
            ["--input-jsonl", str(trace), "--output-dir", str(output_dir)]
        )
        == 2
    )

    assert not output_dir.exists()
    assert "complete preselection candidate sets" in capsys.readouterr().err


def test_repository_logger_contract_example_exports(tmp_path: Path) -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "tadps_selector_candidate_frame.v1.example.jsonl"
    )

    replay = export_tadps_candidate_replay(example)

    assert replay["schema_version"] == "tadps_candidate_replay.v1"
    assert replay["sequences"][0]["frames"][0]["candidates"]


def test_strict_record_boundary_rejects_malformed_scientific_fields(
    tmp_path: Path,
) -> None:
    base = _record(
        "pile-a",
        0,
        map_sha256=HEX_A,
        candidates=[_candidate("candidate", accepted=True)],
    )
    cases: list[tuple[dict, str]] = []

    bad_sequence = copy.deepcopy(base)
    bad_sequence["sequence_id"] = ""
    cases.append((bad_sequence, "sequence_id"))
    bad_index = copy.deepcopy(base)
    bad_index["frame_index"] = True
    cases.append((bad_index, "frame_index"))
    bad_stamp = copy.deepcopy(base)
    bad_stamp["stamp_s"] = -1.0
    cases.append((bad_stamp, "stamp_s"))
    bad_hash = copy.deepcopy(base)
    bad_hash["map_sha256"] = "A" * 64
    cases.append((bad_hash, "map_sha256"))
    bad_candidates = copy.deepcopy(base)
    bad_candidates["candidates"] = {}
    cases.append((bad_candidates, "candidates"))
    bad_position = copy.deepcopy(base)
    bad_position["candidates"][0]["position_m"] = [1.0, 0.2]
    cases.append((bad_position, "position_m"))
    bad_features = copy.deepcopy(base)
    del bad_features["candidates"][0]["features"]["roughness_penalty"]
    cases.append((bad_features, "feature fields"))
    bad_terrain = copy.deepcopy(base)
    bad_terrain["candidates"][0]["features"]["terrain_valid"] = 1
    cases.append((bad_terrain, "terrain_valid"))
    bad_planner = copy.deepcopy(base)
    bad_planner["candidates"][0]["downstream_planner_accepted"] = "yes"
    cases.append((bad_planner, "downstream_planner_accepted"))

    for index, (record, message) in enumerate(cases):
        trace = tmp_path / f"bad-{index}.jsonl"
        _write_jsonl(trace, [record])
        with pytest.raises(TadpsReplayExportError, match=message):
            export_tadps_candidate_replay(trace)


def test_jsonl_framing_is_strict(tmp_path: Path) -> None:
    blank = tmp_path / "blank.jsonl"
    blank.write_text("\n", encoding="utf-8")
    with pytest.raises(TadpsReplayExportError, match="must not be blank"):
        export_tadps_candidate_replay(blank)

    malformed = tmp_path / "malformed.jsonl"
    malformed.write_text("{\n", encoding="utf-8")
    with pytest.raises(TadpsReplayExportError, match="not valid JSON"):
        export_tadps_candidate_replay(malformed)

    duplicate = tmp_path / "duplicate.jsonl"
    duplicate.write_text('{"frame_id":"world","frame_id":"map"}\n', encoding="utf-8")
    with pytest.raises(TadpsReplayExportError, match="duplicate JSON field"):
        export_tadps_candidate_replay(duplicate)
