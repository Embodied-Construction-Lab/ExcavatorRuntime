import json
from pathlib import Path

import pytest

from mission.tadps_live_capture import (
    TadpsLiveCaptureError,
    TadpsLiveCaptureSession,
    load_selector_bridge_descriptor,
    verify_selector_bridge_provenance,
)


def _candidate_frame(frame_index: int) -> dict:
    return {
        "schema_version": "tadps_selector_candidate_frame.v1",
        "record_type": "complete_preselection_candidate_set",
        "sequence_id": "pile-stage-001",
        "frame_id": "world",
        "frame_index": frame_index,
        "stamp_s": 100.0 + frame_index,
        "map_sha256": "a" * 64,
        "candidate_set_complete": True,
        "candidates": [],
    }


def _source_snapshot() -> dict:
    return {
        "schema_version": "tadps_selector_source_snapshot.v1",
        "bridge_id": "excavator_dig_point_tadps_candidate_trace.v1",
        "descriptor_sha256": "b" * 64,
        "patch_sha256": "c" * 64,
        "source_files": {"src/dig_point_node.cpp": "d" * 64},
        "config_files": {"config/dig_point.yaml": "e" * 64},
    }


def test_capture_writes_append_only_trace_and_source_bound_manifest(
    tmp_path: Path,
) -> None:
    output = tmp_path / "capture"
    provenance = _source_snapshot()

    with TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=provenance,
    ) as capture:
        capture.record_json(json.dumps(_candidate_frame(0)))
        capture.record_json(json.dumps(_candidate_frame(1)))

    trace = output / "candidate_frames.jsonl"
    manifest = json.loads((output / "capture_manifest.json").read_text())
    assert [json.loads(line)["frame_index"] for line in trace.read_text().splitlines()] == [
        0,
        1,
    ]
    assert manifest["schema_version"] == "tadps_live_capture_manifest.v1"
    assert manifest["frame_count"] == 2
    assert manifest["first_frame_index"] == 0
    assert manifest["last_frame_index"] == 1
    assert manifest["selector_source_snapshot"] == provenance
    assert len(manifest["candidate_frames_sha256"]) == 64


def test_bridge_provenance_verifies_patch_source_and_config_hashes(
    tmp_path: Path,
) -> None:
    bridge_directory = tmp_path / "bridge"
    bridge_directory.mkdir()
    patch = bridge_directory / "selector.patch"
    patch.write_text("reproducible patch\n", encoding="utf-8")
    selector = tmp_path / "selector"
    (selector / "src").mkdir(parents=True)
    (selector / "config").mkdir()
    source = selector / "src" / "dig_point_node.cpp"
    config = selector / "config" / "dig_point.yaml"
    source.write_text("patched source\n", encoding="utf-8")
    config.write_text("candidate_trace_enabled: false\n", encoding="utf-8")

    import hashlib

    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    descriptor = bridge_directory / "provenance.json"
    descriptor.write_text(
        json.dumps(
            {
                "schema_version": "tadps_selector_bridge_provenance.v1",
                "bridge_id": "excavator_dig_point_tadps_candidate_trace.v1",
                "package_name": "excavator_dig_point",
                "patch_file": "selector.patch",
                "patch_sha256": sha(patch),
                "base_files": {"src/dig_point_node.cpp": "f" * 64},
                "patched_source_files": {"src/dig_point_node.cpp": sha(source)},
                "patched_config_files": {"config/dig_point.yaml": sha(config)},
                "runtime_contract": {
                    "enabled_parameter": "candidate_trace_enabled",
                    "default_enabled": False,
                    "topic_parameter": "candidate_trace_topic",
                    "default_topic": "/digging/tadps_candidate_frame",
                    "sequence_id_parameter": "candidate_trace_sequence_id",
                    "message_type": "std_msgs/msg/String",
                    "payload_schema": "tadps_selector_candidate_frame.v1",
                    "record_type": "complete_preselection_candidate_set",
                    "motion_commands_emitted": 0,
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    static_descriptor = load_selector_bridge_descriptor(descriptor)
    assert static_descriptor["bridge_id"] == (
        "excavator_dig_point_tadps_candidate_trace.v1"
    )
    assert static_descriptor["patch_sha256"] == sha(patch)

    snapshot = verify_selector_bridge_provenance(
        descriptor_path=descriptor,
        selector_source_root=selector,
    )

    assert snapshot["schema_version"] == "tadps_selector_source_snapshot.v1"
    assert snapshot["patch_sha256"] == sha(patch)
    assert snapshot["source_files"] == {"src/dig_point_node.cpp": sha(source)}
    assert snapshot["config_files"] == {"config/dig_point.yaml": sha(config)}
    assert len(snapshot["descriptor_sha256"]) == 64


def test_capture_rejects_frame_gap_without_publishing_manifest(tmp_path: Path) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )
    capture.record_json(json.dumps(_candidate_frame(0)))

    with pytest.raises(TadpsLiveCaptureError, match="contiguous"):
        capture.record_json(json.dumps(_candidate_frame(2)))
    capture.abort()

    assert not (output / "capture_manifest.json").exists()
    assert len((output / "candidate_frames.jsonl").read_text().splitlines()) == 1


@pytest.mark.parametrize("second_stamp_s", [100.0, 99.0])
def test_capture_rejects_non_increasing_timestamp_before_append(
    tmp_path: Path,
    second_stamp_s: float,
) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )
    capture.record_json(json.dumps(_candidate_frame(0)))
    second = _candidate_frame(1)
    second["stamp_s"] = second_stamp_s

    with pytest.raises(TadpsLiveCaptureError, match="stamp_s.*strictly increasing"):
        capture.record_json(json.dumps(second))
    capture.abort()

    assert len((output / "candidate_frames.jsonl").read_text().splitlines()) == 1


def test_capture_rejects_missing_sequence_head(tmp_path: Path) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )

    with pytest.raises(TadpsLiveCaptureError, match="must start at zero"):
        capture.record_json(json.dumps(_candidate_frame(1)))
    capture.abort()

    assert (output / "candidate_frames.jsonl").read_bytes() == b""


def test_capture_rejects_selected_only_payload_before_append(tmp_path: Path) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )

    with pytest.raises(TadpsLiveCaptureError, match="complete preselection"):
        capture.record_json(json.dumps({"selected_point_m": [1.0, 0.0, 0.0]}))
    capture.abort()

    assert (output / "candidate_frames.jsonl").read_bytes() == b""
    assert not (output / "capture_manifest.json").exists()


def test_capture_rejects_run_or_frame_identity_changes(tmp_path: Path) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )
    wrong_sequence = _candidate_frame(0)
    wrong_sequence["sequence_id"] = "another-run"
    with pytest.raises(TadpsLiveCaptureError, match="sequence_id"):
        capture.record_json(json.dumps(wrong_sequence))

    capture.record_json(json.dumps(_candidate_frame(0)))
    wrong_frame = _candidate_frame(1)
    wrong_frame["frame_id"] = "map"
    with pytest.raises(TadpsLiveCaptureError, match="frame_id changed"):
        capture.record_json(json.dumps(wrong_frame))
    capture.abort()
    with pytest.raises(TadpsLiveCaptureError, match="closed"):
        capture.record_json(json.dumps(_candidate_frame(1)))


def test_empty_capture_and_existing_output_directory_fail_closed(
    tmp_path: Path,
) -> None:
    output = tmp_path / "capture"
    capture = TadpsLiveCaptureSession(
        output_directory=output,
        expected_sequence_id="pile-stage-001",
        source_topic="/digging/tadps_candidate_frame",
        selector_source_snapshot=_source_snapshot(),
    )
    with pytest.raises(TadpsLiveCaptureError, match="contains no candidate frames"):
        capture.close()
    assert not (output / "capture_manifest.json").exists()

    with pytest.raises(FileExistsError):
        TadpsLiveCaptureSession(
            output_directory=output,
            expected_sequence_id="pile-stage-001",
            source_topic="/digging/tadps_candidate_frame",
            selector_source_snapshot=_source_snapshot(),
        )


def test_provenance_rejects_patch_or_source_hash_drift(tmp_path: Path) -> None:
    bridge_directory = tmp_path / "bridge"
    bridge_directory.mkdir()
    patch = bridge_directory / "selector.patch"
    patch.write_text("expected patch\n", encoding="utf-8")
    selector = tmp_path / "selector"
    (selector / "src").mkdir(parents=True)
    (selector / "config").mkdir()
    source = selector / "src" / "dig_point_node.cpp"
    config = selector / "config" / "dig_point.yaml"
    source.write_text("expected source\n", encoding="utf-8")
    config.write_text("expected config\n", encoding="utf-8")

    import hashlib

    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    descriptor = bridge_directory / "provenance.json"
    descriptor.write_text(
        json.dumps(
            {
                "schema_version": "tadps_selector_bridge_provenance.v1",
                "bridge_id": "excavator_dig_point_tadps_candidate_trace.v1",
                "package_name": "excavator_dig_point",
                "patch_file": "selector.patch",
                "patch_sha256": sha(patch),
                "base_files": {"src/dig_point_node.cpp": "f" * 64},
                "patched_source_files": {"src/dig_point_node.cpp": sha(source)},
                "patched_config_files": {"config/dig_point.yaml": sha(config)},
                "runtime_contract": {
                    "enabled_parameter": "candidate_trace_enabled",
                    "default_enabled": False,
                    "topic_parameter": "candidate_trace_topic",
                    "default_topic": "/digging/tadps_candidate_frame",
                    "sequence_id_parameter": "candidate_trace_sequence_id",
                    "message_type": "std_msgs/msg/String",
                    "payload_schema": "tadps_selector_candidate_frame.v1",
                    "record_type": "complete_preselection_candidate_set",
                    "motion_commands_emitted": 0,
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    patch.write_text("tampered patch\n", encoding="utf-8")
    with pytest.raises(TadpsLiveCaptureError, match="patch SHA-256"):
        load_selector_bridge_descriptor(descriptor)

    patch.write_text("expected patch\n", encoding="utf-8")
    source.write_text("tampered source\n", encoding="utf-8")
    with pytest.raises(TadpsLiveCaptureError, match="source.*SHA-256"):
        verify_selector_bridge_provenance(
            descriptor_path=descriptor,
            selector_source_root=selector,
        )


def test_versioned_bridge_descriptor_and_patch_are_self_consistent() -> None:
    descriptor = (
        Path(__file__).resolve().parents[1]
        / "bridges"
        / "excavator_dig_point_tadps_candidate_trace.v1.json"
    )

    value = load_selector_bridge_descriptor(descriptor)

    assert value["bridge_id"] == (
        "excavator_dig_point_tadps_candidate_trace.v1"
    )
    assert value["runtime_contract"]["default_enabled"] is False
    assert value["runtime_contract"]["motion_commands_emitted"] == 0


def test_checked_in_live_seam_example_does_not_invent_planner_acceptance() -> None:
    example = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "tadps_selector_candidate_frame.v1.example.jsonl"
    )
    rows = [json.loads(line) for line in example.read_text().splitlines()]

    assert rows
    assert all(
        candidate["downstream_planner_accepted"] is None
        for row in rows
        for candidate in row["candidates"]
    )
