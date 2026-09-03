#!/usr/bin/env python3
"""Freeze a complete selector-candidate JSONL trace for offline TADPS study."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

AIRY_ROOT = Path(__file__).resolve().parents[2]
if str(AIRY_ROOT) not in sys.path:
    sys.path.insert(0, str(AIRY_ROOT))

from mission.tadps_replay_export import (
    TadpsReplayExportError,
    export_tadps_candidate_replay,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export complete preselection candidate frames to a frozen "
            "tadps_candidate_replay.v1 artifact."
        )
    )
    parser.add_argument("--input-jsonl", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        input_path = arguments.input_jsonl.expanduser().resolve(strict=True)
        input_bytes = input_path.read_bytes()
        replay = export_tadps_candidate_replay(input_path)
        replay_bytes = (
            json.dumps(replay, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
        ).encode("utf-8")
        manifest = _build_manifest(
            replay,
            input_jsonl_sha256=hashlib.sha256(input_bytes).hexdigest(),
            output_replay_sha256=hashlib.sha256(replay_bytes).hexdigest(),
        )
        target = _prepare_output_dir(arguments.output_dir)
        (target / "candidate_replay.json").write_bytes(replay_bytes)
        (target / "export_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    except (OSError, TadpsReplayExportError, ValueError) as exc:
        print(f"TADPS replay export failed: {exc}", file=sys.stderr)
        return 2
    print(str(target))
    return 0


def _prepare_output_dir(path: Path) -> Path:
    target = path.expanduser().resolve()
    if target.exists() and (not target.is_dir() or any(target.iterdir())):
        raise ValueError("output directory must be absent or empty")
    target.mkdir(parents=True, exist_ok=True)
    return target


def _build_manifest(
    replay: dict,
    *,
    input_jsonl_sha256: str,
    output_replay_sha256: str,
) -> dict:
    frames = [
        frame
        for sequence in replay["sequences"]
        for frame in sequence["frames"]
    ]
    return {
        "schema_version": "tadps_replay_export_manifest.v1",
        "input_schema_version": "tadps_selector_candidate_frame.v1",
        "output_schema_version": replay["schema_version"],
        "input_jsonl_sha256": input_jsonl_sha256,
        "output_replay_sha256": output_replay_sha256,
        "sequence_count": len(replay["sequences"]),
        "frame_count": len(frames),
        "candidate_count": sum(len(frame["candidates"]) for frame in frames),
        "source_candidate_set_attestation": (
            "complete_preselection_candidate_set"
        ),
        "reconstructed_from_selected_points": False,
        "motion_commands_emitted": 0,
    }


if __name__ == "__main__":
    raise SystemExit(main())
