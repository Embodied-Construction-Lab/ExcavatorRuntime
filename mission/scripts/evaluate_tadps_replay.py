#!/usr/bin/env python3
"""Evaluate frozen TADPS candidate replays and write evidence artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

AIRY_ROOT = Path(__file__).resolve().parents[2]
if str(AIRY_ROOT) not in sys.path:
    sys.path.insert(0, str(AIRY_ROOT))

from mission.tadps_benchmark import TadpsBenchmarkError, evaluate_tadps_replay


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate highest-point, score-only, and full TADPS offline."
    )
    parser.add_argument("--replay", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        replay_bytes = _read_regular_file(args.replay, "replay")
        config_bytes = _read_regular_file(args.config, "config")
        replay = json.loads(replay_bytes)
        config = json.loads(config_bytes)
        result = evaluate_tadps_replay(replay, config)
        manifest = {
            **result["manifest"],
            "input_file_sha256": hashlib.sha256(replay_bytes).hexdigest(),
            "config_file_sha256": hashlib.sha256(config_bytes).hexdigest(),
        }
        _write_artifacts(
            args.output_dir,
            manifest=manifest,
            per_frame=result["per_frame"],
            summary=result["summary"],
        )
    except (OSError, json.JSONDecodeError, TadpsBenchmarkError, ValueError) as exc:
        print(f"TADPS benchmark failed: {exc}", file=sys.stderr)
        return 2
    print(str(args.output_dir.expanduser().resolve()))
    return 0


def _read_regular_file(path: Path, label: str) -> bytes:
    resolved = path.expanduser().resolve(strict=True)
    if not resolved.is_file():
        raise ValueError(f"{label} must be a regular file")
    return resolved.read_bytes()


def _write_artifacts(
    output_dir: Path,
    *,
    manifest: dict,
    per_frame: list[dict],
    summary: dict,
) -> None:
    target = output_dir.expanduser().resolve()
    if target.exists() and (not target.is_dir() or any(target.iterdir())):
        raise ValueError("output directory must be absent or empty")
    target.mkdir(parents=True, exist_ok=True)
    (target / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (target / "per_frame.jsonl").write_text(
        "".join(
            json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
            for row in per_frame
        ),
        encoding="utf-8",
    )
    (target / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    raise SystemExit(main())
