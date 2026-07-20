#!/usr/bin/env python3
"""Derive a replay CSV with only the physical swing command rescaled."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Sequence


SWING_VELOCITY_COLUMN = "swing_v_ref_radps"
SWING_ACTION_COLUMN = "swing_action_cmd"
REQUIRED_COLUMNS = ("timestamp_s", SWING_VELOCITY_COLUMN, SWING_ACTION_COLUMN)
OTHER_AXES = ("boom", "stick", "bucket")


def _format(value: float) -> str:
    return f"{value:.12g}"


def scale_swing(value: float, *, scale: float, max_abs_radps: float) -> float:
    """Scale one physical swing speed without changing its sign or other axes."""
    if not all(math.isfinite(item) for item in (value, scale, max_abs_radps)):
        raise ValueError("swing value, scale, and max_abs_radps must be finite")
    if scale <= 0.0 or max_abs_radps <= 0.0:
        raise ValueError("scale and max_abs_radps must be positive")
    return max(-max_abs_radps, min(max_abs_radps, value * scale))


def transform_csv(
    input_path: Path,
    output_path: Path,
    *,
    scale: float,
    max_abs_radps: float,
    zero_other_axes: bool = False,
) -> tuple[int, float]:
    """Copy a replay CSV while rescaling swing and optionally zeroing other axes."""
    text = input_path.read_text(encoding="utf-8-sig")
    lines = text.splitlines()
    comments = [line for line in lines if line.startswith("#")]
    table = [line for line in lines if line and not line.startswith("#")]
    if not table:
        raise ValueError("input CSV contains no table")
    reader = csv.DictReader(table)
    if reader.fieldnames is None or not set(REQUIRED_COLUMNS).issubset(reader.fieldnames):
        raise ValueError(f"input CSV must contain {list(REQUIRED_COLUMNS)}")
    if zero_other_axes:
        other_columns = {
            f"{axis}_action_cmd" for axis in OTHER_AXES
        } | {f"{axis}_v_ref_mps" for axis in OTHER_AXES}
        if not other_columns.issubset(reader.fieldnames):
            raise ValueError("input CSV lacks non-swing action/velocity columns")
    rows = list(reader)
    if not rows:
        raise ValueError("input CSV contains no data rows")

    peak = 0.0
    for row_number, row in enumerate(rows, start=2):
        try:
            original = float(row[SWING_VELOCITY_COLUMN])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"row {row_number} has invalid swing velocity") from exc
        transformed = scale_swing(
            original, scale=scale, max_abs_radps=max_abs_radps
        )
        row[SWING_VELOCITY_COLUMN] = _format(transformed)
        row[SWING_ACTION_COLUMN] = _format(transformed / max_abs_radps)
        if zero_other_axes:
            for axis in OTHER_AXES:
                row[f"{axis}_action_cmd"] = "0"
                row[f"{axis}_v_ref_mps"] = "0"
        peak = max(peak, abs(transformed))

    final_swing = float(rows[-1][SWING_VELOCITY_COLUMN])
    if final_swing != 0.0:
        raise ValueError("input CSV must end with an explicit zero swing command")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8-sig", newline="") as output_file:
        for comment in comments:
            output_file.write(f"{comment}\n")
        output_file.write(f"# derived_from={input_path.name}\n")
        transform = (
            "swing_only_scale_then_clamp"
            if zero_other_axes
            else "swing_scale_then_clamp"
        )
        output_file.write(f"# derived_transform={transform}\n")
        output_file.write(f"# swing_scale={_format(scale)}\n")
        output_file.write(f"# swing_max_abs_radps={_format(max_abs_radps)}\n")
        writer = csv.DictWriter(output_file, fieldnames=reader.fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows), peak


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--swing-scale", type=float, required=True)
    parser.add_argument("--swing-max-abs-radps", type=float, default=0.6)
    parser.add_argument(
        "--zero-other-axes",
        action="store_true",
        help="set boom, stick, and bucket action/physical velocity columns to zero",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    rows, peak = transform_csv(
        args.input,
        args.output,
        scale=args.swing_scale,
        max_abs_radps=args.swing_max_abs_radps,
        zero_other_axes=args.zero_other_axes,
    )
    print(f"rows={rows} swing_peak_radps={_format(peak)}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
