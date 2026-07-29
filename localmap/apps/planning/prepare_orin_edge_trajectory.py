#!/usr/bin/env python3
"""Generate a fresh execution-strict trajectory for audited Orin handoff."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
AIRY_ROOT = LOCALMAP_DIR.parent
ROS_PYTHON = Path("/usr/bin/python3")
sys.path.insert(0, str(LOCALMAP_DIR))
sys.path.insert(0, str(AIRY_ROOT))

from apps.planning.run_planning_once import (
    execute_staged_run,
    invalidate_outputs,
    prepare_orin_edge_planning_run,
    publish_prepared_run,
    require_ros_python,
)
from localmap_core.orin_edge_handoff import (
    build_orin_edge_handoff,
    write_orin_edge_handoff,
)
from localmap_core.planning_profile import (
    DEFAULT_PLANNING_PROFILE,
    load_planning_profile,
)
from mission.contract import load_mission


HANDOFF_FILE_NAME = "orin_edge_trajectory_handoff.json"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "冻结当前LocalMap与Bucket Tip，生成execution-strict轨迹和Orin交接清单；"
            "本命令不连接Orin、不创建动作发送器。"
        )
    )
    parser.add_argument(
        "--mission",
        type=Path,
        required=True,
        help="excavation_mission.v1 JSON",
    )
    parser.add_argument("--phase", choices=("dig", "dump"), required=True)
    parser.add_argument(
        "--profile",
        type=Path,
        default=DEFAULT_PLANNING_PROFILE,
        help="Planning Profile JSON",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        profile = load_planning_profile(args.profile)
        mission = load_mission(args.mission)
        require_ros_python()
        if not profile.inputs.machine_profile.is_file():
            raise ValueError(
                f"planning input machine_profile 不存在: "
                f"{profile.inputs.machine_profile}"
            )

        profile.outputs.directory.mkdir(parents=True, exist_ok=True)
        handoff_path = profile.outputs.directory / HANDOFF_FILE_NAME
        handoff_path.unlink(missing_ok=True)
        invalidate_outputs(profile.outputs)
        profile.outputs.observation_slice.unlink(missing_ok=True)
        created_at_s = time.time()
        with tempfile.TemporaryDirectory(
            prefix=".orin-edge-planning-",
            dir=profile.outputs.directory,
        ) as staging_directory:
            prepared = prepare_orin_edge_planning_run(
                profile,
                mission_path=args.mission,
                phase=args.phase,
                now_s=created_at_s,
                python=ROS_PYTHON,
                staging_dir=Path(staging_directory),
            )
            execute_staged_run(prepared)
            manifest = build_orin_edge_handoff(
                prepared.staging_outputs.trajectory,
                mission=mission,
                phase=args.phase,
                source_bucket_tip=prepared.snapshot.bucket_tip,
                created_at_s=time.time(),
            )
            staged_handoff_path = (
                prepared.staging_outputs.directory / HANDOFF_FILE_NAME
            )
            write_orin_edge_handoff(staged_handoff_path, manifest)
            publish_prepared_run(prepared)
            os.replace(staged_handoff_path, handoff_path)
        print(
            "Orin edge trajectory prepared: "
            f"phase={args.phase}, execution_eligible=true, action_datagrams=0"
        )
        print(f"trajectory: {profile.outputs.trajectory}")
        print(f"handoff: {handoff_path}")
        print(f"sha256: {manifest['trajectory']['sha256']}")
        print(
            "start_distance_m: "
            f"{manifest['start_distance_m']:.6f} "
            f"(limit={manifest['waypoint_tolerance_m']:.6f})"
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"Orin edge planning failed: {exc}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
