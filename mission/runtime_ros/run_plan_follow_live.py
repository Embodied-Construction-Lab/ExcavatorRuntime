#!/usr/bin/python3
"""Plan one live Mission phase and execute it through the Orin Follow gateway."""

from __future__ import annotations

import sys
from pathlib import Path
from types import MappingProxyType

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions

from localmap_core.planning_inputs import wait_for_live_planning_inputs
from localmap_core.planning_profile import load_planning_profile
from mission.contract import ExcavationMission, load_mission
from mission.demo import load_demo_program
from mission.runtime_ros.run_plan_follow_shadow import (
    PlanFollowLiveClient,
    build_arg_parser,
)


DEFAULT_PLANNING_PROFILE = (
    Path(get_package_share_directory("airy_localmap")) / "config/planning.json"
)
DEFAULT_DEMO_PROGRAM = (
    Path(get_package_share_directory("airy_mission_runtime"))
    / "config"
    / "excavation_demo.json"
)


def _selected_demo_mission(path: Path, point_id: str) -> tuple[ExcavationMission, str]:
    program = load_demo_program(path)
    matches = [point for point in program.dig_points if point.point_id == point_id]
    if len(matches) != 1:
        raise ValueError(f"dig point is not configured: {point_id}")
    point = matches[0]
    mission = ExcavationMission(
        mission_id=program.demo_id,
        mission_type="dig_transport_dump",
        frame_id=program.frame_id,
        target_status=program.target_status,
        targets=MappingProxyType(
            {"dig": point.target, "dump": program.dump_target}
        ),
        limits=program.limits,
        sha256=program.sha256,
    )
    return mission, f"{program.demo_id}:dig:{point.point_id}"


def run(argv: list[str] | None = None) -> int:
    parser = build_arg_parser(
        "Run one causally bound Plan→Follow cycle through the live Orin gateway."
    )
    parser.add_argument(
        "--planning-profile",
        type=Path,
        default=DEFAULT_PLANNING_PROFILE,
    )
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO_PROGRAM)
    parser.add_argument("--dig-point")
    args = parser.parse_args(argv)
    node = None
    try:
        print("waiting for fresh live planning inputs", flush=True)
        wait_for_live_planning_inputs(
            load_planning_profile(args.planning_profile),
            timeout_s=args.wait_s,
        )
        print("live planning inputs ready", flush=True)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = PlanFollowLiveClient()
        if args.dig_point is None:
            mission = load_mission(args.mission)
            target_id = None
        else:
            if args.phase != "dig":
                raise ValueError("--dig-point is only valid for the dig phase")
            mission, target_id = _selected_demo_mission(
                args.demo, args.dig_point
            )
        outcome = node.run_phase(
            mission=mission,
            phase=args.phase,
            wait_s=args.wait_s,
            target_id=target_id,
        )
        print(
            "Plan→Follow live complete: "
            f"reason={outcome.follow_result.reason_code} "
            f"quiescence_confirmed={outcome.follow_result.quiescence_confirmed} "
            f"action_datagrams={outcome.follow_result.action_datagrams}",
            flush=True,
        )
        return 0
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Plan→Follow live failed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Plan→Follow live cancelled by operator", file=sys.stderr)
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
