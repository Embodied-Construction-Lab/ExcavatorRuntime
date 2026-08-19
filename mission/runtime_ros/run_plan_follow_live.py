#!/usr/bin/python3
"""Plan one live Mission phase and execute it through the Orin Follow gateway."""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions

from localmap_core.planning_inputs import wait_for_live_planning_inputs
from localmap_core.planning_profile import load_planning_profile
from mission.contract import load_mission
from mission.runtime_ros.run_plan_follow_shadow import (
    PlanFollowLiveClient,
    build_arg_parser,
)


DEFAULT_PLANNING_PROFILE = (
    Path(get_package_share_directory("airy_localmap")) / "config/planning.json"
)


def run(argv: list[str] | None = None) -> int:
    parser = build_arg_parser(
        "Run one causally bound Plan→Follow cycle through the live Orin gateway."
    )
    parser.add_argument(
        "--planning-profile",
        type=Path,
        default=DEFAULT_PLANNING_PROFILE,
    )
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
        outcome = node.run_phase(
            mission=load_mission(args.mission),
            phase=args.phase,
            wait_s=args.wait_s,
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
