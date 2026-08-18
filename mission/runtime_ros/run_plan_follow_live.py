#!/usr/bin/python3
"""Plan one live Mission phase and execute it through the Orin Follow gateway."""

from __future__ import annotations

import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

from mission.contract import load_mission
from mission.runtime_ros.run_plan_follow_shadow import (
    PlanFollowLiveClient,
    build_arg_parser,
)


def run(argv: list[str] | None = None) -> int:
    args = build_arg_parser(
        "Run one causally bound Plan→Follow cycle through the live Orin gateway."
    ).parse_args(argv)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PlanFollowLiveClient()
    try:
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
