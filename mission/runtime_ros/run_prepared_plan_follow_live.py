#!/usr/bin/python3
"""Prepare one live execution trajectory, then wait for a one-shot gate to Follow."""

from __future__ import annotations

import math
import re
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions

from airy_excavator_interfaces.snapshot_digest import trajectory_snapshot_message_sha256
from localmap_core.planning_inputs import load_live_planning_inputs, wait_for_live_planning_inputs
from localmap_core.planning_profile import load_planning_profile
from mission.contract import ExcavationMission, load_mission
from mission.demo import load_demo_program
from mission.runtime_ros.run_plan_follow_live import (
    DEFAULT_DEMO_PROGRAM,
    DEFAULT_PLANNING_PROFILE,
    _selected_demo_mission,
)
from mission.runtime_ros.run_plan_follow_shadow import (
    PlanFollowLiveClient,
    build_arg_parser,
)


class PreparedFollowFallback(RuntimeError):
    """The prepared trajectory is no longer safe to activate after the gate."""


_SAFE_GATE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def load_latest_live_bucket_tip(profile, *, now_s: float) -> dict:
    return dict(load_live_planning_inputs(profile, now_s=now_s).bucket_tip)


def validate_prepared_follow_activation(
    *,
    snapshot,
    runtime_now_s: float,
    latest_bucket_tip: dict,
    allowed_first_waypoint_distance_m: float,
) -> None:
    if (
        isinstance(allowed_first_waypoint_distance_m, bool)
        or not isinstance(allowed_first_waypoint_distance_m, (int, float))
        or not math.isfinite(float(allowed_first_waypoint_distance_m))
        or float(allowed_first_waypoint_distance_m) <= 0.0
        or float(allowed_first_waypoint_distance_m) > 2.0
    ):
        raise ValueError("allowed_first_waypoint_distance_m must be in (0, 2]")
    if trajectory_snapshot_message_sha256(snapshot) != snapshot.trajectory_sha256:
        raise PreparedFollowFallback("prepared trajectory digest mismatch")
    if snapshot.input_source != "live":
        raise PreparedFollowFallback("prepared trajectory input source is not live")
    if snapshot.map_source != "live_local_map":
        raise PreparedFollowFallback("prepared trajectory map source is not live_local_map")
    valid_until_s = (
        float(snapshot.valid_until.sec) + float(snapshot.valid_until.nanosec) * 1e-9
    )
    if runtime_now_s > valid_until_s:
        raise PreparedFollowFallback("prepared trajectory expired before Follow submission")
    if not snapshot.waypoints:
        raise PreparedFollowFallback("prepared trajectory has no waypoints")
    position = latest_bucket_tip.get("position_m")
    if (
        not isinstance(position, (list, tuple))
        or len(position) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in position
        )
    ):
        raise PreparedFollowFallback("live bucket tip position is invalid")
    first = snapshot.waypoints[0]
    first_distance_m = math.dist(
        (float(first.x), float(first.y), float(first.z)),
        tuple(float(value) for value in position),
    )
    if first_distance_m > float(allowed_first_waypoint_distance_m):
        raise PreparedFollowFallback(
            "prepared trajectory first waypoint is too far from the fresh live bucket tip: "
            f"distance_m={first_distance_m:.4f}"
        )


def wait_for_start_gate(path: Path, *, poll_interval_s: float = 0.05) -> None:
    gate = Path(path)
    if not gate.is_absolute():
        raise ValueError("start gate must be an absolute path")
    if gate.parent == gate or "/" in gate.name or not _SAFE_GATE_NAME.fullmatch(gate.name):
        raise ValueError("start gate must be a safe absolute single file path")
    if not math.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
        raise ValueError("poll_interval_s must be positive")
    while True:
        try:
            gate.unlink()
        except FileNotFoundError:
            time.sleep(poll_interval_s)
        else:
            return


def _load_selected_mission(args) -> tuple[ExcavationMission, str | None]:
    if args.dig_point is None:
        return load_mission(args.mission), None
    if args.phase != "dig":
        raise ValueError("--dig-point is only valid for the dig phase")
    return _selected_demo_mission(args.demo, args.dig_point)


def run(argv: list[str] | None = None) -> int:
    parser = build_arg_parser(
        "Prepare one execution-strict trajectory, wait for a one-shot gate, then Follow it."
    )
    parser.add_argument(
        "--planning-profile",
        type=Path,
        default=DEFAULT_PLANNING_PROFILE,
    )
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO_PROGRAM)
    parser.add_argument("--dig-point")
    parser.add_argument("--start-gate", type=Path, required=True)
    parser.add_argument(
        "--first-waypoint-distance-m",
        type=float,
        default=0.08,
        help="Maximum allowed distance from the fresh live Bucket Tip to trajectory waypoint[0]. Bounded to (0, 2].",
    )
    args = parser.parse_args(argv)
    node = None
    try:
        profile = load_planning_profile(args.planning_profile)
        print("waiting for fresh live planning inputs", flush=True)
        wait_for_live_planning_inputs(profile, timeout_s=args.wait_s)
        mission, target_id = _load_selected_mission(args)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = PlanFollowLiveClient()
        plan_result = node.plan_phase(
            mission=mission,
            phase=args.phase,
            wait_s=args.wait_s,
            target_id=target_id,
        )
        trajectory = plan_result.trajectory
        print(
            "prepared follow ready: "
            f"trajectory_id={trajectory.trajectory_id} "
            f"valid_until_s={trajectory.valid_until.sec + trajectory.valid_until.nanosec * 1e-9:.3f} "
            f"gate={Path(args.start_gate)}",
            flush=True,
        )
        wait_for_start_gate(args.start_gate)
        node.require_runtime_ready(
            args.wait_s, expected_input_source=trajectory.input_source
        )
        runtime_now_s = node.get_clock().now().nanoseconds * 1e-9
        latest_bucket_tip = load_latest_live_bucket_tip(profile, now_s=runtime_now_s)
        validate_prepared_follow_activation(
            snapshot=trajectory,
            runtime_now_s=runtime_now_s,
            latest_bucket_tip=latest_bucket_tip,
            allowed_first_waypoint_distance_m=args.first_waypoint_distance_m,
        )
        follow_result = node.follow_trajectory(trajectory, wait_s=args.wait_s)
        print(
            "Prepared→Follow live complete: "
            f"reason={follow_result.reason_code} "
            f"quiescence_confirmed={follow_result.quiescence_confirmed} "
            f"action_datagrams={follow_result.action_datagrams}",
            flush=True,
        )
        return 0
    except PreparedFollowFallback as exc:
        print(f"Prepared→Follow live safe fallback: {exc}", file=sys.stderr)
        return 3
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Prepared→Follow live failed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Prepared→Follow live cancelled by operator", file=sys.stderr)
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
