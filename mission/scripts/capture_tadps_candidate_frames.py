#!/usr/bin/env python3
"""Capture opt-in selector evidence without publishing any control command."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

AIRY_ROOT = Path(__file__).resolve().parents[2]
if str(AIRY_ROOT) not in sys.path:
    sys.path.insert(0, str(AIRY_ROOT))

from mission.tadps_live_capture import (
    TadpsLiveCaptureError,
    TadpsLiveCaptureSession,
    verify_selector_bridge_provenance,
)


DEFAULT_TOPIC = "/digging/tadps_candidate_frame"
DEFAULT_DESCRIPTOR = (
    Path(__file__).resolve().parent.parent
    / "bridges"
    / "excavator_dig_point_tadps_candidate_trace.v1.json"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe read-only to complete selector candidate frames and write "
            "an append-only JSONL trace plus a source-bound atomic manifest."
        )
    )
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--sequence-id", required=True)
    parser.add_argument("--selector-source-root", type=Path, required=True)
    parser.add_argument("--bridge-descriptor", type=Path, default=DEFAULT_DESCRIPTOR)
    parser.add_argument("--source-topic", default=DEFAULT_TOPIC)
    parser.add_argument(
        "--maximum-payload-mib",
        type=int,
        default=64,
        help="Reject any single String payload larger than this many MiB.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 1 <= args.maximum_payload_mib <= 256:
        raise TadpsLiveCaptureError("--maximum-payload-mib must be in [1, 256]")
    snapshot = verify_selector_bridge_provenance(
        descriptor_path=args.bridge_descriptor,
        selector_source_root=args.selector_source_root,
    )

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError("ROS 2 rclpy and std_msgs are required for live capture") from exc

    capture = TadpsLiveCaptureSession(
        output_directory=args.output_directory,
        expected_sequence_id=args.sequence_id,
        source_topic=args.source_topic,
        selector_source_snapshot=snapshot,
        maximum_payload_bytes=args.maximum_payload_mib * 1024 * 1024,
    )
    fatal_error: list[BaseException] = []
    rclpy.init(args=None)
    node = Node("tadps_candidate_frame_capture")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=128,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    def record(message: String) -> None:
        try:
            capture.record_json(message.data)
        except BaseException as exc:  # fail closed: never bless an incomplete trace
            fatal_error.append(exc)
            node.get_logger().error(f"TADPS evidence capture rejected: {exc}")
            rclpy.shutdown()

    subscription = node.create_subscription(String, args.source_topic, record, qos)
    _ = subscription
    print(
        "TADPS capture ready; start selector with "
        f"candidate_trace_enabled:=true candidate_trace_sequence_id:={args.sequence_id}",
        flush=True,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if fatal_error:
        capture.abort()
        raise TadpsLiveCaptureError(str(fatal_error[0])) from fatal_error[0]
    manifest = capture.close()
    print(f"TADPS capture manifest: {manifest}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (TadpsLiveCaptureError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
