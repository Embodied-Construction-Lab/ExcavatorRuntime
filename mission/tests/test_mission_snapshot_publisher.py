import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from airy_excavator_interfaces.msg import TargetSnapshot
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mission.apps.publish_mission_markers import MissionMarkerPublisher, parse_cli_args


MISSION_PATH = Path(__file__).resolve().parents[1] / "config/excavation_cycle.json"


def test_mission_publisher_accepts_ros_launch_arguments():
    args = parse_cli_args(
        [
            "mission_snapshot_publisher",
            "--rate-hz",
            "4.0",
            "--ros-args",
            "-r",
            "__node:=mission_snapshot_publisher",
        ]
    )

    assert args.rate_hz == 4.0


def test_mission_publisher_accepts_an_explicit_demo_program():
    args = parse_cli_args(
        [
            "mission_snapshot_publisher",
            "--demo",
            str(Path(__file__).resolve().parents[1] / "config/excavation_demo.json"),
        ]
    )

    assert args.demo.name == "excavation_demo.json"


def test_mission_publisher_exposes_typed_dig_and_dump_snapshots():
    rclpy.init()
    publisher = MissionMarkerPublisher(
        MISSION_PATH, "/test/mission_target_markers", 100.0
    )
    observer = rclpy.create_node("mission_snapshot_observer")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received = {}
    for phase in ("dig", "dump"):
        observer.create_subscription(
            TargetSnapshot,
            f"/mission/{phase}_target_snapshot",
            lambda message, phase=phase: received.__setitem__(phase, message),
            qos,
        )
    try:
        publisher.publish_markers()
        deadline = time.monotonic() + 2.0
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert set(received) == {"dig", "dump"}
        assert received["dig"].target_id == "field_cycle_001:dig"
        assert received["dig"].mission_phase == "dig"
        assert received["dig"].header.frame_id == "machine_root_ros"
        assert received["dump"].target_id == "field_cycle_001:dump"
        assert received["dig"].mission_sha256 == received["dump"].mission_sha256
    finally:
        observer.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
