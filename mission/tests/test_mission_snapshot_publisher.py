import json
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from airy_excavator_interfaces.msg import TargetSnapshot
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from mission.apps.publish_mission_markers import MissionMarkerPublisher, parse_cli_args


MISSION_PATH = Path(__file__).resolve().parents[1] / "config/excavation_cycle.json"
MARKER_STYLE_PATH = Path(__file__).resolve().parents[1] / "config/marker_style.json"
CATALOG_PATH = (
    Path(__file__).resolve().parents[1]
    / "config/excavation_dig_point_catalog.v1.json"
)


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


def test_mission_publisher_accepts_the_authoritative_dig_point_catalog():
    args = parse_cli_args(
        [
            "mission_snapshot_publisher",
            "--dig-point-catalog",
            str(CATALOG_PATH),
        ]
    )

    assert args.dig_point_catalog.name == "excavation_dig_point_catalog.v1.json"


def test_mission_publisher_exposes_typed_dig_and_dump_snapshots():
    rclpy.init()
    publisher = MissionMarkerPublisher(
        MISSION_PATH,
        "/test/mission_target_markers",
        100.0,
        dig_point_catalog_path=CATALOG_PATH,
        marker_style_path=MARKER_STYLE_PATH,
    )
    observer = rclpy.create_node("mission_snapshot_observer")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received = {}
    marker_messages = []
    observer.create_subscription(
        MarkerArray,
        "/test/mission_target_markers",
        marker_messages.append,
        qos,
    )
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
        while (len(received) < 2 or not marker_messages) and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert set(received) == {"dig", "dump"}
        assert received["dig"].target_id == "field_cycle_001:dig"
        assert received["dig"].mission_phase == "dig"
        assert received["dig"].header.frame_id == "machine_root_ros"
        assert received["dump"].target_id == "field_cycle_001:dump"
        assert received["dig"].mission_sha256 == received["dump"].mission_sha256
        assert received["dig"].radius_m == pytest.approx(0.25)
        target_markers = [
            marker
            for marker in marker_messages[-1].markers
            if marker.ns == "mission_targets"
        ]
        catalog = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
        assert len(target_markers) == len(catalog["dig_points"]) + 1
        assert all(marker.scale.x == pytest.approx(0.08) for marker in target_markers)
    finally:
        observer.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()


def test_invalid_display_style_does_not_suppress_target_snapshots(tmp_path):
    invalid_style = tmp_path / "marker_style.json"
    invalid_style.write_text("{not-json", encoding="utf-8")
    rclpy.init()
    publisher = MissionMarkerPublisher(
        MISSION_PATH,
        "/test/invalid_style_target_markers",
        100.0,
        dig_point_catalog_path=CATALOG_PATH,
        marker_style_path=invalid_style,
    )
    observer = rclpy.create_node("invalid_style_snapshot_observer")
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
        assert received["dig"].radius_m == pytest.approx(0.25)
    finally:
        observer.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
