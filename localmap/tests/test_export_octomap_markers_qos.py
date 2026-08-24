from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from localmap.apps.perception.export_octomap_markers_to_local_map import (
    marker_capture_qos,
)


def test_marker_capture_requests_the_latched_octomap_snapshot():
    qos = marker_capture_qos()

    assert qos.depth == 1
    assert qos.reliability is ReliabilityPolicy.RELIABLE
    assert qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
