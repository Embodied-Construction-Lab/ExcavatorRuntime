#!/usr/bin/python3
"""持续读取 Mission 文件，并在 RViz 中显示 dig/dump 目标。"""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from airy_excavator_interfaces.msg import TargetSnapshot
    from geometry_msgs.msg import Point, Vector3
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
except ModuleNotFoundError as exc:
    raise SystemExit(
        "无法导入ROS2 Python模块；请source ROS环境并使用/usr/bin/python3运行。\n"
        f"原始错误: {exc}"
    ) from exc

from mission.contract import MissionContractError, load_mission
from mission.demo import load_demo_program
from mission.markers import (
    MissionMarkerStyleError,
    build_demo_marker_specs,
    build_mission_marker_specs,
    load_mission_marker_style,
)


DEFAULT_MISSION = (
    Path(get_package_share_directory("airy_mission_runtime"))
    / "config/excavation_cycle.json"
)
DEFAULT_DEMO = (
    Path(get_package_share_directory("airy_mission_runtime"))
    / "config/excavation_demo.json"
)
DEFAULT_MARKER_STYLE = (
    Path(get_package_share_directory("airy_mission_runtime"))
    / "config/marker_style.json"
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="发布文件式Mission的dig/dump RViz标记。")
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    parser.add_argument("--demo", type=Path)
    parser.add_argument("--marker-style", type=Path, default=DEFAULT_MARKER_STYLE)
    parser.add_argument("--topic", default="/mission/target_markers")
    parser.add_argument("--rate-hz", type=float, default=2.0)
    return parser


def parse_cli_args(argv=None) -> argparse.Namespace:
    """Parse application arguments while accepting standard ROS launch arguments."""
    non_ros_args = rclpy.utilities.remove_ros_args(args=argv)
    return build_arg_parser().parse_args(non_ros_args[1:])


class MissionMarkerPublisher(Node):
    def __init__(
        self,
        mission_path: Path,
        topic: str,
        rate_hz: float,
        demo_path: Path | None = None,
        marker_style_path: Path = DEFAULT_MARKER_STYLE,
    ) -> None:
        super().__init__("excavation_mission_marker_publisher")
        self.mission_path = mission_path
        self.demo_path = demo_path
        self.marker_style_path = marker_style_path
        self.last_mtime_ns: tuple[int, int | None, int | None] | None = None
        self.mission = None
        self.specs = ()
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, topic, marker_qos)
        self.target_publishers = {
            phase: self.create_publisher(
                TargetSnapshot, f"/mission/{phase}_target_snapshot", marker_qos
            )
            for phase in ("dig", "dump")
        }
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish_markers)
        source = f"{mission_path}, demo={demo_path}" if demo_path else str(mission_path)
        self.get_logger().info(f"mission markers: {source} -> {topic}")

    def _reload(self) -> bool:
        try:
            mtime_ns = (
                self.mission_path.stat().st_mtime_ns,
                self.demo_path.stat().st_mtime_ns if self.demo_path else None,
                (
                    self.marker_style_path.stat().st_mtime_ns
                    if self.marker_style_path.exists()
                    else None
                ),
            )
            if self.mission is not None and mtime_ns == self.last_mtime_ns:
                return True
            mission = load_mission(self.mission_path)
            demo = load_demo_program(self.demo_path) if self.demo_path else None
        except (OSError, MissionContractError) as exc:
            self.specs = ()
            self.mission = None
            self.last_mtime_ns = None
            self.get_logger().error(f"Mission无效，清除目标标记: {exc}", throttle_duration_sec=5.0)
            return False

        self.mission = mission
        self.last_mtime_ns = mtime_ns
        try:
            style = load_mission_marker_style(self.marker_style_path)
            self.specs = (
                build_demo_marker_specs(demo, style)
                if demo is not None
                else build_mission_marker_specs(mission, style)
            )
        except (OSError, MissionMarkerStyleError) as exc:
            self.specs = ()
            self.get_logger().error(
                f"目标显示样式无效，仅清除RViz标记: {exc}",
                throttle_duration_sec=5.0,
            )

        self.get_logger().info(
            f"loaded mission {mission.mission_id}, status={mission.target_status}, sha256={mission.sha256[:12]}"
        )
        return True

    def publish_markers(self) -> None:
        clear = Marker()
        clear.action = Marker.DELETEALL
        if not self._reload():
            self.publisher.publish(MarkerArray(markers=[clear]))
            return
        stamp = self.get_clock().now().to_msg()
        markers = [clear]
        for index, spec in enumerate(self.specs):
            sphere = Marker()
            sphere.header.frame_id = spec.frame_id
            sphere.header.stamp = stamp
            sphere.ns = "mission_targets"
            sphere.id = index * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = spec.position_m
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = spec.diameter_m
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = spec.color_rgba
            markers.append(sphere)

            label = Marker()
            label.header.frame_id = spec.frame_id
            label.header.stamp = stamp
            label.ns = "mission_target_labels"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = spec.position_m
            label.pose.position.z += max(spec.diameter_m * 0.7, 0.12)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.1
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = spec.label
            markers.append(label)
        self.publisher.publish(MarkerArray(markers=markers))
        self._publish_target_snapshots(stamp)

    def _publish_target_snapshots(self, stamp) -> None:
        if self.mission is None:
            return
        for phase, publisher in self.target_publishers.items():
            target = self.mission.targets[phase]
            snapshot = TargetSnapshot()
            snapshot.header.frame_id = self.mission.frame_id
            snapshot.header.stamp = stamp
            snapshot.target_id = f"{self.mission.mission_id}:{phase}"
            snapshot.target_kind = phase
            snapshot.target_status = self.mission.target_status
            snapshot.mission_id = self.mission.mission_id
            snapshot.mission_sha256 = self.mission.sha256
            snapshot.mission_phase = phase
            snapshot.position = Point(
                x=target.position_m[0],
                y=target.position_m[1],
                z=target.position_m[2],
            )
            snapshot.normal = Vector3(
                x=target.normal[0], y=target.normal[1], z=target.normal[2]
            )
            snapshot.radius_m = target.radius_m
            publisher.publish(snapshot)


def main() -> int:
    args = parse_cli_args()
    rclpy.init()
    node = MissionMarkerPublisher(
        args.mission,
        args.topic,
        args.rate_hz,
        demo_path=args.demo,
        marker_style_path=args.marker_style,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
