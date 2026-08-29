#!/usr/bin/env python3
"""从OctoMap MarkerArray导出带obstacles的LocalMap JSON。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
except ModuleNotFoundError as exc:
    raise SystemExit(
        "无法导入ROS2 Python模块。请先source ROS环境，并使用/usr/bin/python3运行。\n"
        f"原始错误: {exc}"
    ) from exc


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.io import load_json, write_json
from localmap_core.octomap_obstacle_adapter import centers_to_obstacle_boxes, parse_bounds


DEFAULT_BASE_LOCAL_MAP = LOCALMAP_DIR / "exports" / "live_latest" / "local_map.live.json"
DEFAULT_OUTPUT = LOCALMAP_DIR / "exports" / "live_latest" / "local_map.octomap_obstacles.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """构造OctoMap MarkerArray导出参数。"""
    parser = argparse.ArgumentParser(description="把/occupied_cells_vis_array转换成LocalMap.obstacles。")
    parser.add_argument("--topic", default="/occupied_cells_vis_array", help="OctoMap MarkerArray topic")
    parser.add_argument("--base-local-map", type=Path, default=DEFAULT_BASE_LOCAL_MAP, help="作为ground/target来源的LocalMap JSON")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出LocalMap JSON")
    parser.add_argument("--expected-frame", default="machine_root_ros", help="期望MarkerArray frame_id")
    parser.add_argument("--timeout-s", type=float, default=5.0, help="等待MarkerArray的超时时间")
    parser.add_argument("--box-size", type=float, default=0.15, help="粗化后的box尺寸，单位米")
    parser.add_argument("--max-obstacles", type=int, default=2000, help="最多输出多少个box obstacle")
    parser.add_argument("--bounds", type=float, nargs=6, metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX", "Z_MIN", "Z_MAX"), help="在machine_root下裁剪occupied cells")
    return parser


def marker_array_to_centers(message: MarkerArray, expected_frame: str) -> tuple[np.ndarray, str]:
    """从MarkerArray提取occupied cell中心点。"""
    centers: list[list[float]] = []
    frame_id = expected_frame
    for marker in message.markers:
        if marker.header.frame_id:
            frame_id = marker.header.frame_id
        if marker.type == Marker.CUBE_LIST:
            # 关键：octomap_server通常用CUBE_LIST发布所有occupied cell中心。
            centers.extend([[point.x, point.y, point.z] for point in marker.points])
        elif marker.type == Marker.CUBE:
            centers.append([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    return np.asarray(centers, dtype=np.float64).reshape(-1, 3), frame_id


class MarkerCaptureNode(Node):
    """一次性订阅MarkerArray，用于离线导出LocalMap obstacles。"""

    def __init__(self, topic: str) -> None:
        super().__init__("airy_octomap_marker_exporter")
        self.message: MarkerArray | None = None
        self.subscription = self.create_subscription(
            MarkerArray,
            topic,
            self.on_message,
            marker_capture_qos(),
        )

    def on_message(self, message: MarkerArray) -> None:
        """收到第一帧MarkerArray后保存，主循环会退出。"""
        self.message = message


def marker_capture_qos() -> QoSProfile:
    """Match octomap_server's latched MarkerArray publisher."""

    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def main() -> int:
    """入口函数：MarkerArray -> LocalMap.obstacles。"""
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = MarkerCaptureNode(args.topic)
    deadline = time.time() + args.timeout_s
    try:
        while rclpy.ok() and node.message is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.message is None:
        raise SystemExit(f"等待{args.topic}超时：{args.timeout_s}s")

    centers, frame_id = marker_array_to_centers(node.message, args.expected_frame)
    if frame_id != args.expected_frame:
        raise SystemExit(f"MarkerArray frame {frame_id} 与期望 {args.expected_frame} 不一致")

    local_map = load_json(args.base_local_map)
    if local_map["frame_id"] != frame_id:
        raise SystemExit(f"base LocalMap frame {local_map['frame_id']} 与MarkerArray frame {frame_id} 不一致")

    obstacles = centers_to_obstacle_boxes(
        centers=centers,
        box_size_m=args.box_size,
        bounds=parse_bounds(args.bounds),
        max_obstacles=args.max_obstacles,
    )
    # 关键：复制原LocalMap语义信息，只替换obstacles；ground/targets仍由既有链路负责。
    output_map = dict(local_map)
    output_map["obstacles"] = obstacles
    output_map["notes"] = list(local_map.get("notes", [])) + [
        "obstacles由OctoMap occupied MarkerArray粗化生成，仅用于第一版bucket-tip简单避障。",
    ]
    write_json(args.output, output_map)

    print(f"topic: {args.topic}")
    print(f"frame_id: {frame_id}")
    print(f"occupied_centers: {centers.shape[0]}")
    print(f"obstacles: {len(obstacles)}")
    print(f"output: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
