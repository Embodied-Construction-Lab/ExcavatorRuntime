#!/usr/bin/env python3
"""PC state bridge: receive Orin Machine State and optionally publish JointState."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from runtime_bridge.protocol import (
    ExcavatorStatePacket,
    MachineStatePacket,
    PacketDecodeError,
    decode_packet,
)
from runtime_bridge.runtime_config import DEFAULT_RUNTIME_CONFIG, load_runtime_config
from runtime_bridge.ros_provenance import set_ros_header_stamp


DEFAULT_LATEST_STATE = PROJECT_ROOT / "runtime_bridge" / "exports" / "latest_state.json"


def non_negative_int(value: str) -> int:
    """解析允许0的包计数间隔。"""
    converted = int(value)
    if converted < 0:
        raise argparse.ArgumentTypeError("必须是大于等于0的整数")
    return converted


def should_print_state(state_count: int, print_every: int) -> bool:
    """按有效状态包计数判断是否打印本包。"""
    return print_every > 0 and state_count % print_every == 0


def build_arg_parser() -> argparse.ArgumentParser:
    """构造通信诊断入口参数。"""
    parser = argparse.ArgumentParser(description="接收 Orin 状态包并发布只读 PC 状态。")
    parser.add_argument("--config", type=Path, default=DEFAULT_RUNTIME_CONFIG, help="运行配置JSON")
    parser.add_argument("--publish-joint-states", action="store_true", help="把状态包关节角发布为ROS2 /joint_states")
    parser.add_argument(
        "--print-every",
        type=non_negative_int,
        default=None,
        help="每N个有效状态包打印一次；0关闭打印，默认读取runtime配置",
    )
    return parser


class JointStatePublisher:
    """可选 ROS2 /joint_states 发布器；未启用时不导入ROS模块。"""

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "无法导入ROS2 Python模块。请先source ROS环境，并使用/usr/bin/python3运行。\n"
                f"原始错误: {exc}"
            ) from exc

        self.rclpy = rclpy
        self.JointState = JointState
        rclpy.init(args=None)
        self.node = Node("pc_runtime_joint_state_bridge")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)

    def publish(self, state: ExcavatorStatePacket | MachineStatePacket) -> None:
        """发布 ROS2 JointState，供 waji_description 计算 bucket tip。"""
        message = self.JointState()
        set_ros_header_stamp(message.header, state.stamp_ms)
        message.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        # 关键：协议里是短名，ROS JointState 里是运动学包要求的 joint name。
        message.position = [
            state.joint_position_rad["swing"],
            state.joint_position_rad["boom"],
            state.joint_position_rad["arm"],
            state.joint_position_rad["bucket"],
        ]
        message.velocity = [
            state.joint_velocity_rad_s["swing"],
            state.joint_velocity_rad_s["boom"],
            state.joint_velocity_rad_s["arm"],
            state.joint_velocity_rad_s["bucket"],
        ]
        self.publisher.publish(message)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        """关闭 ROS2 节点。"""
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def write_latest_state(path: Path, state: ExcavatorStatePacket | MachineStatePacket) -> None:
    """写出最近状态，方便 smoke check 或人工排查。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    """接收状态包，按需写 JSON 和发布 JointState。"""
    args = build_arg_parser().parse_args()
    try:
        # 读取并验证配置文件 runtime_bridge/config/runtime.json,包括网络端口和诊断打印间隔。
        config = load_runtime_config(args.config)
    except (OSError, ValueError) as exc:
        print(f"runtime diagnostic configuration error: {exc}", file=sys.stderr, flush=True)
        return 2

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(config.network.state_endpoint)
    joint_state_publisher = JointStatePublisher() if args.publish_joint_states else None
    print_every = config.diagnostics.print_every if args.print_every is None else args.print_every

    state_count = 0
    print(
        "pc state bridge started: "
        f"state <- {config.network.state_endpoint}, "
        f"publish_joint_states={args.publish_joint_states}, "
        f"print_every={print_every}, receive_only=true",
        flush=True,
    )

    try:
        # UDP 接收循环，按配置打印、写出最近状态，并按需发布 JointState。
        while True:
            payload, address = recv_sock.recvfrom(4096)
            try:
                # 协议里是二进制包，必须 decode_packet() 才能得到 ExcavatorStatePacket 或 MachineStatePacket。
                # packet : seq, stamp_ms, 具体的状态数据(包括关节角度、速度、传感器状态等)
                packet = decode_packet(payload)
            except PacketDecodeError as exc:
                # 当 packet_type 无效时会抛出 PacketDecodeError并丢掉该包
                print(f"drop invalid packet from {address}: {exc}", flush=True)
                continue
            if not isinstance(packet, ExcavatorStatePacket | MachineStatePacket):
                continue

            state_count += 1
            if (
                config.diagnostics.write_every > 0
                and state_count % config.diagnostics.write_every == 0
            ):
                # 按照write_every间隔输出状态
                write_latest_state(DEFAULT_LATEST_STATE, packet)
            if joint_state_publisher is not None:
                # 发布 ROS2 JointState，供 waji_description 计算 bucket tip。
                joint_state_publisher.publish(packet)
            if should_print_state(state_count, print_every):
                age_ms = int(time.time() * 1000) - packet.stamp_ms
                if isinstance(packet, MachineStatePacket):
                    # 关键：正式协议下把安全状态也打出来，方便联调时一眼看出为何不执行动作。
                    safety = packet.safety
                    print(
                        f"state[{state_count}] from {address}: seq={packet.seq}, age={age_ms}ms, "
                        f"estop={safety['estop']}, sensor_valid={safety['sensor_valid']}, "
                        f"control_enabled={safety['control_enabled']}, faults={safety['fault_flags']}, "
                        "pc_motion_gate=not_applicable",
                        flush=True,
                    )
                else:
                    print(
                        f"state[{state_count}] from {address}: seq={packet.seq}, age={age_ms}ms, estop={packet.estop}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print("pc runtime bridge stopped", flush=True)
    finally:
        if joint_state_publisher is not None:
            joint_state_publisher.close()
        recv_sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
