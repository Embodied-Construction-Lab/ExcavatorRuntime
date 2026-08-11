#!/usr/bin/env python3
"""旁路抓取并格式化 Orin -> PC 的 machine_state_v1 UDP 包。

本工具通过 tcpdump 观察网络包，不绑定 18081，因此可与只读 PC 状态桥并行运行。
需要以 root/cap_net_raw 权限执行 tcpdump。
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from typing import Any


DEFAULT_ORIN_HOST = "192.168.2.88"
DEFAULT_STATE_PORT = 18081
DEFAULT_TIMEOUT_S = 10.0


def extract_machine_state_packets(capture_text: str) -> list[dict[str, Any]]:
    """从 tcpdump 的 ASCII 输出中提取完整 machine_state_v1 JSON object。"""
    packets: list[dict[str, Any]] = []
    decoder = json.JSONDecoder()
    marker = '{"type":"machine_state_v1"'
    cursor = 0

    while True:
        start = capture_text.find(marker, cursor)
        if start < 0:
            return packets
        try:
            value, end = decoder.raw_decode(capture_text[start:])
        except json.JSONDecodeError:
            cursor = start + len(marker)
            continue
        if isinstance(value, dict) and value.get("type") == "machine_state_v1":
            packets.append(value)
        cursor = start + end


def _number(value: object, *, default: float = 0.0) -> float:
    return float(value) if isinstance(value, int | float) and not isinstance(value, bool) else default


def format_machine_state_packet(packet: dict[str, Any]) -> str:
    """将一个已解码状态包显示为便于现场核对的摘要。"""
    safety = packet.get("safety") if isinstance(packet.get("safety"), dict) else {}
    actuators = packet.get("actuator_state") if isinstance(packet.get("actuator_state"), dict) else {}
    positions = packet.get("joint_state", {}).get("position_rad", {})
    velocities = packet.get("joint_state", {}).get("velocity_rad_s", {})

    lines = [
        "=" * 72,
        f"seq={packet.get('seq')}  stamp_ms={packet.get('stamp_ms')}  stm32_stamp_ms={packet.get('stm32_stamp_ms')}",
        f"source={packet.get('source')}  machine_id={packet.get('machine_id')}",
        "safety: "
        f"estop={safety.get('estop')}, stm32_alive={safety.get('stm32_alive')}, "
        f"sensor_valid={safety.get('sensor_valid')}, control_enabled={safety.get('control_enabled')}, "
        f"fault_flags={safety.get('fault_flags', [])}",
        "actuators:",
    ]
    for name in ("boom", "stick", "bucket"):
        state = actuators.get(name) if isinstance(actuators.get(name), dict) else {}
        lines.append(
            f"  {name}: pos={_number(state.get('position_m')):.5f} m, "
            f"vel={_number(state.get('velocity_mps')):.5f} m/s"
        )
    swing = actuators.get("swing") if isinstance(actuators.get("swing"), dict) else {}
    lines.append(
        f"  swing: pos={_number(swing.get('position_rad')):.5f} rad, "
        f"vel={_number(swing.get('velocity_rad_s')):.5f} rad/s"
    )
    lines.append(
        "joints(rad): "
        + ", ".join(
            f"{name}={_number(positions.get(name)):.5f}"
            for name in ("swing", "boom", "arm", "bucket")
        )
    )
    lines.append(
        "joint_vel(rad/s): "
        + ", ".join(
            f"{name}={_number(velocities.get(name)):.5f}"
            for name in ("swing", "boom", "arm", "bucket")
        )
    )
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="旁路格式化显示 Orin machine_state_v1 UDP 包。")
    parser.add_argument("--count", type=int, default=1, help="显示最近捕获的包数，默认1")
    parser.add_argument("--orin-host", default=DEFAULT_ORIN_HOST, help="Orin IP")
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT, help="PC 状态 UDP 端口")
    parser.add_argument("--interface", default="any", help="tcpdump 抓包网卡，默认 any")
    return parser


def capture_packets(args: argparse.Namespace) -> str:
    if not 1 <= args.count <= 100:
        raise ValueError("--count 必须是 1..100")
    tcpdump = shutil.which("tcpdump")
    if tcpdump is None:
        raise RuntimeError("未找到 tcpdump；请安装 tcpdump 后重试")
    packet_filter = f"udp and src host {args.orin_host} and dst port {args.state_port}"
    command = [tcpdump, "-l", "-nn", "-s", "0", "-A", "-c", str(args.count), "-i", args.interface, packet_filter]
    try:
        result = subprocess.run(
            command,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=DEFAULT_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired as exc:
        return exc.stdout or ""
    except subprocess.CalledProcessError as exc:
        message = (exc.stderr or "").strip()
        raise RuntimeError(f"tcpdump 执行失败: {message or exc}") from exc
    return result.stdout


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        capture_text = capture_packets(args)
    except (RuntimeError, ValueError) as exc:
        print(f"Orin packet inspection failed: {exc}")
        return 2

    packets = extract_machine_state_packets(capture_text)
    if not packets:
        print("未在 10 秒内捕获到可解析的 Orin machine_state_v1 包。")
        return 1
    for packet in packets:
        print(format_machine_state_packet(packet), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
