"""Length-prefixed JSON transport for the Orin behavior RPC."""

from __future__ import annotations

import json
import socket
import struct
from typing import Any


SCHEMA_VERSION = "orin_behavior_rpc.v1"
MAX_MESSAGE_BYTES = 1024 * 1024
_LENGTH = struct.Struct(">I")


class OrinBehaviorProtocolError(ValueError):
    """The peer sent a frame that is not valid Orin behavior RPC JSON."""


class OrinBehaviorConnectionError(ConnectionError):
    """The RPC connection closed before a complete message arrived."""


def trajectory_snapshot_to_message(snapshot: Any) -> dict[str, Any]:
    """Serialize every canonical ROS TrajectorySnapshot field for Orin."""
    return {
        "trajectory_id": snapshot.trajectory_id,
        "trajectory_sha256": snapshot.trajectory_sha256,
        "frame_id": snapshot.header.frame_id,
        "created_at_s": _time_seconds(snapshot.header.stamp),
        "mission_id": snapshot.mission_id,
        "mission_sha256": snapshot.mission_sha256,
        "mission_phase": snapshot.mission_phase,
        "task_mode": snapshot.task_mode,
        "planning_scope": snapshot.planning_scope,
        "control_stage": snapshot.control_stage,
        "workspace_constraint": snapshot.workspace_constraint,
        "execution_eligible": bool(snapshot.execution_eligible),
        "source_bucket_tip_stamp_s": _time_seconds(
            snapshot.source_bucket_tip_stamp
        ),
        "source_local_map_stamp_s": _time_seconds(
            snapshot.source_local_map_stamp
        ),
        "inputs_frozen_at_s": _time_seconds(snapshot.inputs_frozen_at),
        "valid_until_s": _time_seconds(snapshot.valid_until),
        "input_source": snapshot.input_source,
        "map_source": snapshot.map_source,
        "clock_mode": snapshot.clock_mode,
        "waypoints": [
            [float(point.x), float(point.y), float(point.z)]
            for point in snapshot.waypoints
        ],
        "waypoint_tolerance_m": float(snapshot.waypoint_tolerance_m),
        "waypoint_dwell_s": float(snapshot.waypoint_dwell_s),
        "tracking_timeout_s": float(snapshot.tracking_timeout_s),
    }


def send_message(connection: socket.socket, message: dict[str, Any]) -> None:
    """Send one JSON object using the behavior RPC frame format."""
    try:
        payload = json.dumps(
            message,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise OrinBehaviorProtocolError(f"message is not JSON serializable: {exc}") from exc
    if len(payload) > MAX_MESSAGE_BYTES:
        raise OrinBehaviorProtocolError(
            f"message is too large: {len(payload)} > {MAX_MESSAGE_BYTES} bytes"
        )
    try:
        connection.sendall(_LENGTH.pack(len(payload)) + payload)
    except OSError as exc:
        raise OrinBehaviorConnectionError(
            f"failed to send Orin behavior RPC message: {exc}"
        ) from exc


def receive_message(connection: socket.socket) -> dict[str, Any]:
    """Read one framed UTF-8 JSON object from ``connection``."""
    header = _receive_exact(connection, _LENGTH.size)
    (length,) = _LENGTH.unpack(header)
    if length < 1 or length > MAX_MESSAGE_BYTES:
        raise OrinBehaviorProtocolError(
            f"message length must be between 1 and {MAX_MESSAGE_BYTES} bytes"
        )
    payload = _receive_exact(connection, length)
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OrinBehaviorProtocolError(f"invalid UTF-8 JSON message: {exc}") from exc
    if not isinstance(message, dict):
        raise OrinBehaviorProtocolError("message must be a JSON object")
    return message


def _receive_exact(connection: socket.socket, length: int) -> bytes:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        try:
            chunk = connection.recv(remaining)
        except OSError as exc:
            raise OrinBehaviorConnectionError(
                f"failed to receive Orin behavior RPC message: {exc}"
            ) from exc
        if not chunk:
            raise OrinBehaviorConnectionError(
                "Orin behavior RPC connection closed before a complete message"
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _time_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9
