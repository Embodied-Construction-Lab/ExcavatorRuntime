"""Blocking client for Orin-hosted Follow and fixed-action behaviors."""

from __future__ import annotations

import math
import select
import socket
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from runtime_bridge.orin_behavior_rpc import (
    SCHEMA_VERSION,
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
    receive_message,
    send_message,
)


@dataclass(frozen=True)
class FollowFeedback:
    trajectory_id: str
    waypoint_index: int
    waypoint_count: int
    distance_m: float
    elapsed_s: float
    bucket_tip_stamp_s: float
    bucket_tip: tuple[float, float, float]
    tracking_state: str
    action_datagrams: int


@dataclass(frozen=True)
class FollowResult:
    trajectory_id: str
    outcome: str
    reason_code: str
    message: str
    final_waypoint_index: int
    final_distance_m: float
    quiescence_confirmed: bool
    action_datagrams: int


@dataclass(frozen=True)
class FixedActionFeedback:
    behavior: str
    step_index: int
    step_label: str
    phase: str
    max_error: float
    action_datagrams: int


@dataclass(frozen=True)
class FixedActionResult:
    behavior: str
    outcome: str
    reason_code: str
    message: str
    final_step_index: int
    final_step_label: str
    final_max_error: float
    quiescence_confirmed: bool
    action_datagrams: int


@dataclass(frozen=True)
class OrinRuntimeStatus:
    state_fresh: bool
    control_enabled: bool
    sensor_valid: bool
    stm32_alive: bool
    estop: bool
    fault_free: bool
    quiescent: bool
    active_behavior: str
    action_datagrams: int
    motion_gate_reason: str
    sender_constructed: bool
    fixed_actions_available: bool = False
    fixed_actions_validated: bool = False


class FollowRejected(RuntimeError):
    """Orin rejected a start_follow request."""

    def __init__(self, reason_code: str, message: str) -> None:
        super().__init__(f"{reason_code}: {message}")
        self.reason_code = reason_code
        self.message = message


class FixedActionRejected(FollowRejected):
    """Orin rejected a start_fixed_action request."""


@dataclass(frozen=True)
class _StatusSample:
    status: OrinRuntimeStatus
    received_at_s: float


class OrinStatusMonitor:
    """Maintain a status-only connection and expose only fresh connected state."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_s: float = 1.0,
        reconnect_interval_s: float = 0.2,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be non-empty")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if connect_timeout_s <= 0.0 or reconnect_interval_s <= 0.0:
            raise ValueError("status monitor timeouts must be positive")
        self._endpoint = (host, port)
        self._connect_timeout_s = float(connect_timeout_s)
        self._reconnect_interval_s = float(reconnect_interval_s)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._connection: socket.socket | None = None
        self._sample: _StatusSample | None = None
        self._connected = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def latest(self, *, max_age_s: float) -> OrinRuntimeStatus | None:
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        with self._lock:
            sample = self._sample
            connected = self._connected
        if (
            not connected
            or sample is None
            or time.monotonic() - sample.received_at_s > max_age_s
        ):
            return None
        return sample.status

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            connection = self._connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if self._started:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                connection = socket.create_connection(
                    self._endpoint, timeout=self._connect_timeout_s
                )
                connection.settimeout(None)
                with self._lock:
                    self._connection = connection
                    self._connected = True
                    self._sample = None
                self._consume(connection)
            except (OSError, OrinBehaviorConnectionError, OrinBehaviorProtocolError):
                pass
            finally:
                with self._lock:
                    connection = self._connection
                    self._connection = None
                    self._connected = False
                if connection is not None:
                    connection.close()
            self._stop.wait(self._reconnect_interval_s)

    def _consume(self, connection: socket.socket) -> None:
        expected_seq = 0
        while not self._stop.is_set():
            event = receive_message(connection)
            _validate_event_envelope(event, expected_seq=expected_seq)
            expected_seq += 1
            if event.get("type") != "status":
                raise OrinBehaviorProtocolError(
                    "status connection received a non-status event"
                )
            sample = _StatusSample(
                status=_status_from_message(event),
                received_at_s=time.monotonic(),
            )
            with self._lock:
                self._sample = sample


class OrinFollowClient:
    """Run one Follow RPC at a time while preserving session/request/sequence IDs."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        session_id: str | None = None,
        connect_timeout_s: float = 3.0,
        poll_interval_s: float = 0.05,
        stream_silence_timeout_s: float = 1.0,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be non-empty")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not math.isfinite(connect_timeout_s) or connect_timeout_s <= 0.0:
            raise ValueError("connect_timeout_s must be positive")
        if not math.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s must be positive")
        if (
            not math.isfinite(stream_silence_timeout_s)
            or stream_silence_timeout_s <= 0.0
        ):
            raise ValueError("stream_silence_timeout_s must be positive")
        self._endpoint = (host, port)
        self._session_id = session_id or f"pc-gateway-{uuid.uuid4().hex}"
        self._connect_timeout_s = float(connect_timeout_s)
        self._poll_interval_s = float(poll_interval_s)
        self._stream_silence_timeout_s = float(stream_silence_timeout_s)
        self._request_sequence = 0
        self._lock = threading.Lock()

    @property
    def session_id(self) -> str:
        return self._session_id

    def run_follow(
        self,
        trajectory: dict[str, Any],
        *,
        feedback_callback: Callable[[FollowFeedback], None] | None = None,
        status_callback: Callable[[OrinRuntimeStatus], None] | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> FollowResult:
        """Block through accepted/rejected, zero or more feedback, and result."""
        on_feedback = feedback_callback or (lambda _feedback: None)
        on_status = status_callback or (lambda _status: None)
        should_cancel = cancel_requested or (lambda: False)
        with self._lock:
            request_id = f"follow-{uuid.uuid4().hex}"
            try:
                connection = socket.create_connection(
                    self._endpoint, timeout=self._connect_timeout_s
                )
            except OSError as exc:
                raise OrinBehaviorConnectionError(
                    f"failed to connect to Orin behavior RPC: {exc}"
                ) from exc
            with connection:
                connection.settimeout(None)
                send_message(
                    connection,
                    self._request("start_follow", request_id, trajectory=trajectory),
                )
                return self._receive_follow(
                    connection,
                    request_id=request_id,
                    trajectory_id=_required_text(trajectory, "trajectory_id"),
                    feedback_callback=on_feedback,
                    status_callback=on_status,
                    cancel_requested=should_cancel,
                )

    def run_fixed_action(
        self,
        behavior: str,
        *,
        feedback_callback: Callable[[FixedActionFeedback], None] | None = None,
        status_callback: Callable[[OrinRuntimeStatus], None] | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> FixedActionResult:
        if behavior not in {"ExecuteDig", "ExecuteDump"}:
            raise ValueError("behavior must be ExecuteDig or ExecuteDump")
        on_feedback = feedback_callback or (lambda _feedback: None)
        on_status = status_callback or (lambda _status: None)
        should_cancel = cancel_requested or (lambda: False)
        with self._lock:
            request_id = f"fixed-action-{uuid.uuid4().hex}"
            try:
                connection = socket.create_connection(
                    self._endpoint,
                    timeout=self._connect_timeout_s,
                )
            except OSError as exc:
                raise OrinBehaviorConnectionError(
                    f"failed to connect to Orin behavior RPC: {exc}"
                ) from exc
            with connection:
                connection.settimeout(None)
                send_message(
                    connection,
                    self._request(
                        "start_fixed_action",
                        request_id,
                        behavior=behavior,
                    ),
                )
                return self._receive_fixed_action(
                    connection,
                    request_id=request_id,
                    behavior=behavior,
                    feedback_callback=on_feedback,
                    status_callback=on_status,
                    cancel_requested=should_cancel,
                )

    def _receive_follow(
        self,
        connection: socket.socket,
        *,
        request_id: str,
        trajectory_id: str,
        feedback_callback: Callable[[FollowFeedback], None],
        status_callback: Callable[[OrinRuntimeStatus], None],
        cancel_requested: Callable[[], bool],
    ) -> FollowResult:
        accepted = False
        cancel_sent = False
        expected_event_seq = 0
        last_event_at_s = time.monotonic()
        while True:
            if cancel_requested() and not cancel_sent:
                send_message(connection, self._request("cancel_follow", request_id))
                cancel_sent = True
            readable, _, _ = select.select(
                [connection], [], [], self._poll_interval_s
            )
            if not readable:
                if time.monotonic() - last_event_at_s > self._stream_silence_timeout_s:
                    raise OrinBehaviorConnectionError(
                        "Orin Follow stream was silent for "
                        f"{self._stream_silence_timeout_s:.3f}s"
                    )
                continue
            event = receive_message(connection)
            last_event_at_s = time.monotonic()
            _validate_event_envelope(
                event,
                expected_seq=expected_event_seq,
            )
            expected_event_seq += 1
            event_type = event.get("type")
            if event_type == "status":
                status_callback(_status_from_message(event))
                continue
            _validate_follow_identity(
                event,
                session_id=self._session_id,
                request_id=request_id,
            )
            if event_type == "accepted" and not accepted:
                _require_trajectory(event, trajectory_id)
                accepted = True
            elif event_type == "rejected" and not accepted:
                raise FollowRejected(
                    _required_text(event, "reason_code"),
                    _required_text(event, "message"),
                )
            elif event_type == "feedback" and accepted:
                feedback_callback(_feedback_from_message(event, trajectory_id))
            elif event_type == "result" and accepted:
                return _result_from_message(event, trajectory_id)
            else:
                raise OrinBehaviorProtocolError(
                    f"unexpected {event_type!r} event in Follow stream"
                )

    def _receive_fixed_action(
        self,
        connection: socket.socket,
        *,
        request_id: str,
        behavior: str,
        feedback_callback: Callable[[FixedActionFeedback], None],
        status_callback: Callable[[OrinRuntimeStatus], None],
        cancel_requested: Callable[[], bool],
    ) -> FixedActionResult:
        accepted = False
        cancel_sent = False
        expected_event_seq = 0
        last_event_at_s = time.monotonic()
        while True:
            if cancel_requested() and not cancel_sent:
                send_message(
                    connection,
                    self._request("cancel_fixed_action", request_id),
                )
                cancel_sent = True
            readable, _, _ = select.select(
                [connection],
                [],
                [],
                self._poll_interval_s,
            )
            if not readable:
                if time.monotonic() - last_event_at_s > self._stream_silence_timeout_s:
                    raise OrinBehaviorConnectionError(
                        "Orin fixed-action stream was silent for "
                        f"{self._stream_silence_timeout_s:.3f}s"
                    )
                continue
            event = receive_message(connection)
            last_event_at_s = time.monotonic()
            _validate_event_envelope(event, expected_seq=expected_event_seq)
            expected_event_seq += 1
            event_type = event.get("type")
            if event_type == "status":
                status_callback(_status_from_message(event))
                continue
            _validate_follow_identity(
                event,
                session_id=self._session_id,
                request_id=request_id,
            )
            if event_type == "accepted" and not accepted:
                _require_behavior(event, behavior)
                accepted = True
            elif event_type == "rejected" and not accepted:
                raise FixedActionRejected(
                    _required_text(event, "reason_code"),
                    _required_text(event, "message"),
                )
            elif event_type == "feedback" and accepted:
                feedback_callback(_fixed_feedback_from_message(event, behavior))
            elif event_type == "result" and accepted:
                return _fixed_result_from_message(event, behavior)
            else:
                raise OrinBehaviorProtocolError(
                    f"unexpected {event_type!r} event in fixed-action stream"
                )

    def _request(
        self, message_type: str, request_id: str, **fields: Any
    ) -> dict[str, Any]:
        message = {
            "schema_version": SCHEMA_VERSION,
            "type": message_type,
            "session_id": self._session_id,
            "seq": self._request_sequence,
            "request_id": request_id,
            **fields,
        }
        self._request_sequence += 1
        return message


def _validate_event_envelope(
    event: dict[str, Any],
    *,
    expected_seq: int,
) -> None:
    if event.get("schema_version") != SCHEMA_VERSION:
        raise OrinBehaviorProtocolError("unsupported behavior RPC schema_version")
    if event.get("seq") != expected_seq:
        raise OrinBehaviorProtocolError(
            f"event seq must be {expected_seq}, got {event.get('seq')!r}"
        )


def _validate_follow_identity(
    event: dict[str, Any],
    *,
    session_id: str,
    request_id: str,
) -> None:
    if event.get("session_id") != session_id:
        raise OrinBehaviorProtocolError("event session_id does not match request")
    if event.get("request_id") != request_id:
        raise OrinBehaviorProtocolError("event request_id does not match request")


def _feedback_from_message(
    event: dict[str, Any], trajectory_id: str
) -> FollowFeedback:
    _require_trajectory(event, trajectory_id)
    return FollowFeedback(
        trajectory_id=trajectory_id,
        waypoint_index=_required_uint(event, "waypoint_index"),
        waypoint_count=_required_uint(event, "waypoint_count"),
        distance_m=_required_finite(event, "distance_m"),
        elapsed_s=_required_finite(event, "elapsed_s"),
        bucket_tip_stamp_s=_required_finite(event, "bucket_tip_stamp_s"),
        bucket_tip=_required_point(event, "bucket_tip"),
        tracking_state=_required_text(event, "tracking_state"),
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _result_from_message(event: dict[str, Any], trajectory_id: str) -> FollowResult:
    _require_trajectory(event, trajectory_id)
    outcome = _required_text(event, "outcome")
    if outcome not in {"SUCCEEDED", "CANCELLED", "FAILED"}:
        raise OrinBehaviorProtocolError(f"invalid result outcome: {outcome}")
    quiescence_confirmed = event.get("quiescence_confirmed")
    if not isinstance(quiescence_confirmed, bool):
        raise OrinBehaviorProtocolError("quiescence_confirmed must be bool")
    return FollowResult(
        trajectory_id=trajectory_id,
        outcome=outcome,
        reason_code=_required_text(event, "reason_code"),
        message=_required_text(event, "message"),
        final_waypoint_index=_required_uint(event, "final_waypoint_index"),
        final_distance_m=_required_finite(event, "final_distance_m"),
        quiescence_confirmed=quiescence_confirmed,
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _fixed_feedback_from_message(
    event: dict[str, Any],
    behavior: str,
) -> FixedActionFeedback:
    _require_behavior(event, behavior)
    return FixedActionFeedback(
        behavior=behavior,
        step_index=_required_uint(event, "step_index"),
        step_label=_required_text(event, "step_label"),
        phase=_required_text(event, "phase"),
        max_error=_required_finite(event, "max_error"),
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _fixed_result_from_message(
    event: dict[str, Any],
    behavior: str,
) -> FixedActionResult:
    _require_behavior(event, behavior)
    outcome = _required_text(event, "outcome")
    if outcome not in {"SUCCEEDED", "CANCELLED", "FAILED"}:
        raise OrinBehaviorProtocolError(f"invalid result outcome: {outcome}")
    return FixedActionResult(
        behavior=behavior,
        outcome=outcome,
        reason_code=_required_text(event, "reason_code"),
        message=_required_text(event, "message"),
        final_step_index=_required_uint(event, "final_step_index"),
        final_step_label=_fixed_final_step_label(event, outcome),
        final_max_error=_required_finite(event, "final_max_error"),
        quiescence_confirmed=_required_bool(event, "quiescence_confirmed"),
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _fixed_final_step_label(event: dict[str, Any], outcome: str) -> str:
    value = event.get("final_step_label")
    if value == "" and outcome in {"FAILED", "CANCELLED"}:
        return "not_started"
    return _required_text(event, "final_step_label")


def _status_from_message(event: dict[str, Any]) -> OrinRuntimeStatus:
    return OrinRuntimeStatus(
        state_fresh=_required_bool(event, "state_fresh"),
        control_enabled=_required_bool(event, "control_enabled"),
        sensor_valid=_required_bool(event, "sensor_valid"),
        stm32_alive=_required_bool(event, "stm32_alive"),
        estop=_required_bool(event, "estop"),
        fault_free=_required_bool(event, "fault_free"),
        quiescent=_required_bool(event, "quiescent"),
        active_behavior=_required_optional_text(event, "active_behavior"),
        action_datagrams=_required_uint(event, "action_datagrams"),
        motion_gate_reason=_required_text(event, "motion_gate_reason"),
        sender_constructed=_required_bool(event, "sender_constructed"),
        fixed_actions_available=_optional_bool(
            event,
            "fixed_actions_available",
            False,
        ),
        fixed_actions_validated=_optional_bool(
            event,
            "fixed_actions_validated",
            False,
        ),
    )


def _require_trajectory(event: dict[str, Any], expected: str) -> None:
    if event.get("trajectory_id") != expected:
        raise OrinBehaviorProtocolError("event trajectory_id does not match request")


def _require_behavior(event: dict[str, Any], expected: str) -> None:
    if event.get("behavior") != expected:
        raise OrinBehaviorProtocolError("event behavior does not match request")


def _required_text(value: dict[str, Any], field: str) -> str:
    result = value.get(field)
    if not isinstance(result, str) or not result:
        raise OrinBehaviorProtocolError(f"{field} must be non-empty text")
    return result


def _required_optional_text(value: dict[str, Any], field: str) -> str:
    result = value.get(field)
    if not isinstance(result, str):
        raise OrinBehaviorProtocolError(f"{field} must be text")
    return result


def _required_bool(value: dict[str, Any], field: str) -> bool:
    result = value.get(field)
    if not isinstance(result, bool):
        raise OrinBehaviorProtocolError(f"{field} must be bool")
    return result


def _optional_bool(
    value: dict[str, Any],
    field: str,
    default: bool,
) -> bool:
    if field not in value:
        return default
    return _required_bool(value, field)


def _required_uint(value: dict[str, Any], field: str) -> int:
    result = value.get(field)
    if isinstance(result, bool) or not isinstance(result, int) or result < 0:
        raise OrinBehaviorProtocolError(f"{field} must be a nonnegative integer")
    return result


def _required_finite(value: dict[str, Any], field: str) -> float:
    result = value.get(field)
    if (
        isinstance(result, bool)
        or not isinstance(result, (int, float))
        or not math.isfinite(result)
    ):
        raise OrinBehaviorProtocolError(f"{field} must be finite")
    return float(result)


def _required_point(
    value: dict[str, Any], field: str
) -> tuple[float, float, float]:
    point = value.get(field)
    if not isinstance(point, list) or len(point) != 3:
        raise OrinBehaviorProtocolError(f"{field} must contain three values")
    converted = tuple(_finite_point_value(item, field) for item in point)
    return converted  # type: ignore[return-value]


def _finite_point_value(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise OrinBehaviorProtocolError(f"{field} must contain finite values")
    return float(value)
