"""PC client for Orin-local excavation-cycle motion legs."""

from __future__ import annotations

import select
import socket
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable

from runtime_bridge.orin_behavior_rpc import (
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
    receive_message,
    send_message,
)
from runtime_bridge.orin_follow_client import (
    OrinFollowClient,
    OrinRuntimeStatus,
    _required_bool,
    _required_text,
    _required_uint,
    _status_from_message,
    _validate_event_envelope,
    _validate_follow_identity,
)


@dataclass(frozen=True)
class CycleLegFeedback:
    cycle_id: str
    stage: str
    behavior: str
    message: str
    action_datagrams: int


@dataclass(frozen=True)
class CycleLegResult:
    cycle_id: str
    outcome: str
    reason_code: str
    message: str
    completed_stage: str
    quiescence_confirmed: bool
    action_datagrams: int


class CycleLegRejected(RuntimeError):
    """Orin rejected a cycle-leg request."""

    def __init__(self, reason_code: str, message: str) -> None:
        super().__init__(f"{reason_code}: {message}")
        self.reason_code = reason_code
        self.message = message


class OrinCycleClient(OrinFollowClient):
    """Run one Orin-local excavation cycle leg at a time."""

    def run_cycle_dig_leg(
        self,
        cycle_id: str,
        trajectory: dict[str, Any],
        *,
        feedback_callback: Callable[[CycleLegFeedback], None] | None = None,
        status_callback: Callable[[OrinRuntimeStatus], None] | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> CycleLegResult:
        return self._run_cycle_leg(
            cycle_id=cycle_id,
            trajectory=trajectory,
            request_type="start_cycle",
            trajectory_field="dig_trajectory",
            expected_stage="FOLLOW_DIG",
            feedback_callback=feedback_callback,
            status_callback=status_callback,
            cancel_requested=cancel_requested,
        )

    def run_cycle_dump_leg(
        self,
        cycle_id: str,
        trajectory: dict[str, Any],
        *,
        feedback_callback: Callable[[CycleLegFeedback], None] | None = None,
        status_callback: Callable[[OrinRuntimeStatus], None] | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> CycleLegResult:
        return self._run_cycle_leg(
            cycle_id=cycle_id,
            trajectory=trajectory,
            request_type="provide_dump_trajectory",
            trajectory_field="dump_trajectory",
            expected_stage="FOLLOW_DUMP",
            feedback_callback=feedback_callback,
            status_callback=status_callback,
            cancel_requested=cancel_requested,
        )

    def cancel_cycle(self, cycle_id: str) -> CycleLegResult:
        """Cancel a cycle that is quiescent at a PC planning boundary."""
        self._validate_cycle_id(cycle_id)
        with self._lock:
            request_id = f"cancel-cycle-{uuid.uuid4().hex}"
            with self._connect() as connection:
                send_message(
                    connection,
                    self._request(
                        "cancel_cycle",
                        request_id,
                        cycle_id=cycle_id,
                    ),
                )
                return self._receive_cycle_cancel(
                    connection,
                    request_id=request_id,
                    cycle_id=cycle_id,
                )

    def _run_cycle_leg(
        self,
        *,
        cycle_id: str,
        trajectory: dict[str, Any],
        request_type: str,
        trajectory_field: str,
        expected_stage: str,
        feedback_callback: Callable[[CycleLegFeedback], None] | None,
        status_callback: Callable[[OrinRuntimeStatus], None] | None,
        cancel_requested: Callable[[], bool] | None,
    ) -> CycleLegResult:
        self._validate_cycle_id(cycle_id)
        if not isinstance(trajectory, dict):
            raise ValueError("trajectory must be a dict")
        on_feedback = feedback_callback or (lambda _feedback: None)
        on_status = status_callback or (lambda _status: None)
        should_cancel = cancel_requested or (lambda: False)
        with self._lock:
            request_id = "%s-%s" % (
                request_type.replace("_", "-"),
                uuid.uuid4().hex,
            )
            with self._connect() as connection:
                send_message(
                    connection,
                    self._request(
                        request_type,
                        request_id,
                        cycle_id=cycle_id,
                        **{trajectory_field: trajectory},
                    ),
                )
                return self._receive_cycle_leg(
                    connection,
                    request_id=request_id,
                    cycle_id=cycle_id,
                    expected_stage=expected_stage,
                    feedback_callback=on_feedback,
                    status_callback=on_status,
                    cancel_requested=should_cancel,
                )

    def _connect(self) -> socket.socket:
        try:
            connection = socket.create_connection(
                self._endpoint,
                timeout=self._connect_timeout_s,
            )
        except OSError as exc:
            raise OrinBehaviorConnectionError(
                f"failed to connect to Orin behavior RPC: {exc}"
            ) from exc
        connection.settimeout(None)
        return connection

    def _receive_cycle_leg(
        self,
        connection: socket.socket,
        *,
        request_id: str,
        cycle_id: str,
        expected_stage: str,
        feedback_callback: Callable[[CycleLegFeedback], None],
        status_callback: Callable[[OrinRuntimeStatus], None],
        cancel_requested: Callable[[], bool],
    ) -> CycleLegResult:
        accepted = False
        cancel_sent = False
        stream = _StreamState()
        while True:
            if cancel_requested() and not cancel_sent:
                send_message(
                    connection,
                    self._request(
                        "cancel_cycle",
                        request_id,
                        cycle_id=cycle_id,
                    ),
                )
                cancel_sent = True
            event = self._next_event(
                connection,
                stream=stream,
                stream_name="cycle-leg",
                request_id=request_id,
                cycle_id=cycle_id,
                accepted=accepted,
            )
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
                _require_cycle(event, cycle_id)
                if _required_text(event, "stage") != expected_stage:
                    raise OrinBehaviorProtocolError(
                        "accepted cycle stage does not match request"
                    )
                accepted = True
            elif event_type == "rejected" and not accepted:
                raise CycleLegRejected(
                    _required_text(event, "reason_code"),
                    _required_text(event, "message"),
                )
            elif event_type == "feedback" and accepted:
                feedback_callback(_feedback_from_message(event, cycle_id))
            elif event_type == "result" and accepted:
                return _result_from_message(event, cycle_id)
            else:
                raise OrinBehaviorProtocolError(
                    f"unexpected {event_type!r} event in cycle-leg stream"
                )

    def _receive_cycle_cancel(
        self,
        connection: socket.socket,
        *,
        request_id: str,
        cycle_id: str,
    ) -> CycleLegResult:
        stream = _StreamState()
        while True:
            event = self._next_event(
                connection,
                stream=stream,
                stream_name="cycle-cancel",
                request_id=request_id,
                cycle_id=cycle_id,
                accepted=False,
            )
            event_type = event.get("type")
            if event_type == "status":
                continue
            _validate_follow_identity(
                event,
                session_id=self._session_id,
                request_id=request_id,
            )
            if event_type == "rejected":
                raise CycleLegRejected(
                    _required_text(event, "reason_code"),
                    _required_text(event, "message"),
                )
            if event_type == "result":
                return _result_from_message(event, cycle_id)
            raise OrinBehaviorProtocolError(
                "unexpected event in cycle-cancel stream"
            )

    def _next_event(
        self,
        connection: socket.socket,
        *,
        stream: "_StreamState",
        stream_name: str,
        request_id: str,
        cycle_id: str,
        accepted: bool,
    ) -> dict[str, Any]:
        while True:
            readable, _, _ = select.select(
                [connection],
                [],
                [],
                self._poll_interval_s,
            )
            if readable:
                event = receive_message(connection)
                stream.record(event)
                return event
            silence_s = time.monotonic() - stream.last_event_at_s
            if silence_s > self._stream_silence_timeout_s:
                raise OrinBehaviorConnectionError(
                    self._stream_silence_diagnostic(
                        stream_name=stream_name,
                        request_id=request_id,
                        accepted=accepted,
                        silence_s=silence_s,
                        last_event_type=stream.last_event_type,
                        last_event_seq=stream.last_event_seq,
                        events_received=stream.events_received,
                        operation_field="cycle_id",
                        operation_value=cycle_id,
                    )
                )

    @staticmethod
    def _validate_cycle_id(cycle_id: str) -> None:
        if not isinstance(cycle_id, str) or not cycle_id.strip():
            raise ValueError("cycle_id must be non-empty")


class _StreamState:
    def __init__(self) -> None:
        self.expected_seq = 0
        self.last_event_at_s = time.monotonic()
        self.last_event_type = "none"
        self.last_event_seq = -1
        self.events_received = 0

    def record(self, event: dict[str, Any]) -> None:
        _validate_event_envelope(event, expected_seq=self.expected_seq)
        self.last_event_at_s = time.monotonic()
        self.last_event_type = str(event.get("type", "unknown"))
        self.last_event_seq = self.expected_seq
        self.events_received += 1
        self.expected_seq += 1


def _feedback_from_message(
    event: dict[str, Any],
    cycle_id: str,
) -> CycleLegFeedback:
    _require_cycle(event, cycle_id)
    return CycleLegFeedback(
        cycle_id=cycle_id,
        stage=_required_text(event, "stage"),
        behavior=_required_text(event, "behavior"),
        message=_required_text(event, "message"),
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _result_from_message(
    event: dict[str, Any],
    cycle_id: str,
) -> CycleLegResult:
    _require_cycle(event, cycle_id)
    outcome = _required_text(event, "outcome")
    if outcome not in {"SUCCEEDED", "CANCELLED", "FAILED"}:
        raise OrinBehaviorProtocolError(f"invalid result outcome: {outcome}")
    return CycleLegResult(
        cycle_id=cycle_id,
        outcome=outcome,
        reason_code=_required_text(event, "reason_code"),
        message=_required_text(event, "message"),
        completed_stage=_required_text(event, "completed_stage"),
        quiescence_confirmed=_required_bool(event, "quiescence_confirmed"),
        action_datagrams=_required_uint(event, "action_datagrams"),
    )


def _require_cycle(event: dict[str, Any], expected: str) -> None:
    if event.get("cycle_id") != expected:
        raise OrinBehaviorProtocolError("event cycle_id does not match request")
