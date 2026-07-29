#!/usr/bin/python3
"""ROS gateway to Orin-hosted Follow and fixed-action runtimes."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

import rclpy
from airy_excavator_interfaces.action import ExecuteDig, ExecuteDump, Follow
from airy_excavator_interfaces.msg import RuntimeStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from runtime_bridge.control_stage import CONTROL_STAGES, control_stage_policy
from runtime_bridge.live_control import motion_authorization_granted
from runtime_bridge.orin_behavior_rpc import (
    OrinBehaviorConnectionError,
    OrinBehaviorProtocolError,
    trajectory_snapshot_to_message,
)
from runtime_bridge.orin_follow_client import (
    FixedActionFeedback,
    FixedActionRejected,
    FixedActionResult,
    FollowFeedback,
    FollowRejected,
    FollowResult,
    OrinFollowClient,
    OrinRuntimeStatus,
    OrinStatusMonitor,
)
from mission.follow import FollowTrajectorySnapshot


FOLLOW_ACTION = "/excavator/follow"
EXECUTE_DIG_ACTION = "/excavator/execute_dig"
EXECUTE_DUMP_ACTION = "/excavator/execute_dump"
RUNTIME_STATUS_TOPIC = "/mission/runtime_status"
STATUS_MAX_AGE_S = 0.6


class OrinEdgeFollowGatewayNode(Node):
    """Expose the existing ROS Follow Action while Orin owns inference and motion."""

    def __init__(
        self,
        *,
        orin_host: str,
        orin_port: int,
        motion_authorization: str,
        control_stage: str,
        follow_action: str = FOLLOW_ACTION,
        execute_dig_action: str = EXECUTE_DIG_ACTION,
        execute_dump_action: str = EXECUTE_DUMP_ACTION,
        runtime_status_topic: str = RUNTIME_STATUS_TOPIC,
        context=None,
    ) -> None:
        super().__init__("orin_edge_follow_gateway", context=context)
        if not motion_authorization_granted(motion_authorization):
            raise ValueError("Orin Edge Follow requires exact PC motion authorization")
        self._control_stage = control_stage_policy(control_stage).name
        self._client = OrinFollowClient(orin_host, orin_port)
        self._status_monitor = OrinStatusMonitor(orin_host, orin_port)
        self._status_monitor.start()
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._active_behavior = ""
        self._last_rejection_reason = ""
        self._last_rejection_message = ""
        self._stream_status: OrinRuntimeStatus | None = None
        self._stream_status_at_s = 0.0

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            RuntimeStatus, runtime_status_topic, latched_qos
        )
        self._action_server = ActionServer(
            self,
            Follow,
            follow_action,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._execute_dig_server = ActionServer(
            self,
            ExecuteDig,
            execute_dig_action,
            execute_callback=lambda handle: self._execute_fixed(
                handle,
                "ExecuteDig",
                ExecuteDig,
            ),
            goal_callback=lambda request: self._on_fixed_goal(
                request,
                "ExecuteDig",
            ),
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._execute_dump_server = ActionServer(
            self,
            ExecuteDump,
            execute_dump_action,
            execute_callback=lambda handle: self._execute_fixed(
                handle,
                "ExecuteDump",
                ExecuteDump,
            ),
            goal_callback=lambda request: self._on_fixed_goal(
                request,
                "ExecuteDump",
            ),
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._status_timer = self.create_timer(
            0.2, self._publish_status, callback_group=self._callback_group
        )
        self._publish_status()

    def destroy_node(self):
        self._status_timer.cancel()
        self._action_server.destroy()
        self._execute_dig_server.destroy()
        self._execute_dump_server.destroy()
        self._status_monitor.close()
        return super().destroy_node()

    def _on_goal(self, request: Follow.Goal) -> GoalResponse:
        try:
            self._validate_trajectory(request.trajectory)
        except (TypeError, ValueError) as exc:
            return self._reject("INVALID_TRAJECTORY", str(exc))
        remote = self._remote_status()
        if not _remote_goal_admissible(remote):
            reason = (
                remote.motion_gate_reason
                if remote is not None
                else "orin_status_unavailable"
            )
            return self._reject(
                "ORIN_NOT_READY", f"Orin Edge Follow is not ready: {reason}"
            )
        with self._lock:
            if self._active_behavior:
                return self._reject_locked("BUSY", "another Follow owns the gateway")
            self._active_behavior = "Follow"
            self._last_rejection_reason = ""
            self._last_rejection_message = ""
        self._publish_status()
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _on_fixed_goal(self, _request, behavior: str) -> GoalResponse:
        remote = self._remote_status()
        if not _remote_fixed_ready(remote):
            reason = (
                remote.motion_gate_reason
                if remote is not None
                else "orin_status_unavailable"
            )
            return self._reject(
                "ORIN_FIXED_ACTION_NOT_READY",
                f"Orin {behavior} is not ready: {reason}",
            )
        with self._lock:
            if self._active_behavior:
                return self._reject_locked(
                    "BUSY",
                    "another behavior owns the gateway",
                )
            self._active_behavior = behavior
            self._last_rejection_reason = ""
            self._last_rejection_message = ""
        self._publish_status()
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle) -> Follow.Result:
        trajectory = trajectory_snapshot_to_message(goal_handle.request.trajectory)
        try:
            remote = self._client.run_follow(
                trajectory,
                feedback_callback=lambda update: self._publish_feedback(
                    goal_handle, update
                ),
                status_callback=self._on_stream_status,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )
            return self._finish_remote(goal_handle, remote)
        except FollowRejected as exc:
            self._record_rejection(exc.reason_code, exc.message)
            status = self._remote_status()
            return self._finish_error(
                goal_handle,
                reason_code=exc.reason_code,
                message=exc.message,
                quiescence_confirmed=bool(status and status.quiescent),
            )
        except (OrinBehaviorConnectionError, OrinBehaviorProtocolError) as exc:
            self.get_logger().error(f"Orin Follow RPC failed: {exc}")
            return self._finish_error(
                goal_handle,
                reason_code="ORIN_RPC_ERROR",
                message=str(exc),
                quiescence_confirmed=False,
            )
        except Exception as exc:
            self.get_logger().error(f"Orin Follow gateway failed: {exc}")
            return self._finish_error(
                goal_handle,
                reason_code="GATEWAY_ERROR",
                message=str(exc),
                quiescence_confirmed=False,
            )
        finally:
            with self._lock:
                self._active_behavior = ""
                self._stream_status = None
                self._stream_status_at_s = 0.0
            self._publish_status()

    def _execute_fixed(self, goal_handle, behavior: str, action_type):
        try:
            remote = self._client.run_fixed_action(
                behavior,
                feedback_callback=lambda update: self._publish_fixed_feedback(
                    goal_handle,
                    action_type,
                    update,
                ),
                status_callback=self._on_stream_status,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )
            return self._finish_fixed_remote(
                goal_handle,
                action_type,
                remote,
            )
        except FixedActionRejected as exc:
            self._record_rejection(exc.reason_code, exc.message)
            status = self._remote_status()
            return self._finish_fixed_error(
                goal_handle,
                action_type,
                reason_code=exc.reason_code,
                message=exc.message,
                quiescence_confirmed=bool(status and status.quiescent),
            )
        except (OrinBehaviorConnectionError, OrinBehaviorProtocolError) as exc:
            self.get_logger().error(f"Orin {behavior} RPC failed: {exc}")
            return self._finish_fixed_error(
                goal_handle,
                action_type,
                reason_code="ORIN_RPC_ERROR",
                message=str(exc),
                quiescence_confirmed=False,
            )
        except Exception as exc:
            self.get_logger().error(f"Orin {behavior} gateway failed: {exc}")
            return self._finish_fixed_error(
                goal_handle,
                action_type,
                reason_code="GATEWAY_ERROR",
                message=str(exc),
                quiescence_confirmed=False,
            )
        finally:
            with self._lock:
                self._active_behavior = ""
                self._stream_status = None
                self._stream_status_at_s = 0.0
            self._publish_status()

    def _finish_remote(
        self, goal_handle, remote: FollowResult
    ) -> Follow.Result:
        result = Follow.Result()
        outcome = {
            "SUCCEEDED": Follow.Result.OUTCOME_SUCCEEDED,
            "CANCELLED": Follow.Result.OUTCOME_CANCELLED,
            "FAILED": Follow.Result.OUTCOME_FAILED,
        }[remote.outcome]
        result.outcome = outcome
        result.reason_code = remote.reason_code
        result.message = remote.message
        result.final_waypoint_index = remote.final_waypoint_index
        result.final_distance_m = remote.final_distance_m
        result.quiescence_confirmed = remote.quiescence_confirmed
        result.action_datagrams = remote.action_datagrams
        if not remote.quiescence_confirmed:
            result.outcome = Follow.Result.OUTCOME_FAILED
            result.reason_code = "ORIN_QUIESCENCE_UNCONFIRMED"
            result.message = "Orin returned a terminal result without quiescence"
            goal_handle.abort()
        elif remote.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif remote.outcome == "CANCELLED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _finish_error(
        self,
        goal_handle,
        *,
        reason_code: str,
        message: str,
        quiescence_confirmed: bool,
    ) -> Follow.Result:
        goal_handle.abort()
        result = Follow.Result()
        result.outcome = Follow.Result.OUTCOME_FAILED
        result.reason_code = reason_code
        result.message = message
        result.final_waypoint_index = 0
        result.final_distance_m = -1.0
        result.quiescence_confirmed = quiescence_confirmed
        result.action_datagrams = 0
        return result

    def _finish_fixed_remote(
        self,
        goal_handle,
        action_type,
        remote: FixedActionResult,
    ):
        result = action_type.Result()
        result.outcome = {
            "SUCCEEDED": action_type.Result.OUTCOME_SUCCEEDED,
            "CANCELLED": action_type.Result.OUTCOME_CANCELLED,
            "FAILED": action_type.Result.OUTCOME_FAILED,
        }[remote.outcome]
        result.reason_code = remote.reason_code
        result.message = remote.message
        result.quiescence_confirmed = remote.quiescence_confirmed
        result.action_datagrams = remote.action_datagrams
        if not remote.quiescence_confirmed:
            result.outcome = action_type.Result.OUTCOME_FAILED
            result.reason_code = "ORIN_QUIESCENCE_UNCONFIRMED"
            result.message = "Orin returned a terminal result without quiescence"
            goal_handle.abort()
        elif remote.outcome == "SUCCEEDED":
            goal_handle.succeed()
        elif remote.outcome == "CANCELLED":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _finish_fixed_error(
        self,
        goal_handle,
        action_type,
        *,
        reason_code: str,
        message: str,
        quiescence_confirmed: bool,
    ):
        goal_handle.abort()
        result = action_type.Result()
        result.outcome = action_type.Result.OUTCOME_FAILED
        result.reason_code = reason_code
        result.message = message
        result.quiescence_confirmed = quiescence_confirmed
        result.action_datagrams = 0
        return result

    def _publish_feedback(
        self, goal_handle, remote: FollowFeedback
    ) -> None:
        feedback = Follow.Feedback()
        feedback.bucket_tip_stamp = _time_message(remote.bucket_tip_stamp_s)
        feedback.bucket_tip = Point(
            x=remote.bucket_tip[0],
            y=remote.bucket_tip[1],
            z=remote.bucket_tip[2],
        )
        feedback.current_waypoint_index = remote.waypoint_index
        feedback.waypoint_count = remote.waypoint_count
        feedback.distance_m = remote.distance_m
        feedback.elapsed_s = remote.elapsed_s
        feedback.tracking_state = remote.tracking_state
        feedback.action_datagrams = remote.action_datagrams
        goal_handle.publish_feedback(feedback)

    def _publish_fixed_feedback(
        self,
        goal_handle,
        action_type,
        remote: FixedActionFeedback,
    ) -> None:
        feedback = action_type.Feedback()
        feedback.step_index = remote.step_index
        feedback.step_label = remote.step_label
        feedback.phase = remote.phase
        feedback.max_error = remote.max_error
        feedback.action_datagrams = remote.action_datagrams
        goal_handle.publish_feedback(feedback)

    def _validate_trajectory(self, snapshot) -> None:
        fields = trajectory_snapshot_to_message(snapshot)
        fields["waypoints"] = tuple(tuple(point) for point in fields["waypoints"])
        canonical = FollowTrajectorySnapshot(**fields)
        canonical.validate_for_execution(
            now_s=self.get_clock().now().nanoseconds * 1e-9,
            expected_control_stage=self._control_stage,
        )

    def _on_stream_status(self, status: OrinRuntimeStatus) -> None:
        with self._lock:
            self._stream_status = status
            self._stream_status_at_s = time.monotonic()

    def _remote_status(self) -> OrinRuntimeStatus | None:
        with self._lock:
            stream_status = self._stream_status
            stream_age_s = time.monotonic() - self._stream_status_at_s
        if stream_status is not None and stream_age_s <= STATUS_MAX_AGE_S:
            return stream_status
        return self._status_monitor.latest(max_age_s=STATUS_MAX_AGE_S)

    def _publish_status(self) -> None:
        remote = self._remote_status()
        with self._lock:
            local_active = self._active_behavior
            rejection_reason = self._last_rejection_reason
            rejection_message = self._last_rejection_message
        status = _runtime_status_message(
            remote,
            stamp=self.get_clock().now().to_msg(),
            control_stage=self._control_stage,
            local_active=local_active,
            rejection_reason=rejection_reason,
            rejection_message=rejection_message,
        )
        self._status_publisher.publish(status)

    def _reject(self, reason: str, message: str) -> GoalResponse:
        with self._lock:
            return self._reject_locked(reason, message)

    def _reject_locked(self, reason: str, message: str) -> GoalResponse:
        self._last_rejection_reason = reason
        self._last_rejection_message = message
        return GoalResponse.REJECT

    def _record_rejection(self, reason: str, message: str) -> None:
        with self._lock:
            self._last_rejection_reason = reason
            self._last_rejection_message = message
        self._publish_status()


def _runtime_status_message(
    remote: OrinRuntimeStatus | None,
    *,
    stamp,
    control_stage: str,
    local_active: str,
    rejection_reason: str,
    rejection_message: str,
) -> RuntimeStatus:
    status = RuntimeStatus()
    status.header.stamp = stamp
    status.header.frame_id = "machine_root_ros"
    status.input_source = "live"
    status.execution_mode = "control"
    status.control_stage = control_stage
    status.motion_backend = "orin_edge"
    status.motion_authorized = True
    status.sender_constructed = bool(remote and remote.sender_constructed)
    status.quiescent = bool(remote and remote.quiescent and not local_active)
    status.action_datagrams = remote.action_datagrams if remote else 0
    status.state_fresh = bool(remote and remote.state_fresh)
    status.control_enabled = bool(remote and remote.control_enabled)
    status.sensor_valid = bool(remote and remote.sensor_valid)
    status.stm32_alive = bool(remote and remote.stm32_alive)
    status.estop = remote.estop if remote else True
    status.fault_free = bool(remote and remote.fault_free)
    status.fixed_actions_validated = bool(
        remote and remote.fixed_actions_validated
    )
    status.manual_jog_ready = False
    status.follow_control_mode = "edge_onnx"
    status.follow_speed_fraction = 1.0
    status.follow_allowed_actuators = ["boom", "stick", "bucket", "swing"]
    status.follow_max_motion_ms = 0
    status.follow_canary_ready = False
    status.follow_supervision_active = bool(local_active and remote)
    status.motion_gate_reason = (
        "behavior_active"
        if local_active
        else (remote.motion_gate_reason if remote else "orin_status_unavailable")
    )
    status.active_behavior = local_active or (remote.active_behavior if remote else "")
    status.last_rejection_reason = rejection_reason
    status.last_rejection_message = rejection_message
    return status


def _remote_ready(remote: OrinRuntimeStatus | None) -> bool:
    return bool(
        remote
        and remote.sender_constructed
        and remote.state_fresh
        and remote.control_enabled
        and remote.sensor_valid
        and remote.stm32_alive
        and not remote.estop
        and remote.fault_free
        and remote.quiescent
        and not remote.active_behavior
        and remote.motion_gate_reason == "ready"
    )


def _remote_goal_admissible(remote: OrinRuntimeStatus | None) -> bool:
    if _remote_ready(remote):
        return True
    # The status-only connection runs slower than behavior results. Orin emits
    # a successful result only after clearing the active runner and sending its
    # terminal zero, so a safe cached "behavior_active" sample may lag during
    # handoff. The new RPC still passes through Orin's current motion gate.
    return bool(
        remote
        and remote.sender_constructed
        and remote.state_fresh
        and remote.control_enabled
        and remote.sensor_valid
        and remote.stm32_alive
        and not remote.estop
        and remote.fault_free
        and not remote.quiescent
        and remote.active_behavior
        and remote.motion_gate_reason == "behavior_active"
    )


def _remote_fixed_ready(remote: OrinRuntimeStatus | None) -> bool:
    return bool(_remote_goal_admissible(remote) and remote.fixed_actions_available)


def _time_message(value: float) -> Time:
    seconds = math.floor(value)
    nanoseconds = round((value - seconds) * 1e9)
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=int(seconds), nanosec=int(nanoseconds))


def _port(value: str) -> int:
    converted = int(value)
    if not 1 <= converted <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return converted


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Expose ROS Follow through the Orin Edge ONNX runtime"
    )
    parser.add_argument("--orin-host", required=True)
    parser.add_argument("--orin-port", type=_port, required=True)
    parser.add_argument("--motion-authorization", required=True)
    parser.add_argument("--control-stage", choices=CONTROL_STAGES, required=True)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rclpy.init()
    try:
        node = OrinEdgeFollowGatewayNode(
            orin_host=args.orin_host,
            orin_port=args.orin_port,
            motion_authorization=args.motion_authorization,
            control_stage=args.control_stage,
        )
    except ValueError as exc:
        print(f"Orin Edge Follow gateway configuration error: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        return 2
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
