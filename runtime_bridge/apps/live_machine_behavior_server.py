#!/usr/bin/env python3
"""Single PC Command Sink exposing live machine behavior and diagnostic jog Actions."""

from __future__ import annotations

import math
import re
import socket
import sys
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path


AIRY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(AIRY_ROOT))

import rclpy
from airy_excavator_interfaces.action import ExecuteDig, ExecuteDump, Follow, HoldToJog
from airy_excavator_interfaces.msg import (
    JogHeartbeat,
    OperatorHeartbeat,
    RuntimeStatus,
    TrajectorySnapshot,
)
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mission.follow import FollowSession, TrajectoryDigestMismatch
from mission.contract import load_mission
from mission.runtime_ros.follow_action_server import _feedback, _snapshot_from_message
from runtime_bridge.action_journal import ActionJournalUnavailable, RecordedUdpSender
from runtime_bridge.fixed_actions import (
    FixedActionExecutor,
    load_fixed_action_profile,
    physical_velocity_action_from_normalized,
)
from runtime_bridge.follow_audit import (
    FollowAuditUnavailable,
    FollowAuditWriter,
    build_follow_audit_record,
)
from runtime_bridge.live_control import (
    FollowCanaryEnvelope,
    MotionCommandSink,
    build_manual_jog_action,
    build_dynamic_waypoint_values,
    evaluate_actuator_state,
    evaluate_follow_canary_supervision,
    evaluate_motion_state,
    evaluate_state_provenance,
    motion_authorization_granted,
)
from runtime_bridge.control_stage import CONTROL_STAGES, control_stage_policy
from runtime_bridge.observation import ObservationBuilder, load_machine_profile
from runtime_bridge.onnx_policy import OnnxPolicy
from runtime_bridge.protocol import (
    MachineStatePacket,
    PacketDecodeError,
    decode_packet,
    estimate_remote_now_ms,
    now_ms,
)
from runtime_bridge.ros_provenance import set_ros_header_stamp
from runtime_bridge.runtime_config import load_runtime_config
from runtime_bridge.unity_observation_adapter import UnityObservationAdapter


FOLLOW_ACTION = "/excavator/follow"
DIG_ACTION = "/excavator/execute_dig"
DUMP_ACTION = "/excavator/execute_dump"
JOG_ACTION = "/excavator/hold_to_jog"
JOG_HEARTBEAT_TOPIC = "/excavator/jog_heartbeat"
OPERATOR_HEARTBEAT_TOPIC = "/excavator/operator_heartbeat"
TIP_TOPIC = "/bucket_tip_pose_machine_root_ros"
STATUS_TOPIC = "/mission/runtime_status"
_SESSION_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{8,128}$")


@dataclass(frozen=True)
class StateSample:
    sequence: int
    packet: MachineStatePacket
    received_pc_ms: int
    received_monotonic_s: float


class LiveMachineBehaviorNode(Node):
    """Own the only live state socket, behavior lease and UDP action sender."""

    def __init__(
        self,
        *,
        config_path: Path,
        mission_path: Path,
        motion_authorization: str,
        control_stage: str,
        workspace_root: Path | None = None,
        context=None,
    ) -> None:
        super().__init__("live_machine_behavior_server", context=context)
        if not motion_authorization_granted(motion_authorization):
            raise ValueError("live Command Sink requires exact motion authorization")
        self._config = load_runtime_config(config_path)
        self._control_policy = control_stage_policy(control_stage)
        if not self._control_policy.enforce_actuator_position_bounds:
            self.get_logger().warning(
                "COMMISSIONING: provisional actuator position bounds are diagnostic only; "
                "non-finite actuator states remain blocked"
            )
        self._config.artifacts.require_live_control_inputs()
        self._machine_profile = load_machine_profile(self._config.artifacts.machine_profile)
        self._fixed_action_profile = load_fixed_action_profile(
            self._config.artifacts.fixed_action_profile,
            machine_profile_path=self._config.artifacts.machine_profile,
            urdf_path=self._config.artifacts.urdf,
            expected_sha256=self._config.fixed_action.expected_profile_sha256,
            workspace_root=workspace_root or AIRY_ROOT.parent,
        )
        self._mission = load_mission(mission_path)
        if (
            not self._control_policy.require_field_validated_targets
            and self._mission.target_status != "field_validated"
        ):
            self.get_logger().warning(
                "COMMISSIONING: Mission targets are not field_validated; "
                "RViz-adjusted targets are accepted for supervised Follow"
            )
        self._policy = OnnxPolicy(self._config.artifacts.onnx)
        self._follow_envelope = FollowCanaryEnvelope.from_machine_profile(
            self._machine_profile,
            allowed_actuators=self._config.follow_control.allowed_actuators,
        )
        self._adapter = UnityObservationAdapter()
        self._max_state_age_s = self._config.policy.machine_state_timeout_ms / 1000.0
        self._bucket_tip_timeout_s = (
            self._config.policy.bucket_tip_timeout_ms / 1000.0
        )
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._state_sequence = 0
        self._last_remote_state_seq: int | None = None
        self._last_remote_state_stamp_ms: int | None = None
        self._remote_clock_offset_ms: int | None = None
        self._latest_state: StateSample | None = None
        self._states: dict[int, StateSample] = {}
        self._tips: dict[int, PoseStamped] = {}
        self._planned_trajectories: dict[str, str] = {}
        self._active_behavior = ""
        self._last_rejection_reason = ""
        self._last_rejection_message = ""
        self._jog_heartbeat_session = ""
        self._jog_heartbeat_monotonic_s = 0.0
        self._operator_heartbeat_behavior = ""
        self._operator_heartbeat_session = ""
        self._operator_heartbeat_monotonic_s = 0.0
        self._stopping = False

        self._state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_socket.settimeout(0.1)
        self._state_socket.bind(self._config.network.state_endpoint)
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recorded_sender = RecordedUdpSender(
            self._send_socket,
            self._config.network.action_endpoint,
            journal_config=self._config.action_journal,
            source="live_machine_behavior_server",
        )
        self._follow_audit = FollowAuditWriter(
            self._config.action_journal.directory / "follow_canary",
            self._config.artifacts.latest_observation,
        )
        self._follow_audit_sequence = 0
        self._command_sink = MotionCommandSink(
            self._recorded_sender,
            valid_for_ms=self._config.network.action_valid_ms,
            max_state_age_s=self._max_state_age_s,
            physical_action_limits=_physical_action_limits(self._machine_profile),
        )

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(RuntimeStatus, STATUS_TOPIC, latched_qos)
        self._joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            PoseStamped,
            TIP_TOPIC,
            self._on_tip,
            10,
            callback_group=self._callback_group,
        )
        self._jog_heartbeat_subscription = self.create_subscription(
            JogHeartbeat,
            JOG_HEARTBEAT_TOPIC,
            self._on_jog_heartbeat,
            10,
            callback_group=self._callback_group,
        )
        self._operator_heartbeat_subscription = self.create_subscription(
            OperatorHeartbeat,
            OPERATOR_HEARTBEAT_TOPIC,
            self._on_operator_heartbeat,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            TrajectorySnapshot,
            "/planning/trajectory_snapshot",
            self._on_planned_trajectory,
            latched_qos,
            callback_group=self._callback_group,
        )
        self._follow_server = ActionServer(
            self,
            Follow,
            FOLLOW_ACTION,
            execute_callback=self._execute_follow,
            goal_callback=self._on_follow_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._dig_server = ActionServer(
            self,
            ExecuteDig,
            DIG_ACTION,
            execute_callback=lambda handle: self._execute_fixed(handle, "dig", ExecuteDig),
            goal_callback=lambda goal: self._on_fixed_goal(goal, "dig"),
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._dump_server = ActionServer(
            self,
            ExecuteDump,
            DUMP_ACTION,
            execute_callback=lambda handle: self._execute_fixed(handle, "dump", ExecuteDump),
            goal_callback=lambda goal: self._on_fixed_goal(goal, "dump"),
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._jog_server = ActionServer(
            self,
            HoldToJog,
            JOG_ACTION,
            execute_callback=self._execute_jog,
            goal_callback=self._on_jog_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )
        self._status_timer = self.create_timer(
            0.2, self._publish_status, callback_group=self._callback_group
        )
        self._receiver = threading.Thread(target=self._receive_states, daemon=True)
        self._receiver.start()
        self._publish_status()
        self.get_logger().warning(
            "LIVE CONTROL ARMED: one PC Command Sink owns %s -> %s"
            % (self._config.network.state_endpoint, self._config.network.action_endpoint)
        )

    def destroy_node(self):
        self._stopping = True
        self._state_socket.close()
        self._receiver.join(timeout=1.0)
        try:
            sample = self._latest_state
            stamp_ms = self._action_stamp(sample) if sample is not None else now_ms()
            self._command_sink.disarm(action_stamp_ms=stamp_ms)
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                self._send_stop()
                time.sleep(0.05)
            time.sleep(self._config.network.action_valid_ms / 1000.0 + 0.02)
        except (OSError, ActionJournalUnavailable):
            pass
        self._follow_server.destroy()
        self._dig_server.destroy()
        self._dump_server.destroy()
        self._jog_server.destroy()
        self._jog_heartbeat_subscription.destroy()
        self._operator_heartbeat_subscription.destroy()
        self._status_timer.cancel()
        try:
            self._recorded_sender.close()
        finally:
            self._follow_audit.close()
            self._send_socket.close()
        return super().destroy_node()

    def _receive_states(self) -> None:
        expected_host = self._config.network.orin_host
        while not self._stopping:
            try:
                payload, address = self._state_socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if address[0] != expected_host:
                self.get_logger().warning(
                    f"drop state from unexpected host {address[0]}; expected {expected_host}",
                    throttle_duration_sec=2.0,
                )
                continue
            try:
                packet = decode_packet(payload)
            except PacketDecodeError as exc:
                self.get_logger().warning(f"drop invalid state packet: {exc}")
                continue
            if not isinstance(packet, MachineStatePacket):
                continue
            received_pc_ms = now_ms()
            provenance = evaluate_state_provenance(
                packet,
                expected_machine_id=self._machine_profile["machine_id"],
                last_seq=self._last_remote_state_seq,
                last_stamp_ms=self._last_remote_state_stamp_ms,
                received_pc_ms=received_pc_ms,
                expected_clock_offset_ms=self._remote_clock_offset_ms,
            )
            if not provenance.allowed:
                self.get_logger().warning(
                    f"drop Machine State: {provenance.reason}",
                    throttle_duration_sec=2.0,
                )
                continue
            actuator_state = self._evaluate_actuator_state(packet)
            if not actuator_state.allowed:
                self.get_logger().warning(
                    f"Machine State closes motion gate: {actuator_state.reason}",
                    throttle_duration_sec=2.0,
                )
            self._last_remote_state_seq = packet.seq
            self._last_remote_state_stamp_ms = packet.stamp_ms
            if self._remote_clock_offset_ms is None:
                self._remote_clock_offset_ms = packet.stamp_ms - received_pc_ms
            sample = StateSample(
                sequence=0,
                packet=packet,
                received_pc_ms=received_pc_ms,
                received_monotonic_s=time.monotonic(),
            )
            with self._condition:
                self._state_sequence += 1
                sample = replace(sample, sequence=self._state_sequence)
                self._latest_state = sample
                self._states[packet.stamp_ms] = sample
                while len(self._states) > 32:
                    self._states.pop(next(iter(self._states)))
                self._condition.notify_all()
            self._publish_joint_state(packet)

    def _publish_joint_state(self, state: MachineStatePacket) -> None:
        message = JointState()
        set_ros_header_stamp(message.header, state.stamp_ms)
        message.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
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
        self._joint_publisher.publish(message)

    def _on_tip(self, message: PoseStamped) -> None:
        stamp_ms = int(message.header.stamp.sec) * 1000 + int(message.header.stamp.nanosec) // 1_000_000
        if message.header.frame_id != "machine_root_ros" or stamp_ms <= 0:
            return
        with self._condition:
            self._tips[stamp_ms] = message
            while len(self._tips) > 32:
                self._tips.pop(next(iter(self._tips)))
            self._condition.notify_all()

    def _on_jog_heartbeat(self, message: JogHeartbeat) -> None:
        if _SESSION_PATTERN.fullmatch(message.session_id) is None:
            self.get_logger().warning(
                "drop invalid manual-jog heartbeat session_id",
                throttle_duration_sec=2.0,
            )
            return
        with self._condition:
            self._jog_heartbeat_session = message.session_id
            self._jog_heartbeat_monotonic_s = time.monotonic()
            self._condition.notify_all()

    def _on_operator_heartbeat(self, message: OperatorHeartbeat) -> None:
        if message.behavior != "Follow" or _SESSION_PATTERN.fullmatch(message.session_id) is None:
            self.get_logger().warning(
                "drop invalid operator heartbeat",
                throttle_duration_sec=2.0,
            )
            return
        with self._condition:
            self._operator_heartbeat_behavior = message.behavior
            self._operator_heartbeat_session = message.session_id
            self._operator_heartbeat_monotonic_s = time.monotonic()
            self._condition.notify_all()

    def _on_follow_goal(self, request: Follow.Goal) -> GoalResponse:
        try:
            snapshot = _snapshot_from_message(request.trajectory)
            snapshot.validate_for_execution(
                now_s=self._now_s(),
                expected_control_stage=self._control_policy.name,
            )
            _validate_follow_mission(
                snapshot,
                self._mission,
                allowed_target_statuses=self._control_policy.allowed_target_statuses,
            )
        except TrajectoryDigestMismatch as exc:
            return self._reject("TRAJECTORY_PROVENANCE_MISMATCH", str(exc))
        except (TypeError, ValueError) as exc:
            return self._reject("INVALID_TRAJECTORY", str(exc))
        deadline = time.monotonic() + 0.5
        planned_id = None
        with self._condition:
            while snapshot.trajectory_sha256 not in self._planned_trajectories:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            planned_id = self._planned_trajectories.pop(
                snapshot.trajectory_sha256, None
            )
        if planned_id is None:
            return self._reject(
                "TRAJECTORY_NOT_ISSUED_BY_LIVE_PLAN",
                "trajectory digest was not published by the live Plan server",
            )
        if planned_id != snapshot.trajectory_id:
            return self._reject(
                "TRAJECTORY_ID_MISMATCH",
                "trajectory ID does not match the live Plan publication",
            )
        heartbeat_deadline = time.monotonic() + min(
            self._config.follow_control.heartbeat_timeout_ms / 1000.0,
            0.15,
        )
        with self._condition:
            heartbeat_fresh = False
            while not heartbeat_fresh:
                heartbeat_fresh = self._follow_heartbeat_fresh(snapshot.trajectory_id)
                remaining = heartbeat_deadline - time.monotonic()
                if heartbeat_fresh or remaining <= 0.0:
                    break
                self._condition.wait(remaining)
        if not heartbeat_fresh:
            return self._reject(
                "SUPERVISION_HEARTBEAT_TIMEOUT",
                "fresh matching Follow operator heartbeat is required",
            )
        return (
            GoalResponse.ACCEPT
            if self._reserve("Follow", require_mission=False)
            else GoalResponse.REJECT
        )

    def _on_planned_trajectory(self, message: TrajectorySnapshot) -> None:
        try:
            snapshot = _snapshot_from_message(message)
            snapshot.validate_for_execution(
                now_s=self._now_s(),
                expected_control_stage=self._control_policy.name,
            )
            _validate_follow_mission(
                snapshot,
                self._mission,
                allowed_target_statuses=self._control_policy.allowed_target_statuses,
            )
        except (TrajectoryDigestMismatch, TypeError, ValueError) as exc:
            self.get_logger().warning(f"ignore non-executable Plan publication: {exc}")
            return
        with self._condition:
            self._planned_trajectories = {
                **dict(list(self._planned_trajectories.items())[-7:]),
                snapshot.trajectory_sha256: snapshot.trajectory_id,
            }
            self._condition.notify_all()

    def _on_fixed_goal(self, request, expected_phase: str) -> GoalResponse:
        if (
            self._control_policy.name == "production"
            and self._fixed_action_profile.validation_status != "field_validated"
        ):
            return self._reject(
                "FIXED_ACTIONS_NOT_FIELD_VALIDATED",
                "production fixed action profile must be field_validated",
            )
        target = request.target
        expected = self._mission.targets[expected_phase]
        if (
            target.header.frame_id != "machine_root_ros"
            or target.target_kind != expected_phase
            or target.mission_phase != expected_phase
            or not target.target_id
            or target.mission_id != self._mission.mission_id
            or target.mission_sha256 != self._mission.sha256
            or target.target_status not in self._control_policy.allowed_target_statuses
            or self._mission.target_status not in self._control_policy.allowed_target_statuses
            or target.radius_m <= 0.0
            or math.dist(
                (target.position.x, target.position.y, target.position.z),
                expected.position_m,
            ) > 1e-9
            or abs(target.radius_m - expected.radius_m) > 1e-9
        ):
            return self._reject("INVALID_TARGET", f"invalid {expected_phase} target snapshot")
        return GoalResponse.ACCEPT if self._reserve(
            "ExecuteDig" if expected_phase == "dig" else "ExecuteDump"
        ) else GoalResponse.REJECT

    def _on_jog_goal(self, request: HoldToJog.Goal) -> GoalResponse:
        if not self._config.manual_jog.enabled:
            return self._reject("MANUAL_JOG_DISABLED", "manual jog is disabled by runtime config")
        if _SESSION_PATTERN.fullmatch(request.session_id) is None:
            return self._reject("INVALID_JOG_SESSION", "manual jog session_id is invalid")
        if request.actuator not in self._config.manual_jog.allowed_actuators:
            return self._reject("INVALID_JOG_ACTUATOR", "manual jog actuator is not allowed")
        if request.direction not in (-1, 1):
            return self._reject("INVALID_JOG_DIRECTION", "manual jog direction must be -1 or +1")
        heartbeat_deadline = time.monotonic() + min(
            self._config.manual_jog.heartbeat_timeout_ms / 1000.0,
            0.15,
        )
        with self._condition:
            heartbeat_fresh = False
            while not heartbeat_fresh:
                heartbeat_fresh = (
                    self._jog_heartbeat_session == request.session_id
                    and (time.monotonic() - self._jog_heartbeat_monotonic_s) * 1000.0
                    <= self._config.manual_jog.heartbeat_timeout_ms
                )
                remaining = heartbeat_deadline - time.monotonic()
                if heartbeat_fresh or remaining <= 0.0:
                    break
                self._condition.wait(remaining)
        if not heartbeat_fresh:
            return self._reject("JOG_HEARTBEAT_MISSING", "fresh matching jog heartbeat is required")
        sample = self._latest_state
        if sample is not None:
            try:
                jog = build_manual_jog_action(
                    sample.packet,
                    self._machine_profile,
                    actuator=request.actuator,
                    direction=request.direction,
                    allowed_actuators=self._config.manual_jog.allowed_actuators,
                    speed_fraction=self._config.manual_jog.speed_fraction,
                )
            except ValueError as exc:
                return self._reject("INVALID_JOG_CONFIGURATION", str(exc))
            if not jog.allowed:
                return self._reject(jog.reason.upper(), "manual jog input is invalid")
        return GoalResponse.ACCEPT if self._reserve("HoldToJog", require_mission=False) else GoalResponse.REJECT

    def _reserve(self, behavior: str, *, require_mission: bool = True) -> bool:
        with self._lock:
            sample = self._latest_state
            fresh = sample is not None and self._state_age_s(sample) <= self._max_state_age_s
            decision = evaluate_motion_state(sample.packet) if sample is not None else None
            actuator_decision = (
                self._evaluate_actuator_state(sample.packet)
                if sample is not None
                else None
            )
            if self._active_behavior:
                reason, message = "BUSY", f"{self._active_behavior} owns the behavior lease"
            elif not self._recorded_sender.is_healthy:
                reason, message = "ACTION_JOURNAL_UNAVAILABLE", "action journal is unhealthy"
            elif not fresh:
                reason, message = "STALE_MACHINE_STATE", "no fresh live Machine State"
            elif decision is None or not decision.allowed:
                reason = decision.reason.upper() if decision else "MACHINE_STATE_UNAVAILABLE"
                message = "live Machine State safety gate is closed"
            elif actuator_decision is None or not actuator_decision.allowed:
                reason = (
                    actuator_decision.reason.upper()
                    if actuator_decision
                    else "ACTUATOR_STATE_UNAVAILABLE"
                )
                message = "actuator position is outside the machine profile"
            elif (
                require_mission
                and self._mission.target_status
                not in self._control_policy.allowed_target_statuses
            ):
                reason = "MISSION_TARGETS_NOT_FIELD_VALIDATED"
                message = (
                    "Mission target_status is not accepted by "
                    f"{self._control_policy.name} control"
                )
            else:
                self._active_behavior = behavior
                self._last_rejection_reason = ""
                self._last_rejection_message = ""
                return True
            self._last_rejection_reason = reason
            self._last_rejection_message = message
        self._publish_status()
        return False

    def _reject(self, reason: str, message: str) -> GoalResponse:
        with self._lock:
            self._last_rejection_reason = reason
            self._last_rejection_message = message
        self._publish_status()
        return GoalResponse.REJECT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_follow(self, goal_handle) -> Follow.Result:
        snapshot = _snapshot_from_message(goal_handle.request.trajectory)
        session = FollowSession.start(snapshot, accepted_at_s=self._now_s())
        builder = ObservationBuilder(self._machine_profile, task_mode=snapshot.task_mode)
        trajectory = {
            "frame_id": snapshot.frame_id,
            "waypoints_base": [list(point) for point in snapshot.waypoints],
            "waypoint_count": len(snapshot.waypoints),
            "tube_radius": self._machine_profile["observation_schema"]["normalizers"]["tube_radius"],
        }
        previous_action = [0.0] * 4
        last_sequence = self._state_sequence
        pending_physical: list[float] | None = None
        started = time.monotonic()
        last_synchronized_at_s = started
        start_datagrams = self._command_sink.action_datagrams
        latest_distance = -1.0
        try:
            while rclpy.ok(context=self.context):
                supervision = self._follow_supervision_decision(snapshot.trajectory_id)
                if not supervision.allowed:
                    return self._finish_follow(
                        goal_handle,
                        Follow.Result.OUTCOME_FAILED,
                        supervision.reason.upper(),
                        "supervised Follow canary stopped",
                        session,
                        latest_distance,
                        start_datagrams,
                    )
                if goal_handle.is_cancel_requested:
                    return self._finish_follow(goal_handle, Follow.Result.OUTCOME_CANCELLED, "CANCELLED", "Follow cancelled", session, latest_distance, start_datagrams)
                synchronized = self._wait_state_tip_pair(
                    last_sequence, timeout_s=0.05
                )
                if synchronized is None:
                    latest_sample = self._latest_state
                    if (
                        latest_sample is None
                        or self._state_age_s(latest_sample) > self._max_state_age_s
                    ):
                        return self._finish_follow(goal_handle, Follow.Result.OUTCOME_FAILED, "STATE_STALE", "no fresh Machine State", session, latest_distance, start_datagrams)
                    if (
                        time.monotonic() - last_synchronized_at_s
                        >= self._bucket_tip_timeout_s
                    ):
                        return self._finish_follow(
                            goal_handle,
                            Follow.Result.OUTCOME_FAILED,
                            "STATE_TIP_NOT_SYNCHRONIZED",
                            self._state_tip_timeout_message(latest_sample),
                            session,
                            latest_distance,
                            start_datagrams,
                        )
                    supervision = self._follow_supervision_decision(snapshot.trajectory_id)
                    if not supervision.allowed:
                        return self._finish_follow(
                            goal_handle,
                            Follow.Result.OUTCOME_FAILED,
                            supervision.reason.upper(),
                            "supervised Follow canary stopped",
                            session,
                            latest_distance,
                            start_datagrams,
                        )
                    newer_state_waits_for_tip = (
                        latest_sample.sequence > last_sequence
                    )
                    action = (
                        [0.0] * 4
                        if newer_state_waits_for_tip or pending_physical is None
                        else pending_physical
                    )
                    decision = self._send_motion(
                        latest_sample,
                        action,
                        physical_envelope=self._follow_envelope,
                    )
                    if not decision.allowed:
                        return self._finish_follow(goal_handle, Follow.Result.OUTCOME_FAILED, decision.reason.upper(), "motion safety gate closed", session, latest_distance, start_datagrams)
                    continue
                sample, tip_message = synchronized
                last_sequence = sample.sequence
                last_synchronized_at_s = time.monotonic()
                now_s = self._now_s()
                tip_ros = (
                    float(tip_message.pose.position.x),
                    float(tip_message.pose.position.y),
                    float(tip_message.pose.position.z),
                )
                session, update = session.observe(
                    tip_ros,
                    sample_stamp_s=_stamp_s(tip_message),
                    now_s=now_s,
                )
                if not update.sample_accepted:
                    continue
                latest_distance = update.distance_m
                feedback = _feedback(
                    tip_message,
                    update.current_waypoint_index,
                    update.waypoint_count,
                    update.distance_m,
                    update.elapsed_s,
                )
                feedback.tracking_state = (
                    f"live_{self._control_policy.name}/supervised_follow"
                )
                feedback.action_datagrams = self._command_sink.action_datagrams - start_datagrams
                goal_handle.publish_feedback(feedback)
                if update.timed_out:
                    return self._finish_follow(goal_handle, Follow.Result.OUTCOME_FAILED, "TIMEOUT", "Follow tracking timeout", session, latest_distance, start_datagrams)
                if update.completed:
                    return self._finish_follow(goal_handle, Follow.Result.OUTCOME_SUCCEEDED, "SUCCEEDED", "trajectory completed", session, latest_distance, start_datagrams)
                unity_tip = self._adapter.ros_pose_to_unity_bucket_tip(
                    position_m=tip_ros,
                    orientation_xyzw=(
                        tip_message.pose.orientation.x,
                        tip_message.pose.orientation.y,
                        tip_message.pose.orientation.z,
                        tip_message.pose.orientation.w,
                    ),
                    stamp_ms=sample.packet.stamp_ms,
                    swing_joint_rad=sample.packet.joint_position_rad["swing"],
                )
                waypoint_values = build_dynamic_waypoint_values(
                    trajectory,
                    self._machine_profile,
                    bucket_tip_ros=tip_ros,
                    current_index=update.current_waypoint_index,
                )
                observation = builder.build(
                    sample.packet,
                    unity_tip,
                    self._adapter.waypoint_values_to_unity(waypoint_values),
                    previous_action=previous_action,
                    episode_progress=min((time.monotonic() - started) / snapshot.tracking_timeout_s, 1.0),
                )
                raw_normalized = self._policy.run(observation)

                applied_normalized = self._follow_envelope.apply_normalized(raw_normalized)
                physical = physical_velocity_action_from_normalized(
                    applied_normalized, self._machine_profile
                )
                pending_physical = list(physical)
                supervision = self._follow_supervision_decision(snapshot.trajectory_id)
                if not supervision.allowed:
                    return self._finish_follow(
                        goal_handle,
                        Follow.Result.OUTCOME_FAILED,
                        supervision.reason.upper(),
                        "supervised Follow canary stopped",
                        session,
                        latest_distance,
                        start_datagrams,
                    )
                self._record_follow_decision(
                    snapshot.trajectory_id,
                    sample,
                    started,
                    observation,
                    raw_normalized,
                    applied_normalized,
                    physical,
                )
                decision = self._send_motion(
                    sample,
                    pending_physical,
                    physical_envelope=self._follow_envelope,
                )
                if not decision.allowed:
                    return self._finish_follow(goal_handle, Follow.Result.OUTCOME_FAILED, decision.reason.upper(), "motion safety gate closed", session, latest_distance, start_datagrams)
                previous_action = list(applied_normalized)
        except Exception as exc:
            self.get_logger().error(f"live Follow failed: {exc}")
            return self._finish_follow(goal_handle, Follow.Result.OUTCOME_FAILED, "INTERNAL_ERROR", str(exc), session, latest_distance, start_datagrams)

    def _execute_fixed(self, goal_handle, phase: str, action_type):
        start_datagrams = self._command_sink.action_datagrams
        try:
            return self._run_fixed(goal_handle, phase, action_type, start_datagrams)
        except Exception as exc:
            self.get_logger().error(f"live Execute{phase.title()} failed: {exc}")
            return self._finish_fixed(
                goal_handle,
                action_type,
                action_type.Result.OUTCOME_FAILED,
                "INTERNAL_ERROR",
                str(exc),
                start_datagrams,
            )

    def _execute_jog(self, goal_handle) -> HoldToJog.Result:
        start_datagrams = self._command_sink.action_datagrams
        started = time.monotonic()
        initial_position = float("nan")
        final_position = float("nan")
        config = self._config.manual_jog
        try:
            while rclpy.ok(context=self.context):
                elapsed_s = time.monotonic() - started
                if goal_handle.is_cancel_requested:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_CANCELLED,
                        "CANCELLED",
                        "manual jog released",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                if elapsed_s * 1000.0 >= config.max_hold_ms:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_SUCCEEDED,
                        "MAX_HOLD_REACHED",
                        "manual jog maximum hold time reached",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                with self._lock:
                    heartbeat_age_ms = (
                        (time.monotonic() - self._jog_heartbeat_monotonic_s) * 1000.0
                        if self._jog_heartbeat_session == goal_handle.request.session_id
                        else math.inf
                    )
                if heartbeat_age_ms > config.heartbeat_timeout_ms:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_FAILED,
                        "HEARTBEAT_TIMEOUT",
                        "manual jog heartbeat expired",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                sample = self._latest_state
                if sample is None:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_FAILED,
                        "STATE_STALE",
                        "no live Machine State",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                jog = build_manual_jog_action(
                    sample.packet,
                    self._machine_profile,
                    actuator=goal_handle.request.actuator,
                    direction=goal_handle.request.direction,
                    allowed_actuators=config.allowed_actuators,
                    speed_fraction=config.speed_fraction,
                )
                final_position = jog.position_m
                if not math.isfinite(initial_position):
                    initial_position = final_position
                if not jog.allowed:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_FAILED,
                        jog.reason.upper(),
                        "manual jog input became invalid",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                decision = self._send_motion(sample, jog.physical_action)
                if not decision.allowed:
                    return self._finish_jog(
                        goal_handle,
                        HoldToJog.Result.OUTCOME_FAILED,
                        decision.reason.upper(),
                        "motion safety gate closed",
                        initial_position,
                        final_position,
                        start_datagrams,
                    )
                feedback = HoldToJog.Feedback()
                feedback.actuator = goal_handle.request.actuator
                feedback.direction = goal_handle.request.direction
                action_index = self._machine_profile["actuators"][feedback.actuator]["action_index"]
                feedback.commanded_velocity = jog.physical_action[action_index]
                feedback.position_m = final_position
                feedback.elapsed_s = elapsed_s
                feedback.state = "holding"
                feedback.action_datagrams = self._command_sink.action_datagrams - start_datagrams
                goal_handle.publish_feedback(feedback)
                time.sleep(config.command_period_ms / 1000.0)
        except Exception as exc:
            self.get_logger().error(f"manual jog failed: {exc}")
            return self._finish_jog(
                goal_handle,
                HoldToJog.Result.OUTCOME_FAILED,
                "INTERNAL_ERROR",
                str(exc),
                initial_position,
                final_position,
                start_datagrams,
            )

    def _run_fixed(self, goal_handle, phase: str, action_type, start_datagrams):
        target = goal_handle.request.target
        sample = self._latest_state
        if sample is None:
            return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_FAILED, "STALE_MACHINE_STATE", "no Machine State", start_datagrams)
        if self._control_policy.name == "production":
            tip = self._wait_tip(
                sample.packet.stamp_ms,
                timeout_s=self._bucket_tip_timeout_s,
            )
            if tip is None or math.dist(
                (tip.pose.position.x, tip.pose.position.y, tip.pose.position.z),
                (target.position.x, target.position.y, target.position.z),
            ) > target.radius_m:
                return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_FAILED, "NOT_AT_TARGET", "Bucket Tip is outside target radius", start_datagrams)
            start_decision = self._fixed_action_profile.evaluate_start(
                phase,
                sample.packet,
                self._machine_profile,
            )
            if not start_decision.allowed:
                return self._finish_fixed(
                    goal_handle,
                    action_type,
                    action_type.Result.OUTCOME_FAILED,
                    "START_ENVELOPE_REJECTED",
                    start_decision.reason,
                    start_datagrams,
                )
        tuning = self._fixed_action_profile.controller
        executor = FixedActionExecutor(
            self._fixed_action_profile.sequence(phase),
            self._machine_profile,
            kp=tuning.kp,
            min_action=tuning.min_action,
            max_action=tuning.max_action,
            tolerance=tuning.tolerance,
            step_timeout_s=tuning.step_timeout_s,
            hold_s=tuning.hold_s,
        )
        last_sequence = sample.sequence
        last_sample: StateSample | None = sample
        initial_sample: StateSample | None = sample
        pending_physical: list[float] | None = None
        started = time.monotonic()
        while rclpy.ok(context=self.context):
            if goal_handle.is_cancel_requested:
                return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_CANCELLED, "CANCELLED", f"Execute{phase.title()} cancelled", start_datagrams)
            if initial_sample is not None:
                # Use the exact state frame whose matching Bucket Tip and
                # start envelope were accepted.
                sample = initial_sample
                initial_sample = None
            else:
                sample = self._wait_state(last_sequence, timeout_s=0.05)
            if sample is None:
                if last_sample is None or self._state_age_s(last_sample) > self._max_state_age_s:
                    return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_FAILED, "STATE_STALE", "no fresh Machine State", start_datagrams)
                if pending_physical is None:
                    continue
                decision = self._send_motion(last_sample, pending_physical)
                if not decision.allowed:
                    return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_FAILED, decision.reason.upper(), "motion safety gate closed", start_datagrams)
                continue
            last_sequence = sample.sequence
            last_sample = sample
            packet, status = executor.step(
                sample.packet,
                now_s=time.monotonic() - started,
                seq=0,
                valid_for_ms=self._config.network.action_valid_ms,
            )
            pending_physical = list(packet.action)
            decision = self._send_motion(sample, pending_physical)
            if not decision.allowed:
                return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_FAILED, decision.reason.upper(), "motion safety gate closed", start_datagrams)
            feedback = action_type.Feedback()
            feedback.step_index = status.step_index
            feedback.step_label = status.step_label
            feedback.phase = status.phase
            feedback.max_error = status.max_error
            feedback.action_datagrams = self._command_sink.action_datagrams - start_datagrams
            goal_handle.publish_feedback(feedback)
            if status.failed:
                message = (
                    f"fixed action failed at step={status.step_label}, "
                    f"phase={status.phase}, max_error={status.max_error:.6f}"
                )
                self.get_logger().error(message)
                return self._finish_fixed(
                    goal_handle,
                    action_type,
                    action_type.Result.OUTCOME_FAILED,
                    status.reason_code,
                    message,
                    start_datagrams,
                )
            if status.done:
                return self._finish_fixed(goal_handle, action_type, action_type.Result.OUTCOME_SUCCEEDED, "SEQUENCE_COMPLETED", "fixed action sequence completed", start_datagrams)

    def _wait_state(self, after_sequence: int, *, timeout_s: float) -> StateSample | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._stopping:
                if self._latest_state is not None and self._latest_state.sequence > after_sequence:
                    return self._latest_state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
        return None

    def _wait_tip(self, stamp_ms: int, *, timeout_s: float) -> PoseStamped | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._stopping:
                if stamp_ms in self._tips:
                    return self._tips[stamp_ms]
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
        return None

    def _wait_state_tip_pair(
        self, after_sequence: int, *, timeout_s: float
    ) -> tuple[StateSample, PoseStamped] | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._stopping:
                candidates = [
                    (sample, self._tips[sample.packet.stamp_ms])
                    for sample in self._states.values()
                    if sample.sequence > after_sequence
                    and sample.packet.stamp_ms in self._tips
                ]
                if candidates:
                    return max(candidates, key=lambda pair: pair[0].sequence)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
        return None

    def _state_tip_timeout_message(self, state: StateSample) -> str:
        with self._condition:
            latest_tip_stamp_ms = max(self._tips, default=None)
        return (
            "no synchronized Machine State and FK Bucket Tip for "
            f"{self._config.policy.bucket_tip_timeout_ms} ms; "
            f"latest_state_seq={state.packet.seq}, "
            f"latest_state_stamp_ms={state.packet.stamp_ms}, "
            f"latest_tip_stamp_ms={latest_tip_stamp_ms}"
        )

    def _finish_follow(self, goal_handle, outcome, reason, message, session, distance, start_datagrams):
        quiescent = self._stop_and_release()
        if outcome == Follow.Result.OUTCOME_SUCCEEDED:
            goal_handle.succeed()
        elif outcome == Follow.Result.OUTCOME_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result = Follow.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        result.final_waypoint_index = session.tracker.current_index
        result.final_distance_m = distance
        result.quiescence_confirmed = quiescent
        result.action_datagrams = self._command_sink.action_datagrams - start_datagrams
        return result

    def _finish_fixed(self, goal_handle, action_type, outcome, reason, message, start_datagrams):
        quiescent = self._stop_and_release()
        if outcome == action_type.Result.OUTCOME_SUCCEEDED:
            goal_handle.succeed()
        elif outcome == action_type.Result.OUTCOME_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result = action_type.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        result.quiescence_confirmed = quiescent
        result.action_datagrams = self._command_sink.action_datagrams - start_datagrams
        return result

    def _finish_jog(
        self,
        goal_handle,
        outcome,
        reason,
        message,
        initial_position,
        final_position,
        start_datagrams,
    ):
        quiescent = self._stop_and_release()
        if outcome == HoldToJog.Result.OUTCOME_SUCCEEDED:
            goal_handle.succeed()
        elif outcome == HoldToJog.Result.OUTCOME_CANCELLED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result = HoldToJog.Result()
        result.outcome = outcome
        result.reason_code = reason
        result.message = message
        result.initial_position_m = initial_position
        result.final_position_m = final_position
        result.position_delta_m = (
            final_position - initial_position
            if math.isfinite(initial_position) and math.isfinite(final_position)
            else float("nan")
        )
        result.quiescence_confirmed = quiescent
        result.action_datagrams = self._command_sink.action_datagrams - start_datagrams
        return result

    def _stop_and_release(self) -> bool:
        try:
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                self._send_stop()
                time.sleep(0.05)
            time.sleep(self._config.network.action_valid_ms / 1000.0 + 0.02)
            quiescent = True
        except (OSError, ActionJournalUnavailable) as exc:
            self.get_logger().error(f"failed to send terminal zero: {exc}")
            quiescent = False
        with self._lock:
            self._active_behavior = ""
        self._publish_status()
        return quiescent

    def _send_stop(self) -> None:
        sample = self._latest_state
        stamp_ms = self._action_stamp(sample) if sample is not None else now_ms()
        self._command_sink.send_zero(action_stamp_ms=stamp_ms)

    def _send_motion(
        self,
        sample: StateSample,
        physical_action,
        *,
        physical_envelope: FollowCanaryEnvelope | None = None,
    ) -> object:
        actuator_decision = self._evaluate_actuator_state(sample.packet)
        if not actuator_decision.allowed:
            self._command_sink.send_zero(action_stamp_ms=self._action_stamp(sample))
            return actuator_decision
        return self._command_sink.send_velocity(
            sample.packet,
            physical_action,
            action_stamp_ms=self._action_stamp(sample),
            state_age_s=self._state_age_s(sample),
            physical_envelope=physical_envelope,
        )

    def _follow_heartbeat_fresh(self, session_id: str) -> bool:
        return bool(
            self._operator_heartbeat_behavior == "Follow"
            and self._operator_heartbeat_session == session_id
            and (
                time.monotonic() - self._operator_heartbeat_monotonic_s
            ) * 1000.0
            <= self._config.follow_control.heartbeat_timeout_ms
        )

    def _follow_supervision_decision(self, session_id: str):
        with self._lock:
            heartbeat_session = (
                self._operator_heartbeat_session
                if self._operator_heartbeat_behavior == "Follow"
                else ""
            )
            heartbeat_age_ms = (
                (time.monotonic() - self._operator_heartbeat_monotonic_s) * 1000.0
                if self._operator_heartbeat_monotonic_s > 0.0
                else float("inf")
            )
        return evaluate_follow_canary_supervision(
            expected_session=session_id,
            heartbeat_session=heartbeat_session,
            heartbeat_age_ms=heartbeat_age_ms,
            heartbeat_timeout_ms=self._config.follow_control.heartbeat_timeout_ms,
        )

    def _record_follow_decision(
        self,
        trajectory_id: str,
        sample: StateSample,
        started_monotonic_s: float,
        observation,
        raw_normalized,
        applied_normalized,
        physical_action,
    ) -> None:
        if not self._follow_audit.is_healthy:
            raise FollowAuditUnavailable("Follow audit writer is unhealthy")
        record = build_follow_audit_record(
            sequence=self._follow_audit_sequence,
            trajectory_id=trajectory_id,
            state_seq=sample.packet.seq,
            state_stamp_ms=sample.packet.stamp_ms,
            elapsed_ms=(time.monotonic() - started_monotonic_s) * 1000.0,
            observation=observation,
            raw_normalized=raw_normalized,
            applied_normalized=applied_normalized,
            physical_action=physical_action,
        )
        self._follow_audit.submit(record)
        self._follow_audit_sequence += 1

    @staticmethod
    def _state_age_s(sample: StateSample) -> float:
        return max(0.0, time.monotonic() - sample.received_monotonic_s)

    def _action_stamp(self, sample: StateSample) -> int:
        return (
            estimate_remote_now_ms(sample.packet.stamp_ms, sample.received_pc_ms)
            if self._config.network.action_time_source == "orin"
            else now_ms()
        )

    def _evaluate_actuator_state(
        self, state: MachineStatePacket
    ):
        return evaluate_actuator_state(
            state,
            self._machine_profile,
            enforce_bounds=(
                self._control_policy.enforce_actuator_position_bounds
            ),
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        with self._lock:
            sample = self._latest_state
            active = self._active_behavior
            reason = self._last_rejection_reason
            rejection_message = self._last_rejection_message
        fresh = sample is not None and self._state_age_s(sample) <= self._max_state_age_s
        decision = evaluate_motion_state(sample.packet) if sample is not None else None
        actuator_decision = (
            self._evaluate_actuator_state(sample.packet)
            if sample is not None
            else None
        )
        follow_target_ready = (
            self._mission.target_status in self._control_policy.allowed_target_statuses
        )
        journal_ready = self._recorded_sender.is_healthy
        follow_audit_ready = self._follow_audit.is_healthy
        manual_jog_ready = bool(
            self._config.manual_jog.enabled
            and fresh
            and decision is not None
            and decision.allowed
            and actuator_decision is not None
            and actuator_decision.allowed
            and journal_ready
            and not active
        )
        follow_canary_ready = bool(
            follow_target_ready
            and fresh
            and decision is not None
            and decision.allowed
            and actuator_decision is not None
            and actuator_decision.allowed
            and journal_ready
            and follow_audit_ready
            and not active
        )
        status = RuntimeStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "machine_root_ros"
        status.input_source = "live"
        status.execution_mode = "control"
        status.control_stage = self._control_policy.name
        status.motion_backend = "udp_policy"
        status.motion_authorized = True
        status.sender_constructed = True
        status.quiescent = not active
        status.action_datagrams = self._command_sink.action_datagrams
        status.state_fresh = fresh
        status.control_enabled = bool(sample and sample.packet.safety["control_enabled"])
        status.sensor_valid = bool(sample and sample.packet.safety["sensor_valid"])
        status.stm32_alive = bool(sample and sample.packet.safety["stm32_alive"])
        status.estop = bool(sample and sample.packet.safety["estop"])
        status.fault_free = bool(sample and not sample.packet.safety["fault_flags"])
        status.fixed_actions_validated = (
            self._fixed_action_profile.validation_status == "field_validated"
        )
        status.manual_jog_ready = manual_jog_ready
        status.follow_control_mode = self._config.follow_control.mode
        status.follow_speed_fraction = 1.0
        status.follow_allowed_actuators = list(
            self._config.follow_control.allowed_actuators
        )
        status.follow_max_motion_ms = 0
        status.follow_canary_ready = follow_canary_ready
        status.follow_supervision_active = bool(
            active == "Follow"
            and self._follow_heartbeat_fresh(self._operator_heartbeat_session)
        )
        if not fresh:
            status.motion_gate_reason = "state_stale"
        elif decision is None or not decision.allowed:
            status.motion_gate_reason = decision.reason if decision else "state_unavailable"
        elif actuator_decision is None or not actuator_decision.allowed:
            status.motion_gate_reason = (
                actuator_decision.reason if actuator_decision else "actuator_state_unavailable"
            )
        elif not follow_target_ready:
            status.motion_gate_reason = (
                "mission_targets_not_field_validated"
                if self._control_policy.require_field_validated_targets
                else "mission_targets_not_rviz_adjusted"
            )
        elif not journal_ready:
            status.motion_gate_reason = "action_journal_unavailable"
        elif not follow_audit_ready:
            status.motion_gate_reason = "follow_audit_unavailable"
        else:
            status.motion_gate_reason = "ready"
        status.active_behavior = active
        status.last_rejection_reason = reason
        status.last_rejection_message = rejection_message
        self._status_publisher.publish(status)


def _stamp_s(message: PoseStamped) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _physical_action_limits(machine_profile) -> tuple[float, float, float, float]:
    actuators = machine_profile["actuators"]
    return tuple(
        max(
            abs(float(actuators[name]["max_speed_positive"])),
            abs(float(actuators[name]["max_speed_negative"])),
        )
        for name in ("boom", "stick", "bucket", "swing")
    )


def _validate_follow_mission(
    snapshot,
    mission,
    *,
    allowed_target_statuses=frozenset({"field_validated"}),
) -> None:
    if mission.target_status not in allowed_target_statuses:
        allowed = ", ".join(sorted(allowed_target_statuses))
        raise ValueError(f"Mission target_status must be one of: {allowed}")
    if snapshot.mission_id != mission.mission_id or snapshot.mission_sha256 != mission.sha256:
        raise ValueError("trajectory does not match the loaded Mission")
    target = mission.targets.get(snapshot.mission_phase)
    if target is None:
        raise ValueError("trajectory Mission phase is unavailable")
    if math.dist(snapshot.waypoints[-1], target.position_m) > 1e-6:
        raise ValueError("trajectory endpoint does not match the Mission target")
    if snapshot.waypoint_tolerance_m > target.radius_m + 1e-9:
        raise ValueError("trajectory tolerance exceeds the Mission target radius")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Live PC machine behavior Action server")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--motion-authorization", required=True)
    parser.add_argument("--control-stage", choices=CONTROL_STAGES, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = LiveMachineBehaviorNode(
        config_path=args.config,
        mission_path=args.mission,
        motion_authorization=args.motion_authorization,
        control_stage=args.control_stage,
    )
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
