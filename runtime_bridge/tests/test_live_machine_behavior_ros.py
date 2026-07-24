import hashlib
import json
import socket
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus
from airy_excavator_interfaces.action import ExecuteDig, ExecuteDump, Follow, HoldToJog
from airy_excavator_interfaces.msg import JogHeartbeat, OperatorHeartbeat, TrajectorySnapshot
from airy_excavator_interfaces.snapshot_digest import trajectory_snapshot_message_sha256
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mission.contract import load_mission
from runtime_bridge.apps.live_machine_behavior_server import LiveMachineBehaviorNode
from runtime_bridge.fixed_actions import fixed_action_contract_sha256
from runtime_bridge.protocol import MachineStatePacket, decode_packet, encode_packet, now_ms


AIRY_ROOT = Path(__file__).resolve().parents[2]


def _free_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def _stub_policy(monkeypatch, action=(0.1, -0.1, 0.1, 0.1)):
    class ConstantPolicy:
        def run(self, _observation):
            return list(action)

    monkeypatch.setattr(
        "runtime_bridge.apps.live_machine_behavior_server.OnnxPolicy",
        lambda _model_path: ConstantPolicy(),
    )


def _wait_future(future, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def _write_fixture(
    tmp_path, *, field_actions=False, deploy_observation=False, field_mission=True,
    small_actions=False,
):
    mission_data = json.loads(
        (AIRY_ROOT / "mission/config/excavation_cycle.json").read_text(encoding="utf-8")
    )
    if field_mission:
        mission_data["target_status"] = "field_validated"
    mission_path = tmp_path / "mission.json"
    mission_path.write_text(json.dumps(mission_data), encoding="utf-8")

    state_port = _free_udp_port()
    action_port = _free_udp_port()
    config = json.loads(
        (AIRY_ROOT / "runtime_bridge/config/runtime.json").read_text(encoding="utf-8")
    )
    machine_profile = json.loads(
        (AIRY_ROOT.parent / "shared/machine_profile.json").read_text(encoding="utf-8")
    )
    if not deploy_observation:
        for name in ("boom", "stick", "bucket"):
            machine_profile["actuators"][name].pop("deploy_position_observation", None)
    machine_profile_path = tmp_path / "machine_profile.fixture.json"
    machine_profile_path.write_text(json.dumps(machine_profile), encoding="utf-8")
    machine_profile_sha256 = hashlib.sha256(machine_profile_path.read_bytes()).hexdigest()

    profile = json.loads(
        (AIRY_ROOT / "runtime_bridge/config/fixed_actions.json").read_text(
            encoding="utf-8"
        )
    )
    profile["machine_profile_sha256"] = machine_profile_sha256
    urdf_path = AIRY_ROOT / "kinematics/waji_description/urdf/waji.urdf"
    profile["urdf_sha256"] = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
    configured_onnx = Path(config["artifacts"]["onnx"])
    if not configured_onnx.is_absolute():
        configured_onnx = (AIRY_ROOT / configured_onnx).resolve()
    config["network"].update(
        {
            "state_bind_host": "127.0.0.1",
            "state_port": state_port,
            "orin_host": "127.0.0.1",
            "action_port": action_port,
            "action_time_source": "pc",
        }
    )
    config["artifacts"].update(
        {
            "onnx": str(configured_onnx),
            "machine_profile": str(machine_profile_path),
            "urdf": str(urdf_path),
            "waypoint_slice": str(tmp_path / "unused-waypoints.json"),
            "latest_observation": str(tmp_path / "unused-observation.json"),
        }
    )
    if field_actions:
        profile["profile_id"] = "localhost_field_actions_v1"
        profile["validation_status"] = "field_validated"
        profile["validation_evidence"] = {
            "validated_at": "2026-07-17T12:00:00+08:00",
            "validated_by": "localhost_test",
            "evaluation_report": "EvaluationReport/fixed_action_test.md",
            "evaluation_report_sha256": "0" * 64,
            "experiment_run_ids": ["localhost_fixed_action_001"],
            "validated_phases": ["dig", "dump"],
            "max_validated_normalized_command": 1.0,
        }
    if field_actions or small_actions:
        profile["controller"].update(
            {
                "kp": 20.0,
                "min_action": 0.2,
                "max_action": 1.0,
                "tolerance": 0.002,
                "step_timeout_s": 2.0,
                "hold_s": 0.01,
            }
        )
        profile["actions"] = {
            "dig": [
                {
                    "step_id": "dig_test_boom",
                    "label": "dig_test_boom",
                    "delta_by_actuator": {
                        "boom": 0.02, "stick": 0.0, "bucket": 0.0, "swing": 0.0
                    },
                }
            ],
            "dump": [
                {
                    "step_id": "dump_test_bucket",
                    "label": "dump_test_bucket",
                    "delta_by_actuator": {
                        "boom": 0.0, "stick": 0.0, "bucket": 0.02, "swing": 0.0
                    },
                }
            ],
        }
    if field_actions:
        contract_sha256 = fixed_action_contract_sha256(profile)
        report = tmp_path / "EvaluationReport" / "fixed_action_test.md"
        report.parent.mkdir()
        report.write_text(
            "# Localhost fixed action validation fixture\n\n"
            "fixed_action_profile_id: localhost_field_actions_v1\n"
            f"fixed_action_contract_sha256: {contract_sha256}\n"
            "experiment_run_id: localhost_fixed_action_001\n",
            encoding="utf-8",
        )
        profile["validation_evidence"].update(
            {
                "evaluation_report_sha256": hashlib.sha256(report.read_bytes()).hexdigest(),
                "action_contract_sha256": contract_sha256,
            }
        )
    profile_path = tmp_path / (
        "fixed_actions.field.json" if field_actions else "fixed_actions.candidate.json"
    )
    profile_path.write_text(json.dumps(profile), encoding="utf-8")
    config["artifacts"]["fixed_action_profile"] = str(profile_path)
    config["fixed_action"]["expected_profile_sha256"] = hashlib.sha256(
        profile_path.read_bytes()
    ).hexdigest()
    config["action_journal"]["directory"] = str(tmp_path / "action_journal")
    config_path = tmp_path / "runtime.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    return config_path, mission_path, state_port, action_port


def _state_packet(seq, positions=None):
    stamp = now_ms()
    positions = positions or {"boom": 0.0, "stick": 0.0, "bucket": 0.0, "swing": 0.0}
    return MachineStatePacket(
        seq=seq,
        stamp_ms=stamp,
        safety={
            "estop": False,
            "stm32_alive": True,
            "sensor_valid": True,
            "control_enabled": True,
            "fault_flags": [],
        },
        actuator_state={
            "boom": {"position_m": positions["boom"], "velocity_mps": 0.0},
            "stick": {"position_m": positions["stick"], "velocity_mps": 0.0},
            "bucket": {"position_m": positions["bucket"], "velocity_mps": 0.0},
            "swing": {"position_rad": positions["swing"], "velocity_rad_s": 0.0},
        },
        joint_state={
            "position_rad": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0},
            "velocity_rad_s": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0},
        },
    )


def _follow_goal(node, mission):
    now = node.get_clock().now()
    goal = Follow.Goal()
    snapshot = goal.trajectory
    snapshot.header.frame_id = "machine_root_ros"
    snapshot.header.stamp = now.to_msg()
    snapshot.trajectory_id = "localhost-live-follow"
    snapshot.trajectory_sha256 = "0" * 64
    snapshot.mission_id = mission.mission_id
    snapshot.mission_sha256 = mission.sha256
    snapshot.mission_phase = "dig"
    snapshot.task_mode = "MoveToDig"
    snapshot.planning_scope = "execution_strict"
    snapshot.control_stage = "production"
    snapshot.workspace_constraint = "field_validated"
    snapshot.execution_eligible = True
    snapshot.source_bucket_tip_stamp = now.to_msg()
    snapshot.source_local_map_stamp = now.to_msg()
    snapshot.inputs_frozen_at = now.to_msg()
    snapshot.valid_until = rclpy.time.Time(
        nanoseconds=now.nanoseconds + 5_000_000_000
    ).to_msg()
    snapshot.input_source = "live"
    snapshot.map_source = "live_local_map"
    snapshot.clock_mode = "ros_clock"
    snapshot.waypoints = [Point(x=0.6, y=0.3, z=0.2)]
    snapshot.waypoint_tolerance_m = 0.03
    snapshot.waypoint_dwell_s = 0.05
    snapshot.tracking_timeout_s = 2.0
    snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
    return goal


def _fixed_goal(node, mission, phase, action_type):
    goal = action_type.Goal()
    target = mission.targets[phase]
    goal.target.header.stamp = node.get_clock().now().to_msg()
    goal.target.header.frame_id = "machine_root_ros"
    goal.target.target_id = f"{mission.mission_id}:{phase}"
    goal.target.target_kind = phase
    goal.target.mission_id = mission.mission_id
    goal.target.mission_sha256 = mission.sha256
    goal.target.mission_phase = phase
    goal.target.target_status = mission.target_status
    goal.target.position = Point(
        x=target.position_m[0], y=target.position_m[1], z=target.position_m[2]
    )
    goal.target.normal = Vector3(x=target.normal[0], y=target.normal[1], z=target.normal[2])
    goal.target.radius_m = target.radius_m
    return goal


def _jog_goal(session_id, actuator="boom", direction=1):
    goal = HoldToJog.Goal()
    goal.session_id = session_id
    goal.actuator = actuator
    goal.direction = direction
    return goal


def test_localhost_hold_to_jog_requires_heartbeat_and_release_ends_with_zero(
    tmp_path, monkeypatch
):
    _stub_policy(monkeypatch)
    config_path, mission_path, state_port, action_port = _write_fixture(
        tmp_path, deploy_observation=True, field_mission=False
    )
    action_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_socket.bind(("127.0.0.1", action_port))
    action_socket.settimeout(0.1)
    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()
    received = []
    positions = {"boom": 0.16, "stick": 0.14, "bucket": 0.12, "swing": 0.0}

    def receive_actions():
        while not stop.is_set():
            try:
                payload, _ = action_socket.recvfrom(4096)
            except socket.timeout:
                continue
            received.append(decode_packet(payload))

    def send_states():
        seq = 1
        while not stop.is_set():
            state_socket.sendto(
                encode_packet(_state_packet(seq, positions)), ("127.0.0.1", state_port)
            )
            seq += 1
            time.sleep(0.05)

    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = LiveMachineBehaviorNode(
        control_stage="production",
        config_path=config_path,
        mission_path=mission_path,
        motion_authorization="ALLOW_LIVE_MACHINE_MOTION",
        context=context,
    )
    client_node = rclpy.create_node("localhost_manual_jog_client", context=context)
    heartbeat_publisher = client_node.create_publisher(
        JogHeartbeat, "/excavator/jog_heartbeat", 10
    )
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    receive_thread = threading.Thread(target=receive_actions, daemon=True)
    state_thread = threading.Thread(target=send_states, daemon=True)
    spin_thread.start()
    receive_thread.start()
    state_thread.start()
    client = ActionClient(client_node, HoldToJog, "/excavator/hold_to_jog")
    heartbeat_stop = threading.Event()

    def publish_heartbeats(session_id):
        while not heartbeat_stop.is_set():
            heartbeat = JogHeartbeat()
            heartbeat.header.stamp = client_node.get_clock().now().to_msg()
            heartbeat.session_id = session_id
            heartbeat_publisher.publish(heartbeat)
            time.sleep(0.05)

    try:
        assert client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while server._latest_state is None and time.monotonic() < deadline:
            time.sleep(0.01)

        no_heartbeat = _wait_future(
            client.send_goal_async(_jog_goal("no-heartbeat-001"))
        )
        assert not no_heartbeat.accepted

        session_id = "panel-hold-jog-001"
        heartbeat_thread = threading.Thread(
            target=publish_heartbeats, args=(session_id,), daemon=True
        )
        heartbeat_thread.start()
        time.sleep(0.12)
        start_index = len(received)
        handle = _wait_future(client.send_goal_async(_jog_goal(session_id)))
        assert handle.accepted
        time.sleep(0.12)
        cancel_response = _wait_future(handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        wrapped = _wait_future(handle.get_result_async(), timeout_s=3.0)
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=1.0)

        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert wrapped.result.reason_code == "CANCELLED"
        assert wrapped.result.quiescence_confirmed
        assert wrapped.result.initial_position_m == pytest.approx(positions["boom"])
        assert wrapped.result.final_position_m == pytest.approx(positions["boom"])
        assert wrapped.result.position_delta_m == pytest.approx(0.0)
        jog_packets = received[start_index:]
        assert any(packet.action[0] > 0.0 for packet in jog_packets)
        assert all(
            packet.action[1:] == [0.0, 0.0, 0.0]
            for packet in jog_packets
        )
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]

        timeout_session = "panel-hold-jog-002"
        heartbeat = JogHeartbeat()
        heartbeat.header.stamp = client_node.get_clock().now().to_msg()
        heartbeat.session_id = timeout_session
        for _ in range(3):
            heartbeat_publisher.publish(heartbeat)
            time.sleep(0.05)
        timeout_start = len(received)
        timeout_handle = _wait_future(
            client.send_goal_async(_jog_goal(timeout_session, actuator="stick", direction=-1))
        )
        assert timeout_handle.accepted
        timeout_result = _wait_future(timeout_handle.get_result_async(), timeout_s=3.0)
        assert timeout_result.status == GoalStatus.STATUS_ABORTED
        assert timeout_result.result.reason_code == "HEARTBEAT_TIMEOUT"
        assert timeout_result.result.quiescence_confirmed
        assert any(packet.action[1] < 0.0 for packet in received[timeout_start:])
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]
    finally:
        heartbeat_stop.set()
        stop.set()
        state_thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
        receive_thread.join(timeout=1.0)
        state_socket.close()
        action_socket.close()


def test_localhost_follow_recovers_from_one_missing_tip_and_ends_with_zero(
    tmp_path, monkeypatch
):
    _stub_policy(monkeypatch)
    config_path, mission_path, state_port, action_port = _write_fixture(tmp_path)
    action_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_socket.bind(("127.0.0.1", action_port))
    action_socket.settimeout(0.1)
    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()
    heartbeat_stop = threading.Event()
    pause_states = threading.Event()
    drop_next_tip = threading.Event()
    drop_all_tips = threading.Event()
    dropped_tip = threading.Event()
    received = []
    def receive_actions():
        while not stop.is_set():
            try:
                payload, _ = action_socket.recvfrom(4096)
            except socket.timeout:
                continue
            received.append(decode_packet(payload))

    def send_states():
        seq = 1
        while not stop.is_set():
            if pause_states.is_set():
                time.sleep(0.01)
                continue
            state_socket.sendto(encode_packet(_state_packet(seq)), ("127.0.0.1", state_port))
            seq += 1
            time.sleep(0.05)

    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = LiveMachineBehaviorNode(
        control_stage="production",
        config_path=config_path,
        mission_path=mission_path,
        motion_authorization="ALLOW_LIVE_MACHINE_MOTION",
        context=context,
    )
    client_node = rclpy.create_node("localhost_live_follow_client", context=context)
    tip_publisher = client_node.create_publisher(
        PoseStamped, "/bucket_tip_pose_machine_root_ros", 10
    )
    heartbeat_publisher = client_node.create_publisher(
        OperatorHeartbeat, "/excavator/operator_heartbeat", 10
    )

    def relay_tip(message: JointState):
        if drop_all_tips.is_set():
            return
        if drop_next_tip.is_set():
            drop_next_tip.clear()
            dropped_tip.set()
            return
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = "machine_root_ros"
        pose.pose.position = Point(x=0.6, y=0.3, z=0.2)
        pose.pose.orientation.w = 1.0
        tip_publisher.publish(pose)

    client_node.create_subscription(JointState, "/joint_states", relay_tip, 10)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    receive_thread = threading.Thread(target=receive_actions, daemon=True)
    state_thread = threading.Thread(target=send_states, daemon=True)
    spin_thread.start()
    receive_thread.start()
    state_thread.start()
    client = ActionClient(client_node, Follow, "/excavator/follow")

    def publish_heartbeats(session_id):
        while not heartbeat_stop.is_set():
            heartbeat = OperatorHeartbeat()
            heartbeat.header.stamp = client_node.get_clock().now().to_msg()
            heartbeat.behavior = "Follow"
            heartbeat.session_id = session_id
            heartbeat_publisher.publish(heartbeat)
            time.sleep(0.05)

    try:
        assert client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while server._latest_state is None and time.monotonic() < deadline:
            time.sleep(0.01)
        mission = load_mission(mission_path)
        forged = _follow_goal(client_node, mission)
        forged.trajectory.waypoints[-1].x += 0.1
        forged.trajectory.trajectory_sha256 = trajectory_snapshot_message_sha256(
            forged.trajectory
        )
        rejected = _wait_future(client.send_goal_async(forged))
        assert not rejected.accepted

        valid_goal = _follow_goal(client_node, mission)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        plan_publisher = client_node.create_publisher(
            TrajectorySnapshot, "/planning/trajectory_snapshot", latched
        )
        plan_publisher.publish(valid_goal.trajectory)
        heartbeat_thread = threading.Thread(
            target=publish_heartbeats,
            args=(valid_goal.trajectory.trajectory_id,),
            daemon=True,
        )
        heartbeat_thread.start()
        time.sleep(0.12)
        pause_states.set()
        time.sleep(0.1)
        handle = _wait_future(client.send_goal_async(valid_goal))
        assert handle.accepted
        start_index = len(received)
        drop_next_tip.set()
        pause_states.clear()
        wrapped = _wait_future(handle.get_result_async())
        assert dropped_tip.is_set()
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.reason_code == "SUCCEEDED"
        assert wrapped.result.quiescence_confirmed
        assert wrapped.result.action_datagrams >= 2
        assert received
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]
        goal_packets = received[start_index:]
        first_nonzero = next(
            index for index, packet in enumerate(goal_packets) if any(packet.action)
        )
        assert any(
            packet.action == [0.0, 0.0, 0.0, 0.0]
            for packet in goal_packets[:first_nonzero]
        )
        limits = (0.0351, 0.0444, 0.0419, 0.6)
        assert all(
            abs(value) <= limit + 1e-12
            for packet in received
            for value, limit in zip(packet.action, limits, strict=True)
        )

        pause_states.set()
        time.sleep(0.1)
        plan_publisher.publish(valid_goal.trajectory)
        stalled_handle = _wait_future(client.send_goal_async(valid_goal))
        assert stalled_handle.accepted
        stalled_start_index = len(received)
        drop_all_tips.set()
        stalled_started = time.monotonic()
        pause_states.clear()
        stalled = _wait_future(stalled_handle.get_result_async())
        stalled_elapsed_s = time.monotonic() - stalled_started
        heartbeat_stop.set()
        heartbeat_thread.join(timeout=1.0)
        assert stalled.status == GoalStatus.STATUS_ABORTED
        assert stalled.result.reason_code == "STATE_TIP_NOT_SYNCHRONIZED"
        assert stalled_elapsed_s >= 0.45
        assert received[stalled_start_index:]
        assert all(
            packet.action == [0.0, 0.0, 0.0, 0.0]
            for packet in received[stalled_start_index:]
        )
    finally:
        heartbeat_stop.set()
        pause_states.clear()
        stop.set()
        state_thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
        receive_thread.join(timeout=1.0)
        state_socket.close()
        action_socket.close()


def test_localhost_follow_preserves_policy_output_until_supervision_is_lost(
    tmp_path, monkeypatch
):
    config_path, mission_path, state_port, action_port = _write_fixture(tmp_path)
    action_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_socket.bind(("127.0.0.1", action_port))
    action_socket.settimeout(0.1)
    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()
    heartbeat_stop = threading.Event()
    received = []

    def receive_actions():
        while not stop.is_set():
            try:
                payload, _ = action_socket.recvfrom(4096)
            except socket.timeout:
                continue
            received.append(decode_packet(payload))

    def send_states():
        seq = 1
        while not stop.is_set():
            state_socket.sendto(encode_packet(_state_packet(seq)), ("127.0.0.1", state_port))
            seq += 1
            time.sleep(0.05)

    class RecordingPolicy:
        def __init__(self):
            self.observations = []

        def run(self, observation):
            self.observations.append(list(observation))
            return [1.0, -1.0, 1.0, 1.0]

    policy = RecordingPolicy()
    monkeypatch.setattr(
        "runtime_bridge.apps.live_machine_behavior_server.OnnxPolicy",
        lambda _model_path: policy,
    )
    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = LiveMachineBehaviorNode(
        control_stage="production",
        config_path=config_path,
        mission_path=mission_path,
        motion_authorization="ALLOW_LIVE_MACHINE_MOTION",
        context=context,
    )
    client_node = rclpy.create_node("localhost_live_follow_canary_client", context=context)
    tip_publisher = client_node.create_publisher(
        PoseStamped, "/bucket_tip_pose_machine_root_ros", 10
    )
    heartbeat_publisher = client_node.create_publisher(
        OperatorHeartbeat, "/excavator/operator_heartbeat", 10
    )

    def relay_tip(message: JointState):
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = "machine_root_ros"
        pose.pose.position = Point(x=0.25, y=0.0, z=0.1)
        pose.pose.orientation.w = 1.0
        tip_publisher.publish(pose)

    client_node.create_subscription(JointState, "/joint_states", relay_tip, 10)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    receive_thread = threading.Thread(target=receive_actions, daemon=True)
    state_thread = threading.Thread(target=send_states, daemon=True)
    spin_thread.start()
    receive_thread.start()
    state_thread.start()
    client = ActionClient(client_node, Follow, "/excavator/follow")

    def publish_heartbeats(session_id):
        while not heartbeat_stop.is_set():
            heartbeat = OperatorHeartbeat()
            heartbeat.header.stamp = client_node.get_clock().now().to_msg()
            heartbeat.behavior = "Follow"
            heartbeat.session_id = session_id
            heartbeat_publisher.publish(heartbeat)
            time.sleep(0.05)

    try:
        assert client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while server._latest_state is None and time.monotonic() < deadline:
            time.sleep(0.01)
        mission = load_mission(mission_path)
        goal = _follow_goal(client_node, mission)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        plan_publisher = client_node.create_publisher(
            TrajectorySnapshot, "/planning/trajectory_snapshot", latched
        )
        plan_publisher.publish(goal.trajectory)

        missing_heartbeat = _wait_future(client.send_goal_async(goal))
        assert not missing_heartbeat.accepted

        plan_publisher.publish(goal.trajectory)
        heartbeat_thread = threading.Thread(
            target=publish_heartbeats,
            args=(goal.trajectory.trajectory_id,),
            daemon=True,
        )
        heartbeat_thread.start()
        time.sleep(0.12)
        started = time.monotonic()
        handle = _wait_future(client.send_goal_async(goal))
        assert handle.accepted
        result_future = handle.get_result_async()
        time.sleep(1.2)
        assert not result_future.done(), "Follow must not stop at the former one-second canary limit"
        heartbeat_stop.set()
        wrapped = _wait_future(result_future, timeout_s=4.0)
        elapsed_s = time.monotonic() - started

        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.reason_code == "SUPERVISION_HEARTBEAT_TIMEOUT"
        assert wrapped.result.quiescence_confirmed
        assert elapsed_s >= 1.2
        assert received
        nonzero = [packet for packet in received if any(packet.action)]
        assert nonzero
        limits = (0.0351, 0.0444, 0.0419, 0.6)
        assert all(
            abs(value) <= limit + 1e-12
            for packet in received
            for value, limit in zip(packet.action, limits, strict=True)
        )
        assert len(policy.observations) >= 2
        assert policy.observations[1][30:34] == pytest.approx([1.0, -1.0, 1.0, 1.0])
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]
        latest_audit_path = tmp_path / "unused-observation.json"
        deadline = time.monotonic() + 1.0
        while not latest_audit_path.is_file() and time.monotonic() < deadline:
            time.sleep(0.01)
        audit = json.loads(latest_audit_path.read_text())
        assert audit["trajectory_id"] == goal.trajectory.trajectory_id
        assert audit["raw_normalized"] == [1.0, -1.0, 1.0, 1.0]
        assert audit["applied_normalized"] == [1.0, -1.0, 1.0, 1.0]
        assert len(list((tmp_path / "action_journal/follow_canary").glob("*.jsonl"))) == 1
    finally:
        heartbeat_stop.set()
        stop.set()
        state_thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
        receive_thread.join(timeout=1.0)
        state_socket.close()
        action_socket.close()


def test_commissioning_execute_dig_and_dump_accept_candidate_actions_and_end_with_zero(
    tmp_path, monkeypatch
):
    _stub_policy(monkeypatch)
    config_path, mission_path, state_port, action_port = _write_fixture(
        tmp_path, field_actions=False, field_mission=False, small_actions=True
    )
    action_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    action_socket.bind(("127.0.0.1", action_port))
    action_socket.settimeout(0.1)
    state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()
    lock = threading.Lock()
    positions = {"boom": 0.0, "stick": 0.0, "bucket": 0.0, "swing": 0.0}
    received = []
    first_action_seen = threading.Event()

    def receive_actions():
        while not stop.is_set():
            try:
                payload, _ = action_socket.recvfrom(4096)
            except socket.timeout:
                continue
            packet = decode_packet(payload)
            received.append(packet)
            if any(abs(component) > 0.0 for component in packet.action):
                first_action_seen.set()
            with lock:
                for index, name in enumerate(("boom", "stick", "bucket")):
                    # Orin/STM32 feedback retains the raw sensor direction.
                    # The PC deploy_observation_sign adapter maps it back to the
                    # Unity/training convention used by the fixed-action loop.
                    positions[name] -= packet.action[index] * 0.05

    def send_states():
        seq = 1
        while not stop.is_set():
            with lock:
                snapshot = dict(positions)
            state_socket.sendto(
                encode_packet(_state_packet(seq, snapshot)), ("127.0.0.1", state_port)
            )
            seq += 1
            if seq == 2:
                first_action_seen.wait(timeout=1.0)
            time.sleep(0.05)

    context = rclpy.context.Context()
    rclpy.init(context=context)
    server = LiveMachineBehaviorNode(
        control_stage="commissioning",
        config_path=config_path,
        mission_path=mission_path,
        motion_authorization="ALLOW_LIVE_MACHINE_MOTION",
        workspace_root=tmp_path,
        context=context,
    )
    client_node = rclpy.create_node("localhost_fixed_action_client", context=context)
    mission = load_mission(mission_path)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    receive_thread = threading.Thread(target=receive_actions, daemon=True)
    state_thread = threading.Thread(target=send_states, daemon=True)
    spin_thread.start()
    receive_thread.start()
    state_thread.start()
    dig_client = ActionClient(client_node, ExecuteDig, "/excavator/execute_dig")
    dump_client = ActionClient(client_node, ExecuteDump, "/excavator/execute_dump")
    try:
        assert dig_client.wait_for_server(timeout_sec=2.0)
        assert dump_client.wait_for_server(timeout_sec=2.0)
        deadline = time.monotonic() + 2.0
        while server._latest_state is None and time.monotonic() < deadline:
            time.sleep(0.01)

        dig_start = len(received)
        dig_handle = _wait_future(
            dig_client.send_goal_async(_fixed_goal(client_node, mission, "dig", ExecuteDig))
        )
        assert dig_handle.accepted
        dig_result = _wait_future(dig_handle.get_result_async(), timeout_s=5.0)
        assert dig_result.status == GoalStatus.STATUS_SUCCEEDED
        assert dig_result.result.reason_code == "SEQUENCE_COMPLETED"
        assert dig_result.result.quiescence_confirmed
        assert any(abs(packet.action[0]) > 0.0 for packet in received[dig_start:])
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]

        dump_start = len(received)
        dump_handle = _wait_future(
            dump_client.send_goal_async(
                _fixed_goal(client_node, mission, "dump", ExecuteDump)
            )
        )
        assert dump_handle.accepted
        dump_result = _wait_future(dump_handle.get_result_async(), timeout_s=5.0)
        assert dump_result.status == GoalStatus.STATUS_SUCCEEDED
        assert dump_result.result.reason_code == "SEQUENCE_COMPLETED"
        assert dump_result.result.quiescence_confirmed
        assert any(abs(packet.action[2]) > 0.0 for packet in received[dump_start:])
        assert received[-1].action == [0.0, 0.0, 0.0, 0.0]
    finally:
        stop.set()
        state_thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
        receive_thread.join(timeout=1.0)
        state_socket.close()
        action_socket.close()
