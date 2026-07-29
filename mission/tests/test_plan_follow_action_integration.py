import copy
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from airy_excavator_interfaces.action import Follow, Plan
from airy_excavator_interfaces.msg import RuntimeStatus, TrajectorySnapshot
from airy_excavator_interfaces.snapshot_digest import (
    trajectory_snapshot_message_sha256,
)
from geometry_msgs.msg import Point
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mission.contract import load_mission
from mission.runtime_ros.run_plan_follow_shadow import PlanFollowShadowClient


MISSION_PATH = Path(__file__).resolve().parents[1] / "config/excavation_cycle.json"


class _ActionFixture(Node):
    def __init__(self, *, context, hold_follow=False, shadow_status=True):
        super().__init__("plan_follow_action_fixture", context=context)
        self.hold_follow = hold_follow
        self.follow_cancelled = threading.Event()
        self.release_follow = threading.Event()
        mission = load_mission(MISSION_PATH)
        self.plan_snapshot = self._snapshot(
            mission.sha256,
            "per-goal-result",
            position=mission.targets["dig"].position_m,
        )
        self.preview_snapshot = self._snapshot(
            mission.sha256,
            "unrelated-preview",
            position=(9.0, 0.3, 0.2),
        )
        self.followed = []
        self.plan_requests = 0
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            RuntimeStatus, "/mission/runtime_status", status_qos
        )
        status = RuntimeStatus()
        status.header.frame_id = "machine_root_ros"
        status.header.stamp = self.get_clock().now().to_msg()
        status.input_source = "fixture"
        status.execution_mode = "shadow" if shadow_status else "real"
        status.motion_backend = "none" if shadow_status else "udp"
        status.motion_authorized = not shadow_status
        status.sender_constructed = not shadow_status
        status.quiescent = True
        status.action_datagrams = 0
        self.status_publisher.publish(status)
        self.preview_publisher = self.create_publisher(
            TrajectorySnapshot, "/planning/trajectory_snapshot", 10
        )
        self.plan_server = ActionServer(
            self, Plan, "/planning/plan", execute_callback=self._plan
        )
        self.follow_server = ActionServer(
            self,
            Follow,
            "/excavator/follow",
            execute_callback=self._follow,
            cancel_callback=lambda _request: CancelResponse.ACCEPT,
        )

    def _snapshot(self, mission_sha256, trajectory_id, *, position):
        now = self.get_clock().now()
        snapshot = TrajectorySnapshot()
        snapshot.header.frame_id = "machine_root_ros"
        snapshot.header.stamp = now.to_msg()
        snapshot.trajectory_id = trajectory_id
        snapshot.trajectory_sha256 = "0" * 64
        snapshot.mission_id = "field_cycle_001"
        snapshot.mission_sha256 = mission_sha256
        snapshot.mission_phase = "dig"
        snapshot.task_mode = "MoveToDig"
        snapshot.planning_scope = "preview_global"
        snapshot.control_stage = "none"
        snapshot.workspace_constraint = "none"
        snapshot.execution_eligible = False
        snapshot.source_bucket_tip_stamp = now.to_msg()
        snapshot.source_local_map_stamp = now.to_msg()
        snapshot.inputs_frozen_at = now.to_msg()
        snapshot.valid_until = rclpy.time.Time(
            nanoseconds=now.nanoseconds + 5_000_000_000
        ).to_msg()
        snapshot.input_source = "fixture"
        snapshot.map_source = "fixture_empty"
        snapshot.clock_mode = "ros_clock"
        snapshot.waypoints = [
            Point(x=position[0], y=position[1], z=position[2])
        ]
        snapshot.waypoint_tolerance_m = 0.05
        snapshot.waypoint_dwell_s = 0.3
        snapshot.tracking_timeout_s = 2.0
        snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(snapshot)
        return snapshot

    def _plan(self, goal_handle):
        self.plan_requests += 1
        self.preview_publisher.publish(self.preview_snapshot)
        result = Plan.Result()
        result.outcome = Plan.Result.OUTCOME_SUCCEEDED
        result.reason_code = "SUCCEEDED"
        result.trajectory = copy.deepcopy(self.plan_snapshot)
        result.action_datagrams = 0
        goal_handle.succeed()
        return result

    def _follow(self, goal_handle):
        self.followed.append(copy.deepcopy(goal_handle.request.trajectory))
        result = Follow.Result()
        if self.hold_follow:
            while (
                not goal_handle.is_cancel_requested
                and not self.release_follow.is_set()
            ):
                time.sleep(0.01)
            if goal_handle.is_cancel_requested:
                self.follow_cancelled.set()
                result.outcome = Follow.Result.OUTCOME_CANCELLED
                result.reason_code = "CANCELLED"
                result.quiescence_confirmed = True
                result.action_datagrams = 0
                goal_handle.canceled()
                return result
        result.outcome = Follow.Result.OUTCOME_SUCCEEDED
        result.reason_code = "SUCCEEDED"
        result.quiescence_confirmed = True
        result.action_datagrams = 0
        goal_handle.succeed()
        return result

    def destroy_node(self):
        self.plan_server.destroy()
        self.follow_server.destroy()
        super().destroy_node()


def test_plan_follow_uses_the_per_goal_result_instead_of_the_preview_topic():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    fixture = _ActionFixture(context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = PlanFollowShadowClient(context=context)
    try:
        outcome = client.run_phase(
            mission=load_mission(MISSION_PATH), phase="dig", wait_s=3.0
        )

        assert outcome.plan_result.action_datagrams == 0
        assert outcome.follow_result.action_datagrams == 0
        assert len(fixture.followed) == 1
        assert fixture.followed[0] == fixture.plan_snapshot
        assert fixture.followed[0].trajectory_id == "per-goal-result"
        assert fixture.followed[0].waypoints != fixture.preview_snapshot.waypoints
    finally:
        client.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        fixture.destroy_node()
        rclpy.shutdown(context=context)


def test_follow_result_timeout_cancels_the_accepted_goal():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    fixture = _ActionFixture(context=context, hold_follow=True)
    fixture.plan_snapshot.tracking_timeout_s = 0.05
    fixture.plan_snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(
        fixture.plan_snapshot
    )
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = PlanFollowShadowClient(context=context)
    try:
        with pytest.raises(TimeoutError, match="Follow result"):
            client.run_phase(
                mission=load_mission(MISSION_PATH), phase="dig", wait_s=0.5
            )
        assert fixture.follow_cancelled.wait(timeout=1.0)
        assert len(fixture.followed) == 1
    finally:
        fixture.release_follow.set()
        client.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        fixture.destroy_node()
        rclpy.shutdown(context=context)


def test_non_shadow_runtime_is_rejected_before_planning():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    fixture = _ActionFixture(context=context, shadow_status=False)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = PlanFollowShadowClient(context=context)
    try:
        with pytest.raises(RuntimeError, match="Shadow RuntimeStatus"):
            client.run_phase(
                mission=load_mission(MISSION_PATH), phase="dig", wait_s=0.5
            )
        assert fixture.plan_requests == 0
        assert not fixture.followed
    finally:
        client.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        fixture.destroy_node()
        rclpy.shutdown(context=context)


def test_plan_result_with_the_wrong_mission_target_is_not_followed():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    fixture = _ActionFixture(context=context)
    fixture.plan_snapshot.waypoints[-1].x += 0.5
    fixture.plan_snapshot.trajectory_sha256 = trajectory_snapshot_message_sha256(
        fixture.plan_snapshot
    )
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = PlanFollowShadowClient(context=context)
    try:
        with pytest.raises(ValueError, match="target"):
            client.run_phase(
                mission=load_mission(MISSION_PATH), phase="dig", wait_s=0.5
            )
        assert not fixture.followed
    finally:
        client.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        thread.join(timeout=1.0)
        fixture.destroy_node()
        rclpy.shutdown(context=context)
