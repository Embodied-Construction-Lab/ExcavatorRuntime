import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from mission.runtime_ros import run_prepared_plan_follow_live


def _time(seconds: float) -> SimpleNamespace:
    whole = int(seconds)
    nanosec = int(round((seconds - whole) * 1e9))
    return SimpleNamespace(sec=whole, nanosec=nanosec)


def _snapshot(*, created_at_s: float = 100.0, valid_until_s: float = 110.0):
    snapshot = SimpleNamespace(
        trajectory_id="prepared-trajectory-001",
        trajectory_sha256="digest",
        header=SimpleNamespace(
            frame_id="machine_root_ros",
            stamp=_time(created_at_s),
        ),
        mission_id="field_cycle_001",
        mission_sha256="a" * 64,
        mission_phase="dig",
        task_mode="MoveToDig",
        planning_scope="execution_strict",
        control_stage="commissioning",
        workspace_constraint="disabled_by_operator",
        execution_eligible=True,
        source_bucket_tip_stamp=_time(created_at_s - 0.01),
        source_local_map_stamp=_time(created_at_s - 0.02),
        inputs_frozen_at=_time(created_at_s - 0.005),
        valid_until=_time(valid_until_s),
        input_source="live",
        map_source="live_local_map",
        clock_mode="ros_clock",
        waypoints=[SimpleNamespace(x=1.0, y=0.2, z=0.3)],
        waypoint_tolerance_m=0.05,
        waypoint_dwell_s=0.3,
        tracking_timeout_s=2.0,
    )
    return snapshot


def test_prepared_cli_warms_before_plan_gate_then_prepares_before_follow_gate(
    monkeypatch, tmp_path, capsys
):
    events = []
    snapshot = _snapshot()
    plan_gate = tmp_path / "prepared.plan"
    gate = tmp_path / "prepared.start"

    class FakeClient:
        def __init__(self):
            events.append("client_created")

        def plan_phase(self, **kwargs):
            events.append(("plan_phase", kwargs["phase"], kwargs.get("target_id")))
            return SimpleNamespace(reason_code="SUCCEEDED", trajectory=snapshot)

        def require_runtime_ready(self, wait_s, *, expected_input_source=None):
            events.append(("runtime_ready", wait_s, expected_input_source))

        def follow_trajectory(self, trajectory, *, wait_s):
            events.append(("follow_trajectory", trajectory.trajectory_id, wait_s))
            return SimpleNamespace(
                reason_code="SUCCEEDED",
                quiescence_confirmed=True,
                action_datagrams=9,
            )

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=105_000_000_000)
            )

        def destroy_node(self):
            events.append("client_destroyed")

    monkeypatch.setattr(run_prepared_plan_follow_live, "PlanFollowLiveClient", FakeClient)
    monkeypatch.setattr(run_prepared_plan_follow_live, "wait_for_live_planning_inputs", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(run_prepared_plan_follow_live, "load_planning_profile", lambda _path: object())
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_mission",
        lambda _path: SimpleNamespace(
            mission_id="field_cycle_001",
            sha256="a" * 64,
            targets={"dig": SimpleNamespace(position_m=(1.0, 0.2, 0.3), radius_m=0.1)},
        ),
    )
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "init", lambda **_kwargs: events.append("rclpy_init"))
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "ok", lambda: True)
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "shutdown", lambda: events.append("rclpy_shutdown"))
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "wait_for_start_gate",
        lambda actual: events.append(("wait_gate", actual)),
    )
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "validate_prepared_follow_activation",
        lambda **kwargs: events.append(("validate_activation", kwargs["allowed_first_waypoint_distance_m"])),
    )
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_latest_live_bucket_tip",
        lambda _profile, *, now_s: {"position_m": (1.01, 0.2, 0.3), "stamp_s": now_s, "frame_id": "machine_root_ros", "status": "live_from_tf"},
    )
    monkeypatch.setattr(run_prepared_plan_follow_live, "trajectory_snapshot_message_sha256", lambda _snapshot: "digest")

    result = run_prepared_plan_follow_live.run(
        [
            "dig",
            "--mission",
            str(tmp_path / "mission.json"),
            "--planning-profile",
            str(tmp_path / "planning.json"),
            "--start-gate",
            str(gate),
            "--plan-gate",
            str(plan_gate),
            "--first-waypoint-distance-m",
            "0.08",
            "--wait-s",
            "3",
        ]
    )

    assert result == 0
    assert events[:7] == [
        "rclpy_init",
        "client_created",
        ("wait_gate", plan_gate),
        ("plan_phase", "dig", None),
        ("runtime_ready", 3.0, "live"),
        ("wait_gate", gate),
        ("validate_activation", 0.08),
    ]
    assert ("follow_trajectory", "prepared-trajectory-001", 3.0) in events
    output = capsys.readouterr().out
    assert "prepared planner warm:" in output
    assert "prepared follow ready:" in output
    assert "trajectory_id=prepared-trajectory-001" in output


def test_prepared_cli_refreshes_from_live_state_before_follow(
    monkeypatch, tmp_path, capsys
):
    events = []
    first = _snapshot(created_at_s=100.0)
    first.trajectory_id = "prepared-trajectory-old"
    refreshed = _snapshot(created_at_s=104.5)
    refreshed.trajectory_id = "prepared-trajectory-fresh"
    snapshots = iter((first, refreshed))
    plan_gate = tmp_path / "prepared.plan"
    refresh_gate = tmp_path / "prepared.refresh"
    start_gate = tmp_path / "prepared.start"

    class FakeClient:
        def plan_phase(self, **_kwargs):
            snapshot = next(snapshots)
            events.append(("plan_phase", snapshot.trajectory_id))
            return SimpleNamespace(reason_code="SUCCEEDED", trajectory=snapshot)

        def require_runtime_ready(self, *_args, **_kwargs):
            return None

        def follow_trajectory(self, trajectory, *, wait_s):
            events.append(("follow", trajectory.trajectory_id, wait_s))
            return SimpleNamespace(
                reason_code="SUCCEEDED",
                quiescence_confirmed=True,
                action_datagrams=4,
            )

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=105_000_000_000)
            )

        def destroy_node(self):
            return None

    decisions = iter(("refresh", "start"))
    monkeypatch.setattr(run_prepared_plan_follow_live, "PlanFollowLiveClient", FakeClient)
    monkeypatch.setattr(run_prepared_plan_follow_live, "wait_for_live_planning_inputs", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(run_prepared_plan_follow_live, "load_planning_profile", lambda _path: object())
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_mission",
        lambda _path: SimpleNamespace(mission_id="field_cycle_001"),
    )
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "ok", lambda: True)
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(run_prepared_plan_follow_live, "wait_for_start_gate", lambda _path: None)
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "wait_for_prepared_gate",
        lambda **_kwargs: next(decisions),
        raising=False,
    )
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_latest_live_bucket_tip",
        lambda _profile, *, now_s: {
            "position_m": (1.0, 0.2, 0.3),
            "stamp_s": now_s,
        },
    )
    monkeypatch.setattr(run_prepared_plan_follow_live, "trajectory_snapshot_message_sha256", lambda _snapshot: "digest")

    result = run_prepared_plan_follow_live.run(
        [
            "dump",
            "--mission",
            str(tmp_path / "mission.json"),
            "--planning-profile",
            str(tmp_path / "planning.json"),
            "--plan-gate",
            str(plan_gate),
            "--refresh-gate",
            str(refresh_gate),
            "--start-gate",
            str(start_gate),
            "--wait-s",
            "3",
        ]
    )

    assert result == 0
    assert events == [
        ("plan_phase", "prepared-trajectory-old"),
        ("plan_phase", "prepared-trajectory-fresh"),
        ("follow", "prepared-trajectory-fresh", 3.0),
    ]
    output = capsys.readouterr().out
    assert "prepared follow refreshed:" in output
    assert "trajectory_id=prepared-trajectory-fresh" in output
    assert output.count("prepared plan complete:") == 2
    assert "planning_ms=" in output
    assert "prepared follow activation validated:" in output
    assert "first_waypoint_distance_m=0.0000" in output
    assert "frozen_input_age_ms=505.0" in output


def test_prepared_cli_uses_exit_code_3_for_safe_fallback_before_follow(monkeypatch, tmp_path):
    snapshot = _snapshot()
    gate = tmp_path / "prepared.start"

    class FakeClient:
        def plan_phase(self, **_kwargs):
            return SimpleNamespace(reason_code="SUCCEEDED", trajectory=snapshot)

        def require_runtime_ready(self, *_args, **_kwargs):
            return None

        def follow_trajectory(self, *_args, **_kwargs):
            raise AssertionError("follow_trajectory must not be called after safe fallback")

        def get_clock(self):
            return SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=105_000_000_000)
            )

        def destroy_node(self):
            return None

    monkeypatch.setattr(run_prepared_plan_follow_live, "PlanFollowLiveClient", FakeClient)
    monkeypatch.setattr(run_prepared_plan_follow_live, "wait_for_live_planning_inputs", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(run_prepared_plan_follow_live, "load_planning_profile", lambda _path: object())
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_mission",
        lambda _path: SimpleNamespace(
            mission_id="field_cycle_001",
            sha256="a" * 64,
            targets={"dig": SimpleNamespace(position_m=(1.0, 0.2, 0.3), radius_m=0.1)},
        ),
    )
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "ok", lambda: True)
    monkeypatch.setattr(run_prepared_plan_follow_live.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(run_prepared_plan_follow_live, "wait_for_start_gate", lambda _actual: None)
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "validate_prepared_follow_activation",
        lambda **_kwargs: (_ for _ in ()).throw(run_prepared_plan_follow_live.PreparedFollowFallback("snapshot expired")),
    )
    monkeypatch.setattr(
        run_prepared_plan_follow_live,
        "load_latest_live_bucket_tip",
        lambda _profile, *, now_s: {"position_m": (1.01, 0.2, 0.3), "stamp_s": now_s, "frame_id": "machine_root_ros", "status": "live_from_tf"},
    )
    monkeypatch.setattr(run_prepared_plan_follow_live, "trajectory_snapshot_message_sha256", lambda _snapshot: "digest")

    result = run_prepared_plan_follow_live.run(
        [
            "dig",
            "--mission",
            str(tmp_path / "mission.json"),
            "--planning-profile",
            str(tmp_path / "planning.json"),
            "--start-gate",
            str(gate),
            "--first-waypoint-distance-m",
            "0.08",
        ]
    )

    assert result == 3


def test_wait_for_start_gate_requires_absolute_safe_one_shot_file(tmp_path):
    relative = Path("relative.start")
    with pytest.raises(ValueError, match="absolute"):
        run_prepared_plan_follow_live.wait_for_start_gate(relative)

    invalid = tmp_path / "bad gate.start"
    with pytest.raises(ValueError, match="single file path"):
        run_prepared_plan_follow_live.wait_for_start_gate(invalid)


def test_explicit_refresh_gate_wins_if_activation_arrives_at_the_same_time(
    tmp_path,
):
    start = tmp_path / "prepared.start"
    refresh = tmp_path / "prepared.refresh"
    start.touch()
    refresh.touch()

    decision = run_prepared_plan_follow_live.wait_for_prepared_gate(
        start_gate=start,
        refresh_gate=refresh,
    )

    assert decision == "refresh"
    assert start.is_file()
    assert not refresh.exists()


def test_validate_prepared_follow_activation_rejects_first_waypoint_mismatch():
    snapshot = _snapshot()
    snapshot.trajectory_sha256 = hashlib.sha256(b"wrong").hexdigest()
    with pytest.raises(run_prepared_plan_follow_live.PreparedFollowFallback, match="digest"):
        run_prepared_plan_follow_live.validate_prepared_follow_activation(
            snapshot=snapshot,
            runtime_now_s=105.0,
            latest_bucket_tip={"position_m": (1.2, 0.2, 0.3)},
            allowed_first_waypoint_distance_m=0.05,
        )
