from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory

from mission.runtime_ros import run_plan_follow_live


def test_default_planning_profile_comes_from_installed_localmap_package():
    expected = (
        Path(get_package_share_directory("airy_localmap"))
        / "config"
        / "planning.json"
    )

    assert run_plan_follow_live.DEFAULT_PLANNING_PROFILE == expected
    assert expected.is_file()


def test_live_cli_waits_for_fresh_planning_inputs_before_sending_plan(
    monkeypatch, tmp_path
):
    events = []
    profile = object()

    monkeypatch.setattr(
        run_plan_follow_live,
        "load_planning_profile",
        lambda path: events.append(("load_profile", path)) or profile,
        raising=False,
    )
    monkeypatch.setattr(
        run_plan_follow_live,
        "wait_for_live_planning_inputs",
        lambda actual, *, timeout_s: events.append(
            ("planning_inputs_ready", actual, timeout_s)
        ),
        raising=False,
    )
    monkeypatch.setattr(
        run_plan_follow_live.rclpy,
        "init",
        lambda **_kwargs: events.append("rclpy_init"),
    )
    monkeypatch.setattr(run_plan_follow_live.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        run_plan_follow_live.rclpy,
        "shutdown",
        lambda: events.append("rclpy_shutdown"),
    )

    class FakeClient:
        def __init__(self):
            events.append("client_created")

        def run_phase(self, **_kwargs):
            events.append("plan_sent")
            return SimpleNamespace(
                follow_result=SimpleNamespace(
                    reason_code="SUCCEEDED",
                    quiescence_confirmed=True,
                    action_datagrams=1,
                )
            )

        def destroy_node(self):
            events.append("client_destroyed")

    monkeypatch.setattr(run_plan_follow_live, "PlanFollowLiveClient", FakeClient)
    planning_profile = tmp_path / "planning.json"

    result = run_plan_follow_live.run(
        [
            "dig",
            "--planning-profile",
            str(planning_profile),
            "--wait-s",
            "3",
        ]
    )

    assert result == 0
    assert events[:3] == [
        ("load_profile", planning_profile),
        ("planning_inputs_ready", profile, 3.0),
        "rclpy_init",
    ]
    assert events.index("plan_sent") > events.index("rclpy_init")


def test_live_cli_rejects_stale_inputs_before_creating_ros_client(
    monkeypatch, tmp_path
):
    events = []
    monkeypatch.setattr(
        run_plan_follow_live,
        "load_planning_profile",
        lambda path: events.append(("load_profile", path)) or object(),
    )

    def reject_stale_inputs(_profile, *, timeout_s):
        events.append(("inputs_rejected", timeout_s))
        raise TimeoutError("live planning inputs are stale")

    monkeypatch.setattr(
        run_plan_follow_live,
        "wait_for_live_planning_inputs",
        reject_stale_inputs,
    )
    monkeypatch.setattr(
        run_plan_follow_live.rclpy,
        "init",
        lambda **_kwargs: events.append("rclpy_init"),
    )
    monkeypatch.setattr(run_plan_follow_live.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        run_plan_follow_live,
        "PlanFollowLiveClient",
        lambda: events.append("client_created"),
    )
    planning_profile = tmp_path / "planning.json"

    result = run_plan_follow_live.run(
        [
            "dig",
            "--planning-profile",
            str(planning_profile),
            "--wait-s",
            "3",
        ]
    )

    assert result == 2
    assert events == [
        ("load_profile", planning_profile),
        ("inputs_rejected", 3.0),
    ]
