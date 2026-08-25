from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def launch_text() -> str:
    return (PACKAGE_ROOT / "launch/operator.launch.py").read_text(encoding="utf-8")


def test_operator_launch_owns_one_rviz_and_reuses_common_shadow_stack():
    text = launch_text()

    assert text.count('package="rviz2"') == 1
    assert '"waji_description", "display.launch.py"' in text
    assert '"airy_localmap", "fixture_planning.launch.py"' in text
    assert '"airy_mission_runtime",' in text
    assert '"machine_behaviors_shadow.launch.py"' in text
    assert 'executable="mission_snapshot_publisher"' in text
    assert '"enable_embedded_joint_tests"' in text


def test_operator_launch_uses_only_the_orin_edge_command_sink():
    text = launch_text().lower()

    for forbidden in (
        "fixed_action_player",
        "motion_sender",
        "enable-motion",
        "reply-zero",
        "live_machine_behavior_server.py",
        "udp_policy",
        "live_production",
    ):
        assert forbidden not in text
    assert text.count("orin_edge_follow_gateway.py") >= 1
    assert "allow_live_machine_motion" in text


def test_orin_edge_gateway_receives_behavior_endpoint_but_no_pc_action_sink():
    text = launch_text()
    gateway_process = text[
        text.index("gateway_process = ExecuteProcess(") :
        text.index("entities.append(\n        LogInfo")
    ]

    assert "orin_edge_follow_gateway.py" in gateway_process
    assert '"--orin-host"' in gateway_process
    assert '"--orin-port"' in gateway_process
    assert '"--motion-authorization"' in gateway_process
    assert "live_machine_behavior_server.py" not in gateway_process


def test_non_motion_live_planner_receives_only_supported_arguments():
    text = launch_text()
    planner_process = text[
        text.index("planner_process = ExecuteProcess(") :
        text.index("entities.extend(\n            [planner_process")
    ]

    assert '"--profile"' in planner_process
    assert '"--mission"' in planner_process
    assert '"--demo"' in planner_process
    assert '"--urdf"' in planner_process
    assert '"--runtime-config"' not in planner_process
    assert '"--motion-authorization"' not in planner_process


def test_fixture_surfaces_are_explicitly_scoped_under_offline_namespace():
    text = launch_text()

    assert 'namespace="offline"' in text
    for surface in (
        "/planning/plan",
        "/excavator/follow",
        "/excavator/return_home",
        "/mission/runtime_status",
        "/joint_states",
        "/bucket_tip_pose_machine_root_ros",
        "/tf",
        "/tf_static",
    ):
        assert f'("{surface}", "/offline{surface}")' in text


def test_live_input_adapter_exit_shuts_down_the_whole_operator_stack():
    text = launch_text()

    assert "OnProcessExit" in text
    assert "_required_process(state_bridge_process" in text
    assert "_required_process(perception_process" in text
    assert "_required_process(planner_process" in text
    assert (
        "_required_process(\n"
        "                    gateway_process, \"required Orin Edge Follow Gateway exited\""
    ) in text


def test_live_shadow_can_publish_orin_authoritative_v3a_trajectory():
    text = launch_text()

    assert '"v3a_trajectory_path"' in text
    assert "publish_trajectory_markers.py" in text
    assert '"--trajectory"' in text
    assert "required V3-A trajectory marker publisher exited" in text
