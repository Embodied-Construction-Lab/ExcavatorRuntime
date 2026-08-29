import runpy
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_launch_symbols():
    return runpy.run_path(PACKAGE_ROOT / "launch/operator.launch.py")


def test_profiles_separate_input_provenance_from_motion_permission():
    symbols = load_launch_symbols()
    resolve_profile = symbols["resolve_profile"]

    fixture = resolve_profile("fixture_shadow")
    assert fixture.input_source == "fixture"
    assert fixture.execution_mode == "shadow"
    assert fixture.motion_backend == "none"
    assert fixture.enable_embedded_joint_tests is True

    live = resolve_profile("live_shadow")
    assert live.input_source == "live"
    assert live.execution_mode == "shadow"
    assert live.motion_backend == "none"
    assert live.enable_embedded_joint_tests is False

    commissioning = resolve_profile("live_commissioning")
    assert commissioning.input_source == "live"
    assert commissioning.execution_mode == "control"
    assert commissioning.motion_backend == "orin_edge"
    assert commissioning.control_stage == "commissioning"
    assert commissioning.start_orin_edge_gateway is True

    with pytest.raises(ValueError, match="unsupported operator profile"):
        resolve_profile("live_production")

    with pytest.raises(ValueError, match="unsupported operator profile"):
        resolve_profile("live_motion")


def test_profiles_select_only_their_input_and_planning_adapters():
    resolve_profile = load_launch_symbols()["resolve_profile"]

    fixture = resolve_profile("fixture_shadow")
    assert fixture.namespace == "offline"
    assert fixture.start_fixture_planner is True
    assert fixture.start_live_state_bridge is False
    assert fixture.start_live_perception is False

    live = resolve_profile("live_shadow")
    assert live.namespace == ""
    assert live.start_fixture_planner is False
    assert live.start_live_state_bridge is True
    assert live.start_live_perception is True

    commissioning = resolve_profile("live_commissioning")
    assert commissioning.start_live_state_bridge is True
    assert commissioning.start_live_perception is True
    assert commissioning.start_live_planner is True
    assert commissioning.start_orin_edge_gateway is True


def test_legacy_live_control_profile_is_removed_instead_of_aliased():
    resolve_profile = load_launch_symbols()["resolve_profile"]

    with pytest.raises(ValueError, match="unsupported operator profile"):
        resolve_profile("live_control")


def test_operator_launch_keeps_motion_behind_exact_profile_authorization():
    launch_text = (PACKAGE_ROOT / "launch/operator.launch.py").read_text(
        encoding="utf-8"
    )

    for forbidden in (
        "enable-motion",
        "reply-zero",
        "fixed_action_player",
        "SetEnvironmentVariable",
        "additional_env",
        "os.environ",
    ):
        assert forbidden not in launch_text

    assert "OpaqueFunction" in launch_text
    assert "generate_launch_description" in launch_text
    assert 'authorization != "ALLOW_LIVE_MACHINE_MOTION"' in launch_text
    assert 'default_value="LOCKED"' in launch_text


def test_source_root_resolution_works_from_a_regular_install_prefix(tmp_path):
    resolve_airy_root = load_launch_symbols()["resolve_airy_root"]
    airy_root = tmp_path / "AiryLidar"
    package_prefix = airy_root / "ros2_ws" / "install" / "airy_excavator_bringup"
    (airy_root / "runtime_bridge" / "apps").mkdir(parents=True)
    (airy_root / "localmap" / "apps" / "perception").mkdir(parents=True)
    (airy_root / "runtime_bridge" / "apps" / "pc_runtime_bridge.py").touch()
    (airy_root / "runtime_bridge" / "apps" / "orin_edge_follow_gateway.py").touch()
    (
        airy_root
        / "localmap"
        / "apps"
        / "perception"
        / "run_perception_stack.sh"
    ).touch()
    for relative in (
        "localmap/config/planning.json",
        "localmap/apps/visualization/publish_trajectory_markers.py",
        "mission/config/excavation_cycle.json",
        "mission/config/excavation_dig_point_catalog.v1.json",
        "mission/config/excavation_demo.json",
        "kinematics/waji_description/urdf/waji.urdf",
    ):
        path = airy_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    (airy_root / "localmap" / "localmap_core" / "runtime_ros").mkdir(parents=True)
    (
        airy_root
        / "localmap"
        / "localmap_core"
        / "runtime_ros"
        / "live_plan_action_server.py"
    ).touch()

    assert resolve_airy_root((package_prefix,)) == airy_root
