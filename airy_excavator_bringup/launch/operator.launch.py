"""Unified PC operator launch for fixture, live shadow and gated live control."""

from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


@dataclass(frozen=True)
class OperatorProfile:
    name: str
    input_source: str
    execution_mode: str
    motion_backend: str
    control_stage: str
    enable_embedded_joint_tests: bool
    namespace: str
    start_fixture_planner: bool
    start_live_state_bridge: bool
    start_live_perception: bool
    start_live_planner: bool
    start_live_control: bool
    start_orin_edge_gateway: bool


_PROFILES = {
    "fixture_shadow": OperatorProfile(
        name="fixture_shadow",
        input_source="fixture",
        execution_mode="shadow",
        motion_backend="none",
        control_stage="none",
        enable_embedded_joint_tests=True,
        namespace="offline",
        start_fixture_planner=True,
        start_live_state_bridge=False,
        start_live_perception=False,
        start_live_planner=False,
        start_live_control=False,
        start_orin_edge_gateway=False,
    ),
    "live_shadow": OperatorProfile(
        name="live_shadow",
        input_source="live",
        execution_mode="shadow",
        motion_backend="none",
        control_stage="none",
        enable_embedded_joint_tests=False,
        namespace="",
        start_fixture_planner=False,
        start_live_state_bridge=True,
        start_live_perception=True,
        start_live_planner=False,
        start_live_control=False,
        start_orin_edge_gateway=False,
    ),
    "live_commissioning": OperatorProfile(
        name="live_commissioning",
        input_source="live",
        execution_mode="control",
        motion_backend="orin_edge",
        control_stage="commissioning",
        enable_embedded_joint_tests=False,
        namespace="",
        start_fixture_planner=False,
        start_live_state_bridge=True,
        start_live_perception=True,
        start_live_planner=True,
        start_live_control=False,
        start_orin_edge_gateway=True,
    ),
    "live_production": OperatorProfile(
        name="live_production",
        input_source="live",
        execution_mode="control",
        motion_backend="udp_policy",
        control_stage="production",
        enable_embedded_joint_tests=False,
        namespace="",
        start_fixture_planner=False,
        start_live_state_bridge=False,
        start_live_perception=True,
        start_live_planner=True,
        start_live_control=True,
        start_orin_edge_gateway=False,
    ),
}


_OFFLINE_REMAPPINGS = (
    ("/planning/plan", "/offline/planning/plan"),
    ("/planning/trajectory_snapshot", "/offline/planning/trajectory_snapshot"),
    ("/planning/preview_path", "/offline/planning/preview_path"),
    ("/planning/preview_markers", "/offline/planning/preview_markers"),
    ("/excavator/follow", "/offline/excavator/follow"),
    ("/excavator/execute_dig", "/offline/excavator/execute_dig"),
    ("/excavator/execute_dump", "/offline/excavator/execute_dump"),
    ("/excavator/return_home", "/offline/excavator/return_home"),
    ("/mission/run_cycle", "/offline/mission/run_cycle"),
    ("/mission/runtime_status", "/offline/mission/runtime_status"),
    ("/mission/home_pose_catalog", "/offline/mission/home_pose_catalog"),
    ("/mission/dig_target_snapshot", "/offline/mission/dig_target_snapshot"),
    ("/mission/dump_target_snapshot", "/offline/mission/dump_target_snapshot"),
    ("/mission/target_markers", "/offline/mission/target_markers"),
    ("/mission/follow_markers", "/offline/mission/follow_markers"),
    ("/joint_states", "/offline/joint_states"),
    ("/bucket_tip_pose_machine_root_ros", "/offline/bucket_tip_pose_machine_root_ros"),
    ("/robot_description", "/offline/robot_description"),
    ("/tf", "/offline/tf"),
    ("/tf_static", "/offline/tf_static"),
    ("/localmap/machine_root_ros_points", "/offline/localmap/machine_root_ros_points"),
    ("/localmap/planned_bucket_tip_markers", "/offline/localmap/planned_bucket_tip_markers"),
    ("/localmap/reachable_workspace_markers", "/offline/localmap/reachable_workspace_markers"),
    ("/occupied_cells_vis_array", "/offline/occupied_cells_vis_array"),
    ("/initialpose", "/offline/initialpose"),
    ("/goal_pose", "/offline/goal_pose"),
)

_LIVE_ADAPTER_PATHS = (
    Path("runtime_bridge/apps/pc_runtime_bridge.py"),
    Path("runtime_bridge/apps/live_machine_behavior_server.py"),
    Path("runtime_bridge/apps/orin_edge_follow_gateway.py"),
    Path("localmap/apps/perception/run_perception_stack.sh"),
    Path("localmap/localmap_core/runtime_ros/live_plan_action_server.py"),
    Path("localmap/config/planning.json"),
    Path("runtime_bridge/config/runtime.json"),
    Path("mission/config/excavation_cycle.json"),
    Path("mission/config/excavation_demo.json"),
    Path("kinematics/waji_description/urdf/waji.urdf"),
)


def resolve_profile(name: str) -> OperatorProfile:
    try:
        return _PROFILES[name]
    except KeyError as exc:
        supported = ", ".join(sorted(_PROFILES))
        raise ValueError(
            f"unsupported operator profile {name!r}; supported: {supported}"
        ) from exc


def resolve_airy_root(anchors) -> Path:
    """Find the source workspace from install-prefix or symlink-install anchors."""
    for anchor in anchors:
        resolved = Path(anchor).resolve(strict=False)
        start = resolved.parent if resolved.suffix else resolved
        for candidate in (start, *start.parents):
            if all((candidate / relative).is_file() for relative in _LIVE_ADAPTER_PATHS):
                return candidate
    raise RuntimeError(
        "cannot locate AiryLidar source workspace containing live adapters"
    )


def _include_launch(package: str, launch_file: str, arguments=None):
    source = PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / "launch" / launch_file)
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(arguments or {}).items(),
    )


def _required_process(process, reason: str):
    return RegisterEventHandler(
        OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=reason))],
        )
    )


def _live_adapter_processes(
    airy_root: Path,
    profile: OperatorProfile,
    orin_host,
    orin_port,
):
    state_bridge = airy_root / "runtime_bridge" / "apps" / "pc_runtime_bridge.py"
    perception = airy_root / "localmap" / "apps" / "perception" / "run_perception_stack.sh"
    missing = [str(path) for path in (state_bridge, perception) if not path.is_file()]
    if missing:
        raise RuntimeError(f"live shadow adapter is missing: {', '.join(missing)}")
    entities = []
    if profile.start_live_state_bridge:
        state_bridge_process = ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                str(state_bridge),
                "--publish-joint-states",
                "--print-every",
                "100",
            ],
            cwd=str(airy_root),
            output="screen",
        )
        entities.extend(
            [
                state_bridge_process,
                _required_process(state_bridge_process, "required live state bridge exited"),
            ]
        )
    if profile.start_live_perception:
        perception_process = ExecuteProcess(
            cmd=[str(perception)],
            cwd=str(airy_root),
            output="screen",
        )
        entities.extend(
            [
                perception_process,
                _required_process(perception_process, "required live perception exited"),
            ]
        )
    if profile.start_live_planner:
        planner_process = ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                str(airy_root / "localmap/localmap_core/runtime_ros/live_plan_action_server.py"),
                "--profile",
                str(airy_root / "localmap/config/planning.json"),
                "--mission",
                str(airy_root / "mission/config/excavation_cycle.json"),
                "--demo",
                str(airy_root / "mission/config/excavation_demo.json"),
                "--urdf",
                str(airy_root / "kinematics/waji_description/urdf/waji.urdf"),
                "--runtime-config",
                str(airy_root / "runtime_bridge/config/runtime.json"),
                "--control-stage",
                profile.control_stage,
            ],
            cwd=str(airy_root),
            output="screen",
        )
        entities.extend(
            [planner_process, _required_process(planner_process, "required live Plan server exited")]
        )
    if profile.start_live_control:
        runtime_python = airy_root / ".venv_runtime/bin/python"
        if not runtime_python.is_file():
            raise RuntimeError(f"live control Python is missing: {runtime_python}")
        control_process = ExecuteProcess(
            cmd=[
                str(runtime_python),
                str(airy_root / "runtime_bridge/apps/live_machine_behavior_server.py"),
                "--config",
                str(airy_root / "runtime_bridge/config/runtime.json"),
                "--mission",
                str(airy_root / "mission/config/excavation_cycle.json"),
                "--motion-authorization",
                "ALLOW_LIVE_MACHINE_MOTION",
                "--control-stage",
                profile.control_stage,
            ],
            cwd=str(airy_root),
            output="screen",
        )
        entities.extend(
            [control_process, _required_process(control_process, "required live Command Sink exited")]
        )
    if profile.start_orin_edge_gateway:
        gateway_process = ExecuteProcess(
            cmd=[
                "/usr/bin/python3",
                str(airy_root / "runtime_bridge/apps/orin_edge_follow_gateway.py"),
                "--orin-host",
                orin_host,
                "--orin-port",
                orin_port,
                "--motion-authorization",
                "ALLOW_LIVE_MACHINE_MOTION",
                "--control-stage",
                profile.control_stage,
            ],
            cwd=str(airy_root),
            output="screen",
        )
        entities.extend(
            [gateway_process, _required_process(
                gateway_process, "required Orin Edge Follow Gateway exited")]
        )
    entities.append(
        LogInfo(
            msg=(
                f"live_{profile.control_stage}: execution-strict Plan + one authorized UDP Command Sink"
                if profile.start_live_control
                else (
                    f"live_{profile.control_stage}: PC Plan + Orin Edge ONNX Follow"
                    if profile.start_orin_edge_gateway
                    else "live_shadow: live state/FK/perception with NoMotionBackend"
                )
            )
        )
    )
    return entities


def _launch_profile(context):
    try:
        profile = resolve_profile(LaunchConfiguration("profile").perform(context))
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    authorization = LaunchConfiguration("motion_authorization").perform(context)
    motion_profile = profile.start_live_control or profile.start_orin_edge_gateway
    if motion_profile and authorization != "ALLOW_LIVE_MACHINE_MOTION":
        raise RuntimeError(
            f"live_{profile.control_stage} requires motion_authorization:=ALLOW_LIVE_MACHINE_MOTION"
        )
    if not motion_profile and authorization != "LOCKED":
        raise RuntimeError("motion_authorization is only valid for a live motion profile")

    bringup_share = Path(get_package_share_directory("airy_excavator_bringup"))
    rviz_config = bringup_share / "rviz" / "airy_points.rviz"

    entities = []
    if profile.namespace:
        entities.extend(
            [
                PushRosNamespace(profile.namespace),
                *[
                    SetRemap(src=source, dst=destination)
                    for source, destination in _OFFLINE_REMAPPINGS
                ],
            ]
        )
    if (
        profile.start_live_state_bridge
        or profile.start_live_perception
        or profile.start_live_planner
        or profile.start_live_control
        or profile.start_orin_edge_gateway
    ):
        airy_root = resolve_airy_root(
            (
                Path(get_package_prefix("airy_excavator_bringup")),
                Path(__file__),
                Path.cwd(),
            )
        )
        entities.extend(
            _live_adapter_processes(
                airy_root,
                profile,
                LaunchConfiguration("orin_host"),
                LaunchConfiguration("orin_port"),
            )
        )

    entities.append(_include_launch("waji_description", "display.launch.py"))
    if profile.start_fixture_planner:
        entities.append(_include_launch("airy_localmap", "fixture_planning.launch.py"))
    if not profile.start_live_control and not profile.start_orin_edge_gateway:
        entities.append(
            _include_launch(
                "airy_mission_runtime",
                "machine_behaviors_shadow.launch.py",
                arguments={
                    "input_source": profile.input_source,
                    "execution_mode": profile.execution_mode,
                },
            )
        )
    else:
        entities.append(
            Node(
                package="airy_mission_runtime",
                executable="excavation_cycle_server",
                name="excavation_cycle_server",
                output="screen",
            )
        )
    entities.extend(
        [
            Node(
                package="airy_mission_runtime",
                executable="mission_snapshot_publisher",
                name="mission_snapshot_publisher",
                output="screen",
                arguments=[
                    "--mission",
                    str(
                        airy_root / "mission/config/excavation_cycle.json"
                        if (
                            profile.start_live_state_bridge
                            or profile.start_live_perception
                            or profile.start_live_planner
                            or profile.start_live_control
                            or profile.start_orin_edge_gateway
                        )
                        else Path(get_package_share_directory("airy_mission_runtime"))
                        / "config/excavation_cycle.json"
                    ),
                    *(
                        [
                            "--demo",
                            str(airy_root / "mission/config/excavation_demo.json"),
                        ]
                        if (
                            profile.start_live_state_bridge
                            or profile.start_live_perception
                            or profile.start_live_planner
                            or profile.start_live_control
                            or profile.start_orin_edge_gateway
                        )
                        else []
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[
                    {
                        "enable_embedded_joint_tests":
                            profile.enable_embedded_joint_tests
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
    return [GroupAction(entities)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="fixture_shadow",
                description=(
                    "Operator profile: fixture_shadow, live_shadow, "
                    "live_commissioning, or live_production."
                ),
            ),
            DeclareLaunchArgument(
                "motion_authorization",
                default_value="LOCKED",
                description="Exact live-control authorization token; keep LOCKED for shadow profiles.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start the single operator RViz window.",
            ),
            DeclareLaunchArgument(
                "orin_host",
                default_value="192.168.0.55",
                description="Orin Edge behavior server IPv4 address or host name.",
            ),
            DeclareLaunchArgument(
                "orin_port",
                default_value="18083",
                description="Orin Edge behavior RPC TCP port.",
            ),
            OpaqueFunction(function=_launch_profile),
        ]
    )
