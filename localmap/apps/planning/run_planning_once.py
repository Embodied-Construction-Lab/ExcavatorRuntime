#!/usr/bin/env python3
"""从 live LocalMap 和 bucket tip 生成一次可供策略消费的轨迹产物。"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, replace
from pathlib import Path
from collections.abc import Mapping
from types import MappingProxyType


LOCALMAP_DIR = Path(__file__).resolve().parents[2]
AIRY_ROOT = LOCALMAP_DIR.parent
ROS_PYTHON = Path("/usr/bin/python3")
sys.path.insert(0, str(LOCALMAP_DIR))
sys.path.insert(0, str(AIRY_ROOT))

from mission.contract import ExcavationMission, load_mission
from localmap_core.planning_inputs import LivePlanningInputs, load_live_planning_inputs
from localmap_core.planning_intent import PlanningIntent
from localmap_core.planning_profile import (
    DEFAULT_PLANNING_PROFILE,
    PlanningOutputs,
    PlanningProfile,
    load_planning_profile,
)


@dataclass(frozen=True)
class PlanningCommand:
    name: str
    argv: tuple[str, ...]


@dataclass(frozen=True)
class PreparedPlanningRun:
    commands: tuple[PlanningCommand, ...]
    snapshot: LivePlanningInputs
    local_map_snapshot: Path
    bucket_tip_snapshot: Path
    staging_outputs: PlanningOutputs
    final_outputs: PlanningOutputs
    published_artifacts: tuple[str, ...] = (
        "local_map",
        "request",
        "trajectory",
        "observation_slice",
    )


def build_arg_parser() -> argparse.ArgumentParser:
    """第一阶段只允许 Mission 文件驱动的非执行规划。"""
    parser = argparse.ArgumentParser(
        description="从Mission文件和live感知输入生成非执行bucket-tip规划轨迹。"
    )
    parser.add_argument("--mission", type=Path, required=True, help="excavation_mission.v1 JSON")
    parser.add_argument("--phase", choices=("dig", "dump"), required=True)
    parser.add_argument(
        "--planning-scope",
        choices=("preview_global", "workspace_strict"),
        default="preview_global",
        help="均不可执行；默认忽略可达域做全局预览",
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PLANNING_PROFILE, help="Planning Profile JSON")
    parser.add_argument("--dry-run", action="store_true", help="验证输入并打印内部步骤，不执行规划")
    return parser


def build_planning_commands(
    profile: PlanningProfile,
    intent: PlanningIntent,
    *,
    python: Path,
    outputs: PlanningOutputs | None = None,
    planning_scope: str = "execution_strict",
) -> tuple[PlanningCommand, ...]:
    """把一个 Profile 和任务意图编译为四个内部 Adapter 调用。"""
    if planning_scope not in {"execution_strict", "workspace_strict", "preview_global"}:
        raise ValueError(f"未知planning_scope: {planning_scope!r}")
    apps_dir = LOCALMAP_DIR / "apps"
    inputs = profile.inputs
    selected_outputs = outputs or profile.outputs
    obstacle = profile.obstacle_adapter
    planner = profile.planner

    obstacles = (
        str(python),
        str(apps_dir / "perception" / "export_octomap_markers_to_local_map.py"),
        "--topic",
        inputs.octomap_topic,
        "--base-local-map",
        str(inputs.live_local_map),
        "--output",
        str(selected_outputs.local_map),
        "--expected-frame",
        profile.expected_frame,
        "--timeout-s",
        str(profile.freshness.octomap_timeout_s),
        "--box-size",
        str(obstacle.box_size_m),
        "--max-obstacles",
        str(obstacle.max_obstacles),
        "--bounds",
        *(str(value) for value in obstacle.bounds),
    )
    request = (
        str(python),
        str(apps_dir / "planning" / "generate_rrt_request_from_local_map.py"),
        "--local-map",
        str(selected_outputs.local_map),
        "--machine-profile",
        str(inputs.machine_profile),
        "--bucket-tip",
        str(inputs.live_bucket_tip),
        "--target-id",
        intent.target_id,
        "--target-kind",
        intent.target_kind,
        "--task-mode",
        intent.task_mode,
        "--output",
        str(selected_outputs.request),
    )
    trajectory = (
        str(python),
        str(apps_dir / "planning" / "generate_simple_rrt_trajectory_from_request.py"),
        "--request",
        str(selected_outputs.request),
        "--machine-profile",
        str(inputs.machine_profile),
        "--output",
        str(selected_outputs.trajectory),
        "--planning-scope",
        planning_scope,
        "--bounds",
        *(str(value) for value in planner.bounds),
        "--waypoint-count",
        str(planner.waypoint_count),
        "--collision-radius",
        str(planner.collision_radius_m),
        "--step-size",
        str(planner.step_size_m),
        "--edge-check-step",
        str(planner.edge_check_step_m),
        "--max-iterations",
        str(planner.max_iterations),
        "--goal-sample-rate",
        str(planner.goal_sample_rate),
        "--mask-start-radius",
        str(planner.start_mask_radius_m),
        "--mask-goal-radius",
        str(planner.goal_mask_radius_m),
        "--seed",
        str(planner.seed),
    )
    observation = (
        str(python),
        str(apps_dir / "planning" / "generate_observation_waypoint_slice.py"),
        "--trajectory",
        str(selected_outputs.trajectory),
        "--bucket-tip",
        str(inputs.live_bucket_tip),
        "--machine-profile",
        str(inputs.machine_profile),
        "--current-index",
        "0",
        "--output",
        str(selected_outputs.observation_slice),
    )
    disable_execution_workspace = (
        planning_scope == "execution_strict"
        and planner.execution_workspace_mode == "disabled_by_operator"
    )
    if planning_scope == "preview_global":
        workspace_args = ("--disable-reachable-workspace",)
    elif disable_execution_workspace:
        workspace_args = (
            "--disable-reachable-workspace",
            "--workspace-disable-reason",
            "operator_temporary_workspace_invalid",
        )
    else:
        workspace_args = (
            "--reachable-workspace",
            str(inputs.reachable_workspace),
            "--workspace-mode",
            intent.task_mode,
        )
    trajectory = (*trajectory[:8], *workspace_args, *trajectory[8:])
    commands = (
        PlanningCommand("obstacles", obstacles),
        PlanningCommand("request", request),
        PlanningCommand("trajectory", trajectory),
    )
    if planning_scope != "execution_strict":
        return commands
    return (*commands, PlanningCommand("observation", observation))


def _mutable_json(value):
    if isinstance(value, Mapping):
        return {key: _mutable_json(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_mutable_json(item) for item in value]
    return value


def _freeze_json(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze_json(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze_json(item) for item in value)
    return value


def inject_mission_target(
    local_map: Mapping,
    mission: ExcavationMission,
    phase: str,
) -> tuple[Mapping, PlanningIntent]:
    """把 Mission Snapshot 的单个目标注入新的 LocalMap，不修改 live 输入。"""
    if phase not in {"dig", "dump"}:
        raise ValueError(f"未知Mission phase: {phase!r}")
    if local_map.get("frame_id") != mission.frame_id:
        raise ValueError("Mission frame与LocalMap frame不一致")
    snapshot = _mutable_json(local_map)
    for collection_name in ("dig_targets", "dump_targets"):
        if not isinstance(snapshot.get(collection_name), list):
            raise ValueError(f"LocalMap {collection_name}必须是数组")
    identifier = f"{mission.mission_id}:{phase}"
    if any(
        target.get("id") == identifier
        for collection_name in ("dig_targets", "dump_targets")
        for target in snapshot[collection_name]
        if isinstance(target, dict)
    ):
        raise ValueError(f"LocalMap存在重复Mission target: {identifier}")
    target = mission.targets[phase]
    collection_name = "dig_targets" if phase == "dig" else "dump_targets"
    snapshot[collection_name] = [
        *snapshot[collection_name],
        {
            "id": identifier,
            "position_m": list(target.position_m),
            "normal": list(target.normal),
            "radius_m": target.radius_m,
            "confidence": 1.0,
            "mission": {
                "id": mission.mission_id,
                "sha256": mission.sha256,
                "phase": phase,
            },
        },
    ]
    task_mode = "MoveToDig" if phase == "dig" else "CarryMaterial"
    return _freeze_json(snapshot), PlanningIntent(identifier, phase, task_mode)


def outputs_for_scope(profile: PlanningProfile, planning_scope: str) -> PlanningOutputs:
    """隔离非执行预览与严格可达域验证产物。"""
    if planning_scope not in {"preview_global", "workspace_strict"}:
        raise ValueError(f"未知planning_scope: {planning_scope!r}")
    directory_name = "live_preview" if planning_scope == "preview_global" else "live_validation"
    trajectory_name = f"trajectory_command.{planning_scope}.json"
    directory = profile.outputs.directory.parent / directory_name
    return PlanningOutputs(
        directory=directory,
        local_map=directory / "local_map.octomap_obstacles.json",
        request=directory / "rrt_star_request.octomap_obstacles.json",
        trajectory=directory / trajectory_name,
        observation_slice=directory / "observation.disabled.json",
    )


def invalidate_outputs(outputs: PlanningOutputs) -> None:
    """规划前删除同作用域旧产物，失败时RViz不能继续显示旧路径。"""
    for path in (outputs.local_map, outputs.request, outputs.trajectory):
        path.unlink(missing_ok=True)


def prepare_mission_planning_run(
    profile: PlanningProfile,
    *,
    mission_path: Path,
    phase: str,
    planning_scope: str,
    now_s: float,
    python: Path,
    staging_dir: Path,
) -> PreparedPlanningRun:
    """冻结 Mission 与 live 输入，并从实时Bucket Tip规划本阶段路径。"""
    mission = load_mission(mission_path)
    live_inputs = load_live_planning_inputs(profile, now_s=now_s)
    local_map, intent = inject_mission_target(live_inputs.local_map, mission, phase)
    if profile.task_mode_by_target_kind[phase] != intent.task_mode:
        raise ValueError("Mission phase与Planning Profile task mode不一致")
    snapshot = LivePlanningInputs(
        local_map=local_map,
        bucket_tip=live_inputs.bucket_tip,
    )
    return prepare_planning_snapshot(
        profile,
        intent,
        snapshot,
        python=python,
        staging_dir=staging_dir,
        final_outputs=outputs_for_scope(profile, planning_scope),
        planning_scope=planning_scope,
    )


def prepare_orin_edge_planning_run(
    profile: PlanningProfile,
    *,
    mission_path: Path,
    phase: str,
    now_s: float,
    python: Path,
    staging_dir: Path,
) -> PreparedPlanningRun:
    """Freeze live inputs into an execution-strict artifact for Orin."""
    mission = load_mission(mission_path)
    live_inputs = load_live_planning_inputs(profile, now_s=now_s)
    local_map, intent = inject_mission_target(live_inputs.local_map, mission, phase)
    if profile.task_mode_by_target_kind[phase] != intent.task_mode:
        raise ValueError("Mission phase与Planning Profile task mode不一致")
    snapshot = LivePlanningInputs(
        local_map=local_map,
        bucket_tip=live_inputs.bucket_tip,
    )
    return prepare_planning_snapshot(
        profile,
        intent,
        snapshot,
        python=python,
        staging_dir=staging_dir,
        final_outputs=profile.outputs,
        planning_scope="execution_strict",
    )


def prepare_planning_snapshot(
    profile: PlanningProfile,
    intent: PlanningIntent,
    snapshot: LivePlanningInputs,
    *,
    python: Path,
    staging_dir: Path,
    final_outputs: PlanningOutputs | None = None,
    planning_scope: str = "execution_strict",
) -> PreparedPlanningRun:
    """把已验证的不可变输入快照编译为隔离的规划运行。"""
    selected_outputs = final_outputs or profile.outputs
    local_map_snapshot = staging_dir / "local_map.snapshot.json"
    bucket_tip_snapshot = staging_dir / "bucket_tip.snapshot.json"
    staging_outputs = replace(
        selected_outputs,
        directory=staging_dir,
        local_map=staging_dir / selected_outputs.local_map.name,
        request=staging_dir / selected_outputs.request.name,
        trajectory=staging_dir / selected_outputs.trajectory.name,
        observation_slice=staging_dir / selected_outputs.observation_slice.name,
    )
    staged_profile = replace(
        profile,
        inputs=replace(
            profile.inputs,
            live_local_map=local_map_snapshot,
            live_bucket_tip=bucket_tip_snapshot,
        ),
        outputs=staging_outputs,
    )
    published_artifacts = (
        ("local_map", "request", "trajectory")
        if planning_scope != "execution_strict"
        else ("local_map", "request", "trajectory", "observation_slice")
    )
    return PreparedPlanningRun(
        commands=build_planning_commands(
            staged_profile,
            intent,
            python=python,
            planning_scope=planning_scope,
        ),
        snapshot=snapshot,
        local_map_snapshot=local_map_snapshot,
        bucket_tip_snapshot=bucket_tip_snapshot,
        staging_outputs=staging_outputs,
        final_outputs=selected_outputs,
        published_artifacts=published_artifacts,
    )


def execute_planning_commands(
    commands: tuple[PlanningCommand, ...],
    *,
    dry_run: bool,
) -> None:
    """顺序执行内部步骤；dry-run 只打印，不启动子进程。"""
    for command in commands:
        print(f"[{command.name}] {shlex.join(command.argv)}", flush=True)
        if not dry_run:
            subprocess.run(list(command.argv), check=True)


def _thaw_json(value):
    if isinstance(value, Mapping):
        return {key: _thaw_json(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_json(item) for item in value]
    return value


def _write_snapshot(path: Path, value: Mapping) -> None:
    path.write_text(
        json.dumps(_thaw_json(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def execute_staged_run(prepared: PreparedPlanningRun) -> None:
    """Execute and validate every planning step without publishing outputs."""
    prepared.staging_outputs.directory.mkdir(parents=True, exist_ok=True)
    _write_snapshot(prepared.local_map_snapshot, prepared.snapshot.local_map)
    _write_snapshot(prepared.bucket_tip_snapshot, prepared.snapshot.bucket_tip)
    execute_planning_commands(prepared.commands, dry_run=False)

    staging_paths = tuple(
        getattr(prepared.staging_outputs, name)
        for name in prepared.published_artifacts
    )
    for path in staging_paths:
        if not path.is_file():
            raise ValueError(f"planning step 未生成预期产物: {path}")


def publish_prepared_run(prepared: PreparedPlanningRun) -> None:
    """Publish an already validated staged run; callers may add a commit marker."""
    prepared.final_outputs.directory.mkdir(parents=True, exist_ok=True)
    publish_pairs = tuple(
        (
            getattr(prepared.staging_outputs, name),
            getattr(prepared.final_outputs, name),
        )
        for name in prepared.published_artifacts
    )
    for source, destination in publish_pairs:
        os.replace(source, destination)


def execute_prepared_run(prepared: PreparedPlanningRun, *, dry_run: bool) -> None:
    """执行同一快照的规划；步骤全部成功后才逐个替换最终产物。"""
    if dry_run:
        execute_planning_commands(prepared.commands, dry_run=True)
        return
    execute_staged_run(prepared)
    publish_prepared_run(prepared)


def require_ros_python(python: Path = ROS_PYTHON) -> None:
    """确认内部 Adapter 使用的解释器能加载 ROS2 与 NumPy。"""
    if not python.is_file():
        raise ValueError(f"ROS Python 不存在: {python}")
    subprocess.run(
        [str(python), "-c", "import rclpy, numpy"],
        check=True,
        capture_output=True,
        text=True,
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        profile = load_planning_profile(args.profile)
        require_ros_python()
        if not profile.inputs.machine_profile.is_file():
            raise ValueError(f"planning input machine_profile 不存在: {profile.inputs.machine_profile}")
        if (
            args.planning_scope == "workspace_strict"
            and not profile.inputs.reachable_workspace.is_file()
        ):
            raise ValueError(
                f"planning input reachable_workspace 不存在: {profile.inputs.reachable_workspace}"
            )
        final_outputs = outputs_for_scope(profile, args.planning_scope)
        if args.dry_run:
            prepared = prepare_mission_planning_run(
                profile,
                mission_path=args.mission,
                phase=args.phase,
                planning_scope=args.planning_scope,
                now_s=time.time(),
                python=ROS_PYTHON,
                staging_dir=final_outputs.directory / ".planning-dry-run",
            )
            execute_prepared_run(prepared, dry_run=True)
        else:
            final_outputs.directory.mkdir(parents=True, exist_ok=True)
            invalidate_outputs(final_outputs)
            with tempfile.TemporaryDirectory(
                prefix=".planning-",
                dir=final_outputs.directory,
            ) as staging_directory:
                prepared = prepare_mission_planning_run(
                    profile,
                    mission_path=args.mission,
                    phase=args.phase,
                    planning_scope=args.planning_scope,
                    now_s=time.time(),
                    python=ROS_PYTHON,
                    staging_dir=Path(staging_directory),
                )
                execute_prepared_run(prepared, dry_run=False)
        print(
            f"mission planning complete: phase={args.phase}, scope={args.planning_scope}, "
            f"execution_eligible=false, output={final_outputs.directory}"
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"planning failed: {exc}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
