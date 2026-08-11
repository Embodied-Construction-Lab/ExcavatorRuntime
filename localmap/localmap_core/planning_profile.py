"""一次性 bucket-tip 规划的版本化配置。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

from .perception_profile import load_perception_profile


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLANNING_PROFILE = PROJECT_ROOT / "localmap" / "config" / "planning.json"
PLANNING_PROFILE_SCHEMA = "planning_profile_v2"


class PlanningProfileError(ValueError):
    """Planning Profile 不满足运行契约。"""


@dataclass(frozen=True)
class PlanningInputs:
    perception_profile: Path
    live_local_map: Path
    live_bucket_tip: Path
    octomap_topic: str
    machine_profile: Path
    reachable_workspace: Path


@dataclass(frozen=True)
class PlanningOutputs:
    directory: Path
    local_map: Path
    request: Path
    trajectory: Path
    observation_slice: Path


@dataclass(frozen=True)
class FreshnessSettings:
    local_map_max_age_ms: int
    bucket_tip_max_age_ms: int
    octomap_timeout_s: float


@dataclass(frozen=True)
class ObstacleAdapterSettings:
    bounds: tuple[float, float, float, float, float, float]
    box_size_m: float
    max_obstacles: int


@dataclass(frozen=True)
class PlannerSettings:
    bounds: tuple[float, float, float, float, float, float]
    execution_workspace_mode: str
    collision_radius_m: float
    step_size_m: float
    edge_check_step_m: float
    max_iterations: int
    goal_sample_rate: float
    start_mask_radius_m: float
    goal_mask_radius_m: float
    waypoint_count: int
    seed: int


@dataclass(frozen=True)
class PlanningProfile:
    profile_id: str
    expected_frame: str
    inputs: PlanningInputs
    outputs: PlanningOutputs
    freshness: FreshnessSettings
    obstacle_adapter: ObstacleAdapterSettings
    planner: PlannerSettings
    task_mode_by_target_kind: Mapping[str, str]


def _validate_fields(section: str, data: object, expected: set[str]) -> None:
    if not isinstance(data, dict):
        raise PlanningProfileError(f"{section} 必须是 JSON object")
    missing = expected - set(data)
    if missing:
        raise PlanningProfileError(f"{section} 缺少字段: {', '.join(sorted(missing))}")
    unknown = set(data) - expected
    if unknown:
        raise PlanningProfileError(f"{section} 包含未知字段: {', '.join(sorted(unknown))}")


def _require_int(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise PlanningProfileError(f"{name} 必须是 {minimum}..{maximum} 的整数，实际为 {value!r}")
    return value


def _require_number(name: str, value: object, minimum: float, maximum: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, int | float)
        or not math.isfinite(value)
        or not minimum <= value <= maximum
    ):
        raise PlanningProfileError(f"{name} 必须在 {minimum}..{maximum} 范围内，实际为 {value!r}")
    return float(value)


def _require_string(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PlanningProfileError(f"{name} 必须是非空字符串，实际为 {value!r}")
    return value


def load_planning_profile(
    path: Path = DEFAULT_PLANNING_PROFILE,
    *,
    project_root: Path = PROJECT_ROOT,
) -> PlanningProfile:
    """加载 Planning Profile，相对路径统一按 AiryLidar 根目录解析。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    _validate_fields(
        "root",
        data,
        {
            "schema",
            "profile_id",
            "inputs",
            "freshness",
            "obstacle_adapter",
            "planner",
            "task_mode_by_target_kind",
        },
    )
    if data["schema"] != PLANNING_PROFILE_SCHEMA:
        raise PlanningProfileError(
            f"planning profile schema 必须是 {PLANNING_PROFILE_SCHEMA}，实际为 {data['schema']!r}"
        )
    _validate_fields(
        "inputs",
        data["inputs"],
        {
            "perception_profile",
            "machine_profile",
            "reachable_workspace",
        },
    )
    _validate_fields(
        "freshness",
        data["freshness"],
        {"local_map_max_age_ms", "bucket_tip_max_age_ms", "octomap_timeout_s"},
    )
    _validate_fields(
        "obstacle_adapter",
        data["obstacle_adapter"],
        {"box_size_m", "max_obstacles"},
    )
    _validate_fields(
        "planner",
        data["planner"],
        {
            "execution_workspace_mode",
            "collision_radius_m",
            "step_size_m",
            "edge_check_step_m",
            "max_iterations",
            "goal_sample_rate",
            "start_mask_radius_m",
            "goal_mask_radius_m",
            "waypoint_count",
            "seed",
        },
    )
    _validate_fields(
        "task_mode_by_target_kind",
        data["task_mode_by_target_kind"],
        {"dig", "dump"},
    )
    freshness = data["freshness"]
    obstacle_adapter = data["obstacle_adapter"]
    planner = data["planner"]
    if planner["execution_workspace_mode"] not in {
        "field_validated",
        "disabled_by_operator",
    }:
        raise PlanningProfileError(
            "planner.execution_workspace_mode 必须是 field_validated 或 "
            "disabled_by_operator"
        )
    _require_int("freshness.local_map_max_age_ms", freshness["local_map_max_age_ms"], 1, 60000)
    _require_int("freshness.bucket_tip_max_age_ms", freshness["bucket_tip_max_age_ms"], 1, 60000)
    _require_number("freshness.octomap_timeout_s", freshness["octomap_timeout_s"], 0.1, 60.0)
    _require_number("obstacle_adapter.box_size_m", obstacle_adapter["box_size_m"], 0.001, 10.0)
    _require_int("obstacle_adapter.max_obstacles", obstacle_adapter["max_obstacles"], 0, 1000000)
    _require_number("planner.collision_radius_m", planner["collision_radius_m"], 0.000001, 10.0)
    _require_number("planner.step_size_m", planner["step_size_m"], 0.000001, 10.0)
    _require_number("planner.edge_check_step_m", planner["edge_check_step_m"], 0.000001, 10.0)
    _require_int("planner.max_iterations", planner["max_iterations"], 1, 10000000)
    _require_number("planner.goal_sample_rate", planner["goal_sample_rate"], 0.0, 1.0)
    _require_number("planner.start_mask_radius_m", planner["start_mask_radius_m"], 0.0, 10.0)
    _require_number("planner.goal_mask_radius_m", planner["goal_mask_radius_m"], 0.0, 10.0)
    _require_int("planner.waypoint_count", planner["waypoint_count"], 1, 1000)
    _require_int("planner.seed", planner["seed"], 0, 4294967295)
    _require_string("profile_id", data["profile_id"])
    task_modes = data["task_mode_by_target_kind"]
    expected_task_modes = {"dig": "MoveToDig", "dump": "CarryMaterial"}
    if task_modes != expected_task_modes:
        raise PlanningProfileError(
            "task_mode_by_target_kind 必须是 dig=MoveToDig, dump=CarryMaterial"
        )

    def resolve(name: str, value: object) -> Path:
        _require_string(name, value)
        candidate = Path(value)
        return candidate if candidate.is_absolute() else project_root / candidate

    inputs = data["inputs"]
    perception_profile_path = resolve(
        "inputs.perception_profile",
        inputs["perception_profile"],
    )
    perception = load_perception_profile(
        perception_profile_path,
        project_root=project_root,
    )
    output_dir = perception.outputs.live_local_map.parent
    if perception.outputs.live_bucket_tip.parent != output_dir:
        raise PlanningProfileError(
            "perception live_local_map与live_bucket_tip必须位于同一输出目录"
        )
    planning_outputs = PlanningOutputs(
        directory=output_dir,
        local_map=output_dir / "local_map.octomap_obstacles.json",
        request=output_dir / "rrt_star_request.octomap_obstacles.json",
        trajectory=output_dir / "trajectory_command.simple_rrt.json",
        observation_slice=output_dir / "observation_waypoint_slice.simple_rrt.json",
    )
    live_paths = {
        perception.outputs.live_local_map,
        perception.outputs.live_bucket_tip,
    }
    if len(live_paths) != 2:
        raise PlanningProfileError("perception live输入路径必须互不相同")
    artifact_paths = {
        planning_outputs.local_map,
        planning_outputs.request,
        planning_outputs.trajectory,
        planning_outputs.observation_slice,
    }
    if len(artifact_paths) != 4:
        raise PlanningProfileError("规划产物路径必须互不相同")
    collisions = live_paths & artifact_paths
    if collisions:
        names = ", ".join(str(path) for path in sorted(collisions))
        raise PlanningProfileError(f"live输入与规划产物路径冲突: {names}")
    return PlanningProfile(
        profile_id=data["profile_id"],
        expected_frame=perception.expected_frame,
        inputs=PlanningInputs(
            perception_profile=perception_profile_path,
            live_local_map=perception.outputs.live_local_map,
            live_bucket_tip=perception.outputs.live_bucket_tip,
            octomap_topic=perception.topics.octomap_cells,
            machine_profile=resolve("inputs.machine_profile", inputs["machine_profile"]),
            reachable_workspace=resolve(
                "inputs.reachable_workspace",
                inputs["reachable_workspace"],
            ),
        ),
        outputs=planning_outputs,
        freshness=FreshnessSettings(**freshness),
        obstacle_adapter=ObstacleAdapterSettings(
            bounds=perception.octomap.crop_bounds,
            box_size_m=obstacle_adapter["box_size_m"],
            max_obstacles=obstacle_adapter["max_obstacles"],
        ),
        planner=PlannerSettings(
            bounds=perception.local_map.bounds,
            execution_workspace_mode=planner["execution_workspace_mode"],
            collision_radius_m=planner["collision_radius_m"],
            step_size_m=planner["step_size_m"],
            edge_check_step_m=planner["edge_check_step_m"],
            max_iterations=planner["max_iterations"],
            goal_sample_rate=planner["goal_sample_rate"],
            start_mask_radius_m=planner["start_mask_radius_m"],
            goal_mask_radius_m=planner["goal_mask_radius_m"],
            waypoint_count=planner["waypoint_count"],
            seed=planner["seed"],
        ),
        task_mode_by_target_kind=MappingProxyType(dict(data["task_mode_by_target_kind"])),
    )
