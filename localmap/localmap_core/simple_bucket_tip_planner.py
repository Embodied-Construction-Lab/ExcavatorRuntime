"""bucket tip空间的第一版简单避障规划器。

该模块故意不处理关节可达性和自碰撞，只负责在统一坐标系中为ONNX策略生成
bucket tip waypoint。后续可以把同一输入契约替换成关节空间RRT*/OMPL。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .reachable_workspace import ReachableWorkspace


@dataclass(frozen=True)
class PlanningBounds:
    """规划空间边界，顺序为xmin/xmax/ymin/ymax/zmin/zmax。"""

    minimum: np.ndarray
    maximum: np.ndarray

    @classmethod
    def from_values(cls, values: list[float] | tuple[float, ...]) -> "PlanningBounds":
        """从命令行友好的6个浮点数构造边界。"""
        if len(values) != 6:
            raise ValueError("bounds必须包含6个值: xmin xmax ymin ymax zmin zmax")
        x_min, x_max, y_min, y_max, z_min, z_max = [float(value) for value in values]
        minimum = np.array([x_min, y_min, z_min], dtype=np.float64)
        maximum = np.array([x_max, y_max, z_max], dtype=np.float64)
        if np.any(maximum <= minimum):
            raise ValueError("bounds最大值必须大于最小值")
        return cls(minimum=minimum, maximum=maximum)

    def contains(self, point: np.ndarray) -> bool:
        """检查点是否在规划空间内。"""
        return bool(np.all(point >= self.minimum) and np.all(point <= self.maximum))

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        """在规划空间内均匀采样一个点。"""
        return rng.uniform(self.minimum, self.maximum)


@dataclass(frozen=True)
class PlannedPath:
    """规划结果；失败时waypoints为空并给出reason。"""

    success: bool
    waypoints: np.ndarray
    reason: str
    iterations: int


@dataclass(frozen=True)
class CompiledObstacles:
    """向量化后的障碍物集合；第一版仅支持轴对齐box/sphere/cylinder。"""

    box_centers: np.ndarray
    box_half_sizes: np.ndarray
    sphere_centers: np.ndarray
    sphere_radii: np.ndarray
    cylinder_centers: np.ndarray
    cylinder_radii: np.ndarray
    cylinder_half_heights: np.ndarray
    up_axis_index: int


def compile_obstacles(obstacles: list[dict[str, Any]], collision_radius_m: float, up_axis: str = "y") -> CompiledObstacles:
    """把LocalMap obstacles编译成numpy数组，便于RRT边检查时快速查询碰撞。"""
    axis_index = {"x": 0, "y": 1, "z": 2}[up_axis]
    box_centers: list[list[float]] = []
    box_half_sizes: list[list[float]] = []
    sphere_centers: list[list[float]] = []
    sphere_radii: list[float] = []
    cylinder_centers: list[list[float]] = []
    cylinder_radii: list[float] = []
    cylinder_half_heights: list[float] = []

    for obstacle in obstacles:
        shape = obstacle.get("shape")
        center = [float(value) for value in obstacle["center_m"]]
        if shape == "box":
            size = np.array(obstacle["size_m"], dtype=np.float64)
            box_centers.append(center)
            # 关键：把bucket tip/铲斗简化成小球，等价于膨胀障碍物半径。
            box_half_sizes.append((size * 0.5 + collision_radius_m).tolist())
        elif shape == "sphere":
            sphere_centers.append(center)
            sphere_radii.append(float(obstacle["radius_m"]) + collision_radius_m)
        elif shape == "cylinder":
            cylinder_centers.append(center)
            cylinder_radii.append(float(obstacle["radius_m"]) + collision_radius_m)
            cylinder_half_heights.append(float(obstacle["height_m"]) * 0.5 + collision_radius_m)

    return CompiledObstacles(
        box_centers=np.asarray(box_centers, dtype=np.float64).reshape(-1, 3),
        box_half_sizes=np.asarray(box_half_sizes, dtype=np.float64).reshape(-1, 3),
        sphere_centers=np.asarray(sphere_centers, dtype=np.float64).reshape(-1, 3),
        sphere_radii=np.asarray(sphere_radii, dtype=np.float64),
        cylinder_centers=np.asarray(cylinder_centers, dtype=np.float64).reshape(-1, 3),
        cylinder_radii=np.asarray(cylinder_radii, dtype=np.float64),
        cylinder_half_heights=np.asarray(cylinder_half_heights, dtype=np.float64),
        up_axis_index=axis_index,
    )


def point_in_collision(point: np.ndarray, obstacles: CompiledObstacles) -> bool:
    """判断一个bucket tip点是否与任一障碍物发生碰撞。"""
    if obstacles.box_centers.size:
        delta = np.abs(obstacles.box_centers - point.reshape(1, 3))
        if bool(np.any(np.all(delta <= obstacles.box_half_sizes, axis=1))):
            return True

    if obstacles.sphere_centers.size:
        distances = np.linalg.norm(obstacles.sphere_centers - point.reshape(1, 3), axis=1)
        if bool(np.any(distances <= obstacles.sphere_radii)):
            return True

    if obstacles.cylinder_centers.size:
        up = obstacles.up_axis_index
        horizontal_axes = [axis for axis in range(3) if axis != up]
        horizontal_delta = obstacles.cylinder_centers[:, horizontal_axes] - point[horizontal_axes].reshape(1, 2)
        horizontal_distance = np.linalg.norm(horizontal_delta, axis=1)
        vertical_distance = np.abs(obstacles.cylinder_centers[:, up] - point[up])
        if bool(np.any((horizontal_distance <= obstacles.cylinder_radii) & (vertical_distance <= obstacles.cylinder_half_heights))):
            return True

    return False


def segment_in_collision(start: np.ndarray, end: np.ndarray, obstacles: CompiledObstacles, edge_check_step_m: float) -> bool:
    """沿线段采样检查碰撞；第一版用离散采样换取实现简单可审计。"""
    length = float(np.linalg.norm(end - start))
    sample_count = max(2, int(np.ceil(length / max(edge_check_step_m, 1e-4))) + 1)
    for ratio in np.linspace(0.0, 1.0, sample_count):
        point = start * (1.0 - ratio) + end * ratio
        if point_in_collision(point, obstacles):
            return True
    return False


def infer_bounds(start: np.ndarray, goal: np.ndarray, obstacles: list[dict[str, Any]], margin_m: float = 0.5) -> PlanningBounds:
    """从起点、目标和障碍物推断一个保守规划边界。"""
    points = [start, goal]
    for obstacle in obstacles:
        points.append(np.asarray(obstacle["center_m"], dtype=np.float64))
    stacked = np.vstack(points)
    return PlanningBounds(minimum=np.min(stacked, axis=0) - margin_m, maximum=np.max(stacked, axis=0) + margin_m)


def steer(from_point: np.ndarray, to_point: np.ndarray, step_size_m: float) -> np.ndarray:
    """从树节点朝采样点前进一步。"""
    delta = to_point - from_point
    distance = float(np.linalg.norm(delta))
    if distance <= step_size_m:
        return to_point.copy()
    return from_point + delta / distance * step_size_m


def reconstruct_path(nodes: list[np.ndarray], parents: list[int], node_index: int) -> np.ndarray:
    """从父指针回溯得到start->goal路径。"""
    reversed_points = []
    while node_index >= 0:
        reversed_points.append(nodes[node_index])
        node_index = parents[node_index]
    return np.asarray(list(reversed(reversed_points)), dtype=np.float64)


def shortcut_path(
    path: np.ndarray,
    obstacles: CompiledObstacles,
    edge_check_step_m: float,
    reachable_workspace: ReachableWorkspace | None = None,
    passes: int = 2,
) -> np.ndarray:
    """贪心删除不必要的中间点，让输出waypoints更短。"""
    if path.shape[0] <= 2:
        return path
    simplified = path
    for _ in range(passes):
        kept = [simplified[0]]
        current = 0
        while current < simplified.shape[0] - 1:
            next_index = simplified.shape[0] - 1
            while next_index > current + 1:
                # 关键：shortcut不能为了变短而穿出bucket tip可达区域。
                edge_reachable = reachable_workspace is None or reachable_workspace.segment_inside(
                    simplified[current],
                    simplified[next_index],
                    step_m=edge_check_step_m,
                )
                if edge_reachable and not segment_in_collision(simplified[current], simplified[next_index], obstacles, edge_check_step_m):
                    break
                next_index -= 1
            kept.append(simplified[next_index])
            current = next_index
        simplified = np.asarray(kept, dtype=np.float64)
    return simplified


def resample_path(path: np.ndarray, waypoint_count: int | None) -> np.ndarray:
    """按路径弧长重采样到固定waypoint数量；None表示保留原始节点。"""
    if waypoint_count is None or waypoint_count <= 0 or path.shape[0] <= 1:
        return path
    if path.shape[0] == waypoint_count:
        return path

    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 1e-9:
        return np.repeat(path[:1], waypoint_count, axis=0)

    samples = np.linspace(0.0, total, waypoint_count)
    output = []
    for distance in samples:
        index = int(np.searchsorted(cumulative, distance, side="right") - 1)
        index = min(index, path.shape[0] - 2)
        local_length = cumulative[index + 1] - cumulative[index]
        ratio = 0.0 if local_length <= 1e-9 else (distance - cumulative[index]) / local_length
        output.append(path[index] * (1.0 - ratio) + path[index + 1] * ratio)
    return np.asarray(output, dtype=np.float64)


def plan_bucket_tip_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles: list[dict[str, Any]],
    bounds: PlanningBounds | None = None,
    collision_radius_m: float = 0.08,
    step_size_m: float = 0.20,
    edge_check_step_m: float = 0.04,
    max_iterations: int = 3000,
    goal_sample_rate: float = 0.15,
    waypoint_count: int | None = None,
    seed: int = 0,
    reachable_workspace: ReachableWorkspace | None = None,
) -> PlannedPath:
    """在bucket tip 3D空间规划一条避障路径。

    这是一版工程打通用的RRT式规划器：输出可被现有38维observation链路消费的
    bucket tip waypoints，不声明关节空间最优性。
    """
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    planning_bounds = bounds or infer_bounds(start, goal, obstacles)
    compiled = compile_obstacles(obstacles, collision_radius_m=collision_radius_m)

    if not planning_bounds.contains(start):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "start_out_of_bounds", 0)
    if not planning_bounds.contains(goal):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "goal_out_of_bounds", 0)
    if reachable_workspace is not None and not reachable_workspace.contains(start):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "start_out_of_reachable_workspace", 0)
    if reachable_workspace is not None and not reachable_workspace.contains(goal):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "goal_out_of_reachable_workspace", 0)
    if point_in_collision(start, compiled):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "start_in_collision", 0)
    if point_in_collision(goal, compiled):
        return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "goal_in_collision", 0)

    straight_line_reachable = reachable_workspace is None or reachable_workspace.segment_inside(start, goal, step_m=edge_check_step_m)
    if straight_line_reachable and not segment_in_collision(start, goal, compiled, edge_check_step_m):
        path = resample_path(np.vstack((start, goal)), waypoint_count)
        return PlannedPath(True, path, "straight_line", 0)
    if waypoint_count == 2:
        return PlannedPath(
            False,
            np.empty((0, 3), dtype=np.float64),
            "endpoint_only_path_blocked",
            0,
        )

    rng = np.random.default_rng(seed)
    nodes = [start]
    parents = [-1]
    for iteration in range(1, max_iterations + 1):
        if rng.random() < goal_sample_rate:
            sample = goal
        elif reachable_workspace is not None:
            # 关键：启用可达区域后，RRT直接在bucket tip可达体内采样，而不是在长方体里盲采样。
            sample = reachable_workspace.sample(rng)
        else:
            sample = planning_bounds.sample(rng)

        if not planning_bounds.contains(sample):
            continue
        distances = np.linalg.norm(np.asarray(nodes) - sample.reshape(1, 3), axis=1)
        nearest_index = int(np.argmin(distances))
        candidate = steer(nodes[nearest_index], sample, step_size_m)
        if not planning_bounds.contains(candidate):
            continue
        if reachable_workspace is not None and not reachable_workspace.segment_inside(
            nodes[nearest_index],
            candidate,
            step_m=edge_check_step_m,
        ):
            continue
        if segment_in_collision(nodes[nearest_index], candidate, compiled, edge_check_step_m):
            continue

        nodes.append(candidate)
        parents.append(nearest_index)
        new_index = len(nodes) - 1

        final_edge_reachable = reachable_workspace is None or reachable_workspace.segment_inside(
            candidate,
            goal,
            step_m=edge_check_step_m,
        )
        if np.linalg.norm(candidate - goal) <= step_size_m and final_edge_reachable and not segment_in_collision(candidate, goal, compiled, edge_check_step_m):
            nodes.append(goal)
            parents.append(new_index)
            raw_path = reconstruct_path(nodes, parents, len(nodes) - 1)
            simplified = shortcut_path(raw_path, compiled, edge_check_step_m, reachable_workspace=reachable_workspace)
            return PlannedPath(True, resample_path(simplified, waypoint_count), "planned", iteration)

    return PlannedPath(False, np.empty((0, 3), dtype=np.float64), "max_iterations", max_iterations)
