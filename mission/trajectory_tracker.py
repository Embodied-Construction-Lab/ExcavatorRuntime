"""每帧按实时 Bucket Tip 推进 waypoint 的不可变跟踪状态。"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Sequence


@dataclass(frozen=True)
class TrackerUpdate:
    current_index: int
    distance_m: float
    advanced: bool
    completed: bool
    timed_out: bool


@dataclass(frozen=True)
class TrajectoryTracker:
    waypoints: tuple[tuple[float, float, float], ...]
    tolerance_m: float
    dwell_s: float
    timeout_s: float
    current_index: int = 0
    started_at_s: float | None = None
    within_tolerance_since_s: float | None = None
    last_update_s: float | None = None
    completed: bool = False

    def __post_init__(self) -> None:
        if not self.waypoints:
            raise ValueError("trajectory至少需要一个waypoint")
        if any(len(point) != 3 or not all(math.isfinite(value) for value in point) for point in self.waypoints):
            raise ValueError("waypoint必须是3个有限数值")
        for name, value in (
            ("tolerance_m", self.tolerance_m),
            ("timeout_s", self.timeout_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name}必须大于0")
        if not math.isfinite(self.dwell_s) or self.dwell_s < 0.0:
            raise ValueError("dwell_s必须大于等于0")

    def advance(
        self,
        bucket_tip_m: Sequence[float],
        *,
        now_s: float,
    ) -> tuple["TrajectoryTracker", TrackerUpdate]:
        """用当前铲尖更新驻留时间并返回新的跟踪状态。"""
        tip = _triplet(bucket_tip_m)
        current_time = float(now_s)
        if not math.isfinite(current_time):
            raise ValueError("now_s必须是有限数值")
        if self.last_update_s is not None and current_time < self.last_update_s:
            raise ValueError("now_s不能倒退")

        target = self.waypoints[self.current_index]
        distance = math.dist(tip, target)
        if self.completed:
            return self, TrackerUpdate(self.current_index, distance, False, True, False)

        started_at = current_time if self.started_at_s is None else self.started_at_s
        timed_out = current_time - started_at > self.timeout_s
        if timed_out:
            updated = replace(self, started_at_s=started_at, last_update_s=current_time)
            return updated, TrackerUpdate(self.current_index, distance, False, False, True)

        within_since = self.within_tolerance_since_s
        if distance > self.tolerance_m:
            within_since = None
        elif within_since is None:
            within_since = current_time

        advanced = False
        completed = False
        current_index = self.current_index
        # 时间戳使用浮点秒，0.7 - 0.4 可能得到 0.29999999999999993。
        # 允许机器精度量级误差，避免恰好到达驻留阈值时漏推进一个 waypoint。
        if within_since is not None and current_time - within_since + 1e-12 >= self.dwell_s:
            advanced = True
            if current_index == len(self.waypoints) - 1:
                completed = True
            else:
                current_index += 1
                distance = math.dist(tip, self.waypoints[current_index])
            within_since = None

        updated = replace(
            self,
            current_index=current_index,
            started_at_s=started_at,
            within_tolerance_since_s=within_since,
            last_update_s=current_time,
            completed=completed,
        )
        return updated, TrackerUpdate(current_index, distance, advanced, completed, False)


def _triplet(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError("bucket_tip_m必须是长度3数组")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError("bucket_tip_m必须是有限数值")
    return converted
