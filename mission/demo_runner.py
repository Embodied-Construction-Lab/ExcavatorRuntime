"""多点挖掘演示的确定性顺序调度。"""

from __future__ import annotations

from collections.abc import Callable

from mission.demo import DemoDigPoint, ExcavationDemoProgram


class DemoCycleFailed(RuntimeError):
    def __init__(self, point_id: str, reason: str) -> None:
        super().__init__(f"{point_id}: {reason}")
        self.point_id = point_id
        self.reason = reason


def run_demo_cycles(
    program: ExcavationDemoProgram,
    *,
    repeat: int,
    run_cycle: Callable[[DemoDigPoint, int, int], None],
) -> int:
    """按配置顺序执行完整循环；任一循环失败时立即向上传播。"""
    if isinstance(repeat, bool) or not isinstance(repeat, int) or repeat <= 0:
        raise ValueError("repeat必须是正整数")
    cycle_count = repeat * len(program.dig_points)
    completed = 0
    for _round_index in range(repeat):
        for point in program.dig_points:
            run_cycle(point, completed + 1, cycle_count)
            completed += 1
    return completed
