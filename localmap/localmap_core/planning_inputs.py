"""加载并验证一次规划所需的 live LocalMap 与 bucket tip 快照。"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

from .planning_profile import PlanningProfile


class PlanningInputError(ValueError):
    """规划输入缺失、过期或不满足坐标系契约。"""


@dataclass(frozen=True)
class LivePlanningInputs:
    local_map: Mapping[str, Any]
    bucket_tip: Mapping[str, Any]


def _freeze_json(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze_json(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze_json(item) for item in value)
    return value


def _load_json_object(name: str, path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise PlanningInputError(f"{name} live输入不存在: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise PlanningInputError(f"{name} 必须是JSON object: {path}")
    return data


def _require_fresh(
    name: str,
    value: object,
    *,
    now_s: float,
    max_age_ms: int,
) -> None:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
        raise PlanningInputError(f"{name} 时间戳必须是有限数值，实际为 {value!r}")
    age_ms = (now_s - float(value)) * 1000.0
    if age_ms < 0.0 or age_ms > max_age_ms:
        raise PlanningInputError(f"{name} 已过期或来自未来: age_ms={age_ms:.1f}, limit={max_age_ms}")


def load_live_planning_inputs(
    profile: PlanningProfile,
    *,
    now_s: float,
) -> LivePlanningInputs:
    """读取权威 live 输入；不回退到 measured/mock 文件。"""
    if not math.isfinite(now_s):
        raise PlanningInputError(f"now_s 必须是有限数值，实际为 {now_s!r}")

    local_map = _load_json_object("local_map", profile.inputs.live_local_map)
    bucket_tip = _load_json_object("bucket_tip", profile.inputs.live_bucket_tip)

    if local_map.get("schema_version") != "local_map.v1":
        raise PlanningInputError(f"local_map schema错误: {local_map.get('schema_version')!r}")
    for name, data in (("local_map", local_map), ("bucket_tip", bucket_tip)):
        if data.get("frame_id") != profile.expected_frame:
            raise PlanningInputError(
                f"{name} frame必须是 {profile.expected_frame}，实际为 {data.get('frame_id')!r}"
            )
    if bucket_tip.get("status") != "live_from_tf":
        raise PlanningInputError(
            f"bucket_tip 必须来自live TF，实际status={bucket_tip.get('status')!r}"
        )

    _require_fresh(
        "local_map.timestamp_s",
        local_map.get("timestamp_s"),
        now_s=now_s,
        max_age_ms=profile.freshness.local_map_max_age_ms,
    )
    _require_fresh(
        "bucket_tip.stamp_s",
        bucket_tip.get("stamp_s"),
        now_s=now_s,
        max_age_ms=profile.freshness.bucket_tip_max_age_ms,
    )

    position = bucket_tip.get("position_m")
    if (
        not isinstance(position, list)
        or len(position) != 3
        or any(
            isinstance(value, bool)
            or not isinstance(value, int | float)
            or not math.isfinite(value)
            for value in position
        )
    ):
        raise PlanningInputError("bucket_tip.position_m 必须是3个有限数值")

    return LivePlanningInputs(
        local_map=_freeze_json(local_map),
        bucket_tip=_freeze_json(bucket_tip),
    )


def wait_for_live_planning_inputs(
    profile: PlanningProfile,
    *,
    timeout_s: float,
    poll_interval_s: float = 0.05,
    now: Callable[[], float] = time.time,
    sleep: Callable[[float], None] = time.sleep,
) -> LivePlanningInputs:
    """Wait until the authoritative live planning snapshot becomes fresh."""
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("timeout_s must be a positive finite number")
    if not math.isfinite(poll_interval_s) or poll_interval_s <= 0.0:
        raise ValueError("poll_interval_s must be a positive finite number")
    started_at_s = now()
    if not math.isfinite(started_at_s):
        raise ValueError("now() must return a finite number")
    deadline_s = started_at_s + timeout_s
    last_error: Exception | None = None
    while True:
        current_s = now()
        try:
            return load_live_planning_inputs(profile, now_s=current_s)
        except (OSError, json.JSONDecodeError, PlanningInputError) as exc:
            last_error = exc
        remaining_s = deadline_s - current_s
        if remaining_s <= 0.0:
            raise PlanningInputError(
                "live planning inputs did not become fresh before timeout: "
                f"{last_error}"
            ) from last_error
        sleep(min(poll_interval_s, remaining_s))
