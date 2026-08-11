"""One control-stage policy shared by live planning, control and operator UI."""

from __future__ import annotations

from dataclasses import dataclass


CONTROL_STAGES = ("commissioning",)


@dataclass(frozen=True)
class ControlStagePolicy:
    name: str
    enforce_actuator_position_bounds: bool
    require_field_validated_targets: bool
    require_field_validated_workspace: bool
    allowed_target_statuses: frozenset[str]


_POLICIES = {
    "commissioning": ControlStagePolicy(
        name="commissioning",
        enforce_actuator_position_bounds=False,
        require_field_validated_targets=False,
        require_field_validated_workspace=False,
        allowed_target_statuses=frozenset({"rviz_adjusted", "field_validated"}),
    ),
}


def control_stage_policy(name: str) -> ControlStagePolicy:
    """Resolve a validated immutable policy; unknown stages fail closed."""
    try:
        return _POLICIES[name]
    except (KeyError, TypeError) as exc:
        supported = ", ".join(CONTROL_STAGES)
        raise ValueError(
            f"unsupported control_stage {name!r}; supported: {supported}"
        ) from exc
