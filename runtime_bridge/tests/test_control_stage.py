import unittest

from runtime_bridge.control_stage import control_stage_policy


class ControlStagePolicyTest(unittest.TestCase):
    def test_commissioning_keeps_hard_machine_gates_but_downgrades_calibration_gates(self):
        policy = control_stage_policy("commissioning")

        self.assertFalse(policy.enforce_actuator_position_bounds)
        self.assertFalse(policy.require_field_validated_targets)
        self.assertFalse(policy.require_field_validated_workspace)
        self.assertEqual(
            policy.allowed_target_statuses,
            frozenset({"rviz_adjusted", "field_validated"}),
        )

    def test_retired_production_stage_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "control_stage"):
            control_stage_policy("production")

    def test_unknown_stage_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "control_stage"):
            control_stage_policy("unsafe")


if __name__ == "__main__":
    unittest.main()
