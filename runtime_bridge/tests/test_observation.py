import json
import math
import unittest
from pathlib import Path

from runtime_bridge.observation import (
    BucketTipObservation,
    ObservationBuilder,
    normalize_position,
    normalize_velocity,
    position_observation_range,
)
from runtime_bridge.protocol import MachineStatePacket, decode_packet, encode_packet
from runtime_bridge.unity_observation_adapter import UnityObservationAdapter
from runtime_bridge.apps.pc_policy_bridge import (
    build_arg_parser,
    denormalize_policy_action,
    make_policy_action,
    should_send_policy,
)


def sample_profile():
    return {
        "actuators": {
            "boom": {"range": [-0.1, 0.1], "sign": 1, "max_speed_positive": 0.04, "max_speed_negative": 0.02},
            "stick": {"range": [-0.2, 0.2], "sign": 1, "max_speed_positive": 0.05, "max_speed_negative": 0.05},
            "bucket": {"range": [-0.3, 0.1], "sign": 1, "max_speed_positive": 0.03, "max_speed_negative": 0.06},
            "swing": {"range": [None, None], "sign": 1, "max_speed_positive": 0.6, "max_speed_negative": 0.6},
        },
        "observation_schema": {
            "total_dim": 38,
            "normalizers": {
                "position_normalizer": 1.13,
                "tip_velocity_scale": 0.05,
                "distance_normalizer": 1.13,
                "tube_radius": 0.04,
                "target_threshold": 0.03,
                "pitch_norm_deg": 180.0,
            },
        },
        "task_profile": {"bucket_pitch_targets_deg": {"MoveToDig": 70.0, "CarryMaterial": 180.0}},
    }


def sample_state(control_enabled=False):
    return MachineStatePacket(
        seq=10,
        stamp_ms=1000,
        safety={
            "estop": False,
            "stm32_alive": True,
            "sensor_valid": True,
            "control_enabled": control_enabled,
            "fault_flags": [],
        },
        actuator_state={
            "boom": {"position_m": 0.0, "velocity_mps": 0.02},
            "stick": {"position_m": -0.2, "velocity_mps": -0.025},
            "bucket": {"position_m": 0.1, "velocity_mps": 0.03},
            "swing": {"position_rad": math.pi / 2.0, "velocity_rad_s": 0.3},
        },
        joint_state={"position_rad": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0}},
    )


class ObservationBuilderTest(unittest.TestCase):
    def test_shared_profile_uses_verified_true_machine_cylinder_ranges(self):
        workspace_root = Path(__file__).resolve().parents[3]
        profile = json.loads(
            (workspace_root / "shared" / "machine_profile.json").read_text(
                encoding="utf-8"
            )
        )
        actuators = profile["actuators"]

        self.assertEqual(position_observation_range(actuators["boom"]), (0.070, 0.190))
        self.assertEqual(position_observation_range(actuators["stick"]), (0.060, 0.210))
        self.assertEqual(position_observation_range(actuators["bucket"]), (0.060, 0.170))

    def test_true_machine_cylinder_feedback_maps_to_unity_observation_direction(self):
        workspace_root = Path(__file__).resolve().parents[3]
        profile = json.loads(
            (workspace_root / "shared" / "machine_profile.json").read_text(
                encoding="utf-8"
            )
        )
        actuators = profile["actuators"]

        for name in ("boom", "stick", "bucket"):
            actuator = actuators[name]
            lower, upper = position_observation_range(actuator)
            self.assertEqual(normalize_position(upper, actuator), -1.0)
            self.assertEqual(normalize_position(lower, actuator), 1.0)
            self.assertLess(normalize_velocity(0.01, actuator), 0.0)

        self.assertLess(normalize_velocity(0.01, actuators["swing"]), 0.0)

    def test_rejects_invalid_deploy_observation_sign(self):
        actuator = {
            "range": [-0.1, 0.1],
            "sign": 1,
            "deploy_observation_sign": 0,
            "max_speed_positive": 0.04,
            "max_speed_negative": 0.02,
            "deploy_position_observation": {
                "source": "stm32_absolute_cable_encoder",
                "range": [0.07, 0.19],
                "status": "firmware_safety_bounds",
            },
        }

        with self.assertRaisesRegex(ValueError, "deploy_observation_sign"):
            normalize_position(0.1, actuator)

    def test_swing_feedback_angle_and_velocity_are_adapted_before_onnx(self):
        profile = sample_profile()
        profile["actuators"]["swing"]["deploy_observation_sign"] = -1
        builder = ObservationBuilder(profile, task_mode="MoveToDig")

        observation = builder.build(
            sample_state(),
            BucketTipObservation((0.0, 0.0, 0.0), 0.0, stamp_ms=1000),
            [0.0] * 12,
            previous_action=[0.0] * 4,
        )

        self.assertAlmostEqual(observation[6], -1.0)
        self.assertAlmostEqual(observation[7], 0.0, places=7)
        self.assertAlmostEqual(observation[8], -0.5)

    def test_ros_right_handed_spatial_inputs_preserve_unity_38d_observation(self):
        adapter = UnityObservationAdapter()
        unity_builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        ros_builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        unity_tip = BucketTipObservation((0.113, -0.226, 0.565), math.radians(90.0), stamp_ms=1000)
        ros_tip = BucketTipObservation((0.565, -0.113, -0.226), math.radians(90.0), stamp_ms=1000)
        unity_waypoints = [0.10, -0.20, 0.30, -0.40, 0.50, -0.60, 0.70, 0.80, -0.90, 0.2, 0.3, 0.0]
        ros_waypoints = [0.30, -0.10, -0.20, -0.60, 0.40, 0.50, -0.90, -0.70, 0.80, 0.2, 0.3, 0.0]

        unity_observation = unity_builder.build(
            sample_state(), unity_tip, unity_waypoints, previous_action=[0.1, -0.2, 0.3, -0.4]
        )
        adapted_observation = ros_builder.build(
            sample_state(),
            adapter.bucket_tip_to_unity(ros_tip),
            adapter.waypoint_values_to_unity(ros_waypoints),
            previous_action=[0.1, -0.2, 0.3, -0.4],
        )

        self.assertEqual(adapted_observation, unity_observation)

    def test_ros_right_handed_tip_velocity_preserves_unity_observation(self):
        adapter = UnityObservationAdapter()
        unity_builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        ros_builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        waypoints = [0.0] * 12
        unity_first = BucketTipObservation((0.10, -0.20, 0.30), math.radians(30.0), stamp_ms=1000)
        unity_second = BucketTipObservation((0.11, -0.18, 0.33), math.radians(48.0), stamp_ms=1100)
        ros_first = BucketTipObservation((0.30, -0.10, -0.20), math.radians(30.0), stamp_ms=1000)
        ros_second = BucketTipObservation((0.33, -0.11, -0.18), math.radians(48.0), stamp_ms=1100)

        unity_builder.build(sample_state(), unity_first, waypoints, previous_action=[0.0] * 4)
        ros_builder.build(
            sample_state(), adapter.bucket_tip_to_unity(ros_first), waypoints, previous_action=[0.0] * 4
        )
        unity_observation = unity_builder.build(sample_state(), unity_second, waypoints, previous_action=[0.0] * 4)
        adapted_observation = ros_builder.build(
            sample_state(), adapter.bucket_tip_to_unity(ros_second), waypoints, previous_action=[0.0] * 4
        )

        self.assertEqual(adapted_observation, unity_observation)
        for actual, expected in zip(adapted_observation[12:15], [2.0, 4.0, 6.0]):
            self.assertAlmostEqual(actual, expected)

    def test_ros_pose_adapter_uses_unity_signed_pitch_definition(self):
        adapter = UnityObservationAdapter()

        adapted_tip = adapter.ros_pose_to_unity_bucket_tip(
            position_m=(0.565, -0.113, -0.226),
            orientation_xyzw=(0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0)),
            stamp_ms=1784098376248,
            swing_joint_rad=0.0,
        )

        self.assertEqual(adapted_tip.position_m, (0.113, -0.226, 0.565))
        self.assertAlmostEqual(adapted_tip.pitch_rad, math.pi / 2.0)
        self.assertEqual(adapted_tip.stamp_ms, 1784098376248)

        opposite = adapter.ros_pose_to_unity_bucket_tip(
            position_m=(0.0, 0.0, 0.0),
            orientation_xyzw=(0.0, -math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0)),
            stamp_ms=1784098376249,
            swing_joint_rad=0.0,
        )
        self.assertAlmostEqual(opposite.pitch_rad, -math.pi / 2.0)

    def test_ros_pose_adapter_rejects_invalid_quaternion(self):
        with self.assertRaisesRegex(ValueError, "quaternion"):
            UnityObservationAdapter().ros_pose_to_unity_bucket_tip(
                position_m=(0.0, 0.0, 0.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 0.0),
                stamp_ms=1000,
            )

    def test_policy_bridge_requires_explicit_motion_enable(self):
        parser = build_arg_parser()
        defaults = parser.parse_args([])

        self.assertEqual(set(vars(defaults)), {"config", "task_mode"})
        with self.assertRaises(SystemExit):
            parser.parse_args(["--enable-motion"])

    def test_builds_locked_38d_observation_indices(self):
        builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        waypoint_values = [i / 100.0 for i in range(12)]

        obs = builder.build(
            sample_state(),
            BucketTipObservation((0.113, -0.226, 0.565), math.radians(90.0), stamp_ms=1000),
            waypoint_values,
            previous_action=[0.1, -0.2, 0.3, -0.4],
        )

        self.assertEqual(len(obs), 38)
        self.assertAlmostEqual(obs[0], 0.0)
        self.assertAlmostEqual(obs[1], 0.5)
        self.assertAlmostEqual(obs[2], -1.0)
        self.assertAlmostEqual(obs[6], 1.0)
        self.assertAlmostEqual(obs[7], 0.0, places=6)
        self.assertAlmostEqual(obs[8], 0.5)
        self.assertEqual(obs[15:27], waypoint_values)
        self.assertEqual(obs[27:29], [1.0, 0.0])
        self.assertEqual(obs[30:34], [0.1, -0.2, 0.3, -0.4])
        self.assertAlmostEqual(obs[9], 0.113 / 1.13)
        self.assertAlmostEqual(obs[10], -0.226 / 1.13)
        self.assertAlmostEqual(obs[11], 0.565 / 1.13)
        self.assertAlmostEqual(obs[34], 90.0 / 180.0)
        self.assertAlmostEqual(obs[35], 70.0 / 180.0)
        self.assertAlmostEqual(obs[36], 20.0 / 180.0)

    def test_second_frame_estimates_tip_and_pitch_velocity(self):
        builder = ObservationBuilder(sample_profile(), task_mode="MoveToDig")
        zeros = [0.0] * 12

        builder.build(
            sample_state(),
            BucketTipObservation((0.0, 0.0, 0.0), 0.0, stamp_ms=1000),
            zeros,
            previous_action=[0.0, 0.0, 0.0, 0.0],
        )
        obs = builder.build(
            sample_state(),
            BucketTipObservation((0.005, 0.0, 0.0), math.radians(18.0), stamp_ms=1100),
            zeros,
            previous_action=[0.0, 0.0, 0.0, 0.0],
        )

        self.assertAlmostEqual(obs[12], 1.0)
        self.assertAlmostEqual(obs[37], 1.0)

    def test_action_packet_round_trips_and_safety_gate_defaults_to_zero_when_disabled(self):
        send_policy, reason = should_send_policy(sample_state(control_enabled=False))
        self.assertFalse(send_policy)
        self.assertEqual(reason, "control_disabled")

        packet = make_policy_action(3, [2.0, -2.0, 0.25, 0.0], 100, "normalized_velocity_command")
        decoded = decode_packet(encode_packet(packet))

        self.assertEqual(decoded.action, [2.0, -2.0, 0.25, 0.0])
        self.assertEqual(decoded.action_type, "normalized_velocity_command")

    def test_denormalizes_policy_action_to_physical_velocity_command(self):
        action = denormalize_policy_action([1.0, -1.0, 0.5, -0.5], sample_profile())

        self.assertEqual(action, [0.04, -0.05, 0.015, -0.3])

    def test_pc_joint_observation_matches_unity_profile_sign(self):
        actuator = {
            "range": [0.0, 2.0],
            "sign": -1,
            "max_speed_positive": 2.0,
            "max_speed_negative": 4.0,
        }

        self.assertEqual(normalize_position(0.5, actuator), 0.5)
        self.assertEqual(normalize_velocity(1.0, actuator), -0.25)

    def test_deploy_absolute_encoder_range_overrides_unity_prismatic_range(self):
        actuator = {
            "range": [-0.095, 0.080],
            "sign": 1,
            "deploy_position_observation": {
                "source": "stm32_absolute_cable_encoder",
                "range": [0.140, 0.190],
                "status": "firmware_safety_bounds",
            },
        }

        self.assertEqual(position_observation_range(actuator), (0.140, 0.190))
        self.assertAlmostEqual(normalize_position(0.1572, actuator), -0.312)

    def test_rejects_malformed_deploy_position_observation(self):
        actuator = {
            "range": [-0.1, 0.1],
            "deploy_position_observation": {"range": [0.2, 0.1]},
        }

        with self.assertRaisesRegex(ValueError, "deploy_position_observation"):
            position_observation_range(actuator)

    def test_rejects_pc_encoder_direction_metadata(self):
        actuator = {
            "range": [-0.1, 0.1],
            "deploy_position_observation": {
                "source": "stm32_absolute_cable_encoder",
                "range": [0.06, 0.16],
                "status": "firmware_safety_bounds",
                "command_to_encoder_velocity_sign": -1,
            },
        }

        with self.assertRaisesRegex(ValueError, "deploy_position_observation contract"):
            position_observation_range(actuator)


if __name__ == "__main__":
    unittest.main()
