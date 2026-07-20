import json
import unittest
from pathlib import Path

from runtime_bridge.live_control import (
    FollowCanaryEnvelope,
    LIVE_MOTION_AUTHORIZATION,
    MotionCommandSink,
    build_dynamic_waypoint_values,
    evaluate_actuator_state,
    evaluate_state_provenance,
    build_manual_jog_action,
    evaluate_follow_canary_supervision,
    motion_authorization_granted,
)
from runtime_bridge.protocol import MachineStatePacket, decode_packet


def sample_state(*, seq=1, control_enabled=True, sensor_valid=True, estop=False):
    return MachineStatePacket(
        seq=seq,
        stamp_ms=1000 + seq,
        safety={
            "estop": estop,
            "stm32_alive": True,
            "sensor_valid": sensor_valid,
            "control_enabled": control_enabled,
            "fault_flags": [],
        },
        actuator_state={
            "boom": {"position_m": 0.0, "velocity_mps": 0.0},
            "stick": {"position_m": 0.0, "velocity_mps": 0.0},
            "bucket": {"position_m": 0.0, "velocity_mps": 0.0},
            "swing": {"position_rad": 0.0, "velocity_rad_s": 0.0},
        },
        joint_state={
            "position_rad": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0}
        },
    )


class RecordingSender:
    def __init__(self):
        self.payloads = []

    def send(self, payload):
        self.payloads.append(payload)


class LiveControlTests(unittest.TestCase):
    def test_follow_supervision_requires_matching_fresh_heartbeat_without_fixed_duration_cap(self):
        valid = evaluate_follow_canary_supervision(
            expected_session="trajectory-001",
            heartbeat_session="trajectory-001",
            heartbeat_age_ms=50.0,
            heartbeat_timeout_ms=175,
        )
        self.assertTrue(valid.allowed)

        missing = evaluate_follow_canary_supervision(
            expected_session="trajectory-001",
            heartbeat_session="",
            heartbeat_age_ms=float("inf"),
            heartbeat_timeout_ms=175,
        )
        self.assertEqual(missing.reason, "supervision_heartbeat_timeout")

        mismatched = evaluate_follow_canary_supervision(
            expected_session="trajectory-001",
            heartbeat_session="trajectory-002",
            heartbeat_age_ms=10.0,
            heartbeat_timeout_ms=175,
        )
        self.assertEqual(mismatched.reason, "supervision_heartbeat_timeout")

        long_running = evaluate_follow_canary_supervision(
            expected_session="trajectory-001",
            heartbeat_session="trajectory-001",
            heartbeat_age_ms=10.0,
            heartbeat_timeout_ms=175,
        )
        self.assertTrue(long_running.allowed)

    def test_follow_policy_preserves_normalized_output_and_derives_full_physical_bounds(self):
        profile = {
            "actuators": {
                "boom": {
                    "max_speed_positive": 0.04,
                    "max_speed_negative": 0.02,
                },
                "stick": {
                    "max_speed_positive": 0.05,
                    "max_speed_negative": 0.03,
                },
                "bucket": {
                    "max_speed_positive": 0.06,
                    "max_speed_negative": 0.01,
                },
                "swing": {
                    "max_speed_positive": 1.0,
                    "max_speed_negative": 1.0,
                },
            }
        }
        envelope = FollowCanaryEnvelope.from_machine_profile(
            profile,
            allowed_actuators=("boom", "stick", "bucket", "swing"),
        )

        applied = envelope.apply_normalized((1.0, -0.5, 0.25, 1.0))

        self.assertEqual(applied, (1.0, -0.5, 0.25, 1.0))
        self.assertTrue(envelope.evaluate_physical((0.04, -0.015, 0.015, 1.0)).allowed)
        self.assertEqual(
            envelope.evaluate_physical((-0.03, 0.0, 0.0, 0.0)).reason,
            "follow_canary_envelope_violation",
        )

    def test_follow_canary_rejects_malformed_output_but_preserves_finite_values(self):
        profile = {
            "actuators": {
                name: {
                    "max_speed_positive": 1.0,
                    "max_speed_negative": 1.0,
                }
                for name in ("boom", "stick", "bucket", "swing")
            }
        }
        envelope = FollowCanaryEnvelope.from_machine_profile(
            profile, allowed_actuators=("boom", "stick", "bucket", "swing")
        )

        with self.assertRaisesRegex(ValueError, "four finite"):
            envelope.apply_normalized((0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "four finite"):
            envelope.apply_normalized((0.0, float("nan"), 0.0, 0.0))
        self.assertEqual(
            envelope.apply_normalized((1.25, -1.5, 0.75, -1.0)),
            (1.25, -1.5, 0.75, -1.0),
        )

    def test_command_sink_rejects_direct_canary_envelope_bypass_with_zero(self):
        profile = {
            "actuators": {
                name: {
                    "max_speed_positive": 0.1,
                    "max_speed_negative": 0.1,
                }
                for name in ("boom", "stick", "bucket", "swing")
            }
        }
        envelope = FollowCanaryEnvelope.from_machine_profile(
            profile,
            allowed_actuators=("boom", "stick", "bucket"),
        )
        sender = RecordingSender()
        sink = MotionCommandSink(sender, valid_for_ms=100)

        decision = sink.send_velocity(
            sample_state(seq=1),
            [0.0, 0.0, 0.0, 0.01],
            action_stamp_ms=2000,
            physical_envelope=envelope,
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "follow_canary_axis_locked")
        self.assertEqual(decode_packet(sender.payloads[-1]).action, [0.0] * 4)

    def test_actuator_state_must_be_finite_and_inside_machine_profile_ranges(self):
        profile = {
            "actuators": {
                "boom": {"range": [-0.1, 0.1]},
                "stick": {"range": [-0.2, 0.2]},
                "bucket": {"range": [-0.3, 0.1]},
                "swing": {"range": [None, None]},
            }
        }
        self.assertTrue(evaluate_actuator_state(sample_state(), profile).allowed)
        invalid = sample_state()
        invalid.actuator_state["bucket"]["position_m"] = 0.161
        decision = evaluate_actuator_state(invalid, profile)

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "bucket_position_out_of_range")

    def test_operator_bypass_skips_only_position_bounds(self):
        profile = {
            "actuators": {
                "boom": {"range": [-0.1, 0.1]},
                "stick": {"range": [-0.2, 0.2]},
                "bucket": {"range": [-0.3, 0.1]},
                "swing": {"range": [None, None]},
            }
        }
        outside = sample_state()
        outside.actuator_state["bucket"]["position_m"] = 0.161

        decision = evaluate_actuator_state(
            outside,
            profile,
            enforce_bounds=False,
        )

        self.assertTrue(decision.allowed)
        self.assertEqual(decision.reason, "actuator_position_bounds_not_enforced")

        outside.actuator_state["bucket"]["position_m"] = float("nan")
        nonfinite = evaluate_actuator_state(
            outside,
            profile,
            enforce_bounds=False,
        )
        self.assertFalse(nonfinite.allowed)
        self.assertEqual(nonfinite.reason, "bucket_position_invalid")

    def test_live_gate_uses_deploy_absolute_encoder_ranges(self):
        profile = {
            "actuators": {
                "boom": {
                    "range": [-0.095, 0.080],
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.140, 0.190],
                        "status": "firmware_safety_bounds",
                    },
                },
                "stick": {
                    "range": [-0.060, 0.090],
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.060, 0.220],
                        "status": "firmware_safety_bounds",
                    },
                },
                "bucket": {
                    "range": [-0.085, 0.045],
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.060, 0.160],
                        "status": "firmware_safety_bounds",
                    },
                },
                "swing": {"range": [None, None]},
            }
        }
        state = sample_state()
        state.actuator_state["boom"]["position_m"] = 0.1572
        state.actuator_state["stick"]["position_m"] = 0.15603
        state.actuator_state["bucket"]["position_m"] = 0.1585

        self.assertTrue(evaluate_actuator_state(state, profile).allowed)

    def test_field_observed_boom_position_preserves_both_action_directions(self):
        profile_path = Path(__file__).resolve().parents[2] / "../shared/machine_profile.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        state = sample_state()
        state.actuator_state["boom"]["position_m"] = 0.13810
        state.actuator_state["stick"]["position_m"] = 0.15620
        state.actuator_state["bucket"]["position_m"] = 0.15848
        state.actuator_state["swing"]["position_rad"] = 0.34730

        self.assertTrue(evaluate_actuator_state(state, profile).allowed)

        action_positive = build_manual_jog_action(
            state,
            profile,
            actuator="boom",
            direction=1,
            allowed_actuators=("boom", "stick", "bucket"),
            speed_fraction=0.1,
        )
        action_negative = build_manual_jog_action(
            state,
            profile,
            actuator="boom",
            direction=-1,
            allowed_actuators=("boom", "stick", "bucket"),
            speed_fraction=0.1,
        )

        self.assertTrue(action_positive.allowed)
        self.assertEqual(action_positive.physical_action, (0.00351, 0.0, 0.0, 0.0))
        self.assertTrue(action_negative.allowed)
        self.assertEqual(action_negative.physical_action, (-0.00185, 0.0, 0.0, 0.0))

    def test_motion_authorization_requires_exact_deliberate_token(self):
        self.assertTrue(motion_authorization_granted(LIVE_MOTION_AUTHORIZATION))
        self.assertFalse(motion_authorization_granted("true"))
        self.assertFalse(motion_authorization_granted(""))

    def test_state_provenance_requires_orin_machine_and_strict_sequence(self):
        state = sample_state(seq=5)
        self.assertTrue(
            evaluate_state_provenance(
                state, expected_machine_id="scale_excavator_v1", last_seq=4
            ).allowed
        )
        self.assertEqual(
            evaluate_state_provenance(
                state, expected_machine_id="other-machine", last_seq=4
            ).reason,
            "machine_id_mismatch",
        )
        self.assertEqual(
            evaluate_state_provenance(
                state, expected_machine_id="scale_excavator_v1", last_seq=5
            ).reason,
            "state_sequence_not_increasing",
        )
        self.assertEqual(
            evaluate_state_provenance(
                state,
                expected_machine_id="scale_excavator_v1",
                last_seq=4,
                last_stamp_ms=state.stamp_ms,
            ).reason,
            "state_stamp_not_increasing",
        )
        self.assertEqual(
            evaluate_state_provenance(
                state,
                expected_machine_id="scale_excavator_v1",
                last_seq=4,
                received_pc_ms=1000,
                expected_clock_offset_ms=-1000,
            ).reason,
            "state_clock_offset_jump",
        )

    def test_command_sink_sends_physical_action_only_when_all_state_gates_pass(self):
        sender = RecordingSender()
        sink = MotionCommandSink(sender, valid_for_ms=100)

        decision = sink.send_velocity(
            sample_state(seq=1), [0.01, -0.02, 0.03, -0.4], action_stamp_ms=2000
        )

        packet = decode_packet(sender.payloads[-1])
        self.assertTrue(decision.allowed)
        self.assertEqual(packet.action, [0.01, -0.02, 0.03, -0.4])
        self.assertEqual(packet.action_order, ("boom", "stick", "bucket", "swing"))
        self.assertEqual(packet.stamp_ms, 2000)

    def test_command_sink_replaces_disabled_or_out_of_order_state_with_zero(self):
        sender = RecordingSender()
        sink = MotionCommandSink(sender, valid_for_ms=100)

        disabled = sink.send_velocity(
            sample_state(seq=5, control_enabled=False), [1.0] * 4, action_stamp_ms=2000
        )
        out_of_order = sink.send_velocity(
            sample_state(seq=4), [1.0] * 4, action_stamp_ms=2001
        )

        self.assertFalse(disabled.allowed)
        self.assertEqual(disabled.reason, "control_disabled")
        self.assertFalse(out_of_order.allowed)
        self.assertEqual(out_of_order.reason, "state_out_of_order")
        self.assertEqual(decode_packet(sender.payloads[0]).action, [0.0] * 4)
        self.assertEqual(decode_packet(sender.payloads[1]).action, [0.0] * 4)

    def test_command_sink_allows_bounded_repeat_from_same_verified_state(self):
        sender = RecordingSender()
        sink = MotionCommandSink(sender, valid_for_ms=100, max_state_age_s=0.2)
        state = sample_state(seq=5)

        first = sink.send_velocity(
            state, [0.01, 0.0, 0.0, 0.0], action_stamp_ms=2000, state_age_s=0.0
        )
        repeated = sink.send_velocity(
            state, [0.01, 0.0, 0.0, 0.0], action_stamp_ms=2050, state_age_s=0.05
        )

        self.assertTrue(first.allowed)
        self.assertTrue(repeated.allowed)
        self.assertEqual(decode_packet(sender.payloads[-1]).action, [0.01, 0.0, 0.0, 0.0])

    def test_command_sink_zeroes_stale_state_and_rejects_physical_limit_violation(self):
        sender = RecordingSender()
        sink = MotionCommandSink(
            sender,
            valid_for_ms=100,
            max_state_age_s=0.2,
            physical_action_limits=(0.1, 0.2, 0.3, 0.4),
        )

        stale = sink.send_velocity(
            sample_state(seq=1), [0.01, 0.0, 0.0, 0.0], action_stamp_ms=2000, state_age_s=0.21
        )
        self.assertFalse(stale.allowed)
        self.assertEqual(stale.reason, "state_stale")
        self.assertEqual(decode_packet(sender.payloads[-1]).action, [0.0] * 4)

        with self.assertRaisesRegex(ValueError, "physical_action exceeds configured limit"):
            sink.send_velocity(
                sample_state(seq=2), [0.11, 0.0, 0.0, 0.0], action_stamp_ms=2001
            )

    def test_command_sink_disarm_makes_all_later_velocity_requests_zero(self):
        sender = RecordingSender()
        sink = MotionCommandSink(sender, valid_for_ms=100)
        sink.send_velocity(
            sample_state(seq=1), [0.01, 0.0, 0.0, 0.0], action_stamp_ms=2000
        )

        sink.disarm(action_stamp_ms=2001)
        decision = sink.send_velocity(
            sample_state(seq=2), [0.01, 0.0, 0.0, 0.0], action_stamp_ms=2002
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "command_sink_disarmed")
        self.assertEqual(decode_packet(sender.payloads[-2]).action, [0.0] * 4)
        self.assertEqual(decode_packet(sender.payloads[-1]).action, [0.0] * 4)

    def test_manual_jog_preserves_requested_action_direction_without_encoder_sign(self):
        profile = {
            "actuators": {
                "boom": {
                    "action_index": 0,
                    "max_speed_positive": 0.04,
                    "max_speed_negative": 0.02,
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.14, 0.19],
                        "status": "firmware_safety_bounds",
                    },
                },
                "stick": {
                    "action_index": 1,
                    "max_speed_positive": 0.05,
                    "max_speed_negative": 0.03,
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.06, 0.22],
                        "status": "firmware_safety_bounds",
                    },
                },
                "bucket": {
                    "action_index": 2,
                    "max_speed_positive": 0.03,
                    "max_speed_negative": 0.06,
                    "deploy_position_observation": {
                        "source": "stm32_absolute_cable_encoder",
                        "range": [0.06, 0.16],
                        "status": "firmware_safety_bounds",
                    },
                },
                "swing": {"action_index": 3},
            }
        }
        state = sample_state()
        state.actuator_state["boom"]["position_m"] = 0.16
        state.actuator_state["stick"]["position_m"] = 0.10
        state.actuator_state["bucket"]["position_m"] = 0.159

        allowed = build_manual_jog_action(
            state,
            profile,
            actuator="boom",
            direction=1,
            allowed_actuators=("boom", "stick", "bucket"),
            speed_fraction=0.1,
        )
        self.assertTrue(allowed.allowed)
        self.assertEqual(allowed.physical_action, (0.004, 0.0, 0.0, 0.0))

        toward_upper = build_manual_jog_action(
            state,
            profile,
            actuator="bucket",
            direction=1,
            allowed_actuators=("boom", "stick", "bucket"),
            speed_fraction=0.1,
        )
        away_from_upper = build_manual_jog_action(
            state,
            profile,
            actuator="bucket",
            direction=-1,
            allowed_actuators=("boom", "stick", "bucket"),
            speed_fraction=0.1,
        )
        self.assertTrue(toward_upper.allowed)
        self.assertEqual(toward_upper.physical_action, (0.0, 0.0, 0.003, 0.0))
        self.assertTrue(away_from_upper.allowed)
        self.assertEqual(away_from_upper.physical_action, (0.0, 0.0, -0.006, 0.0))

    def test_manual_jog_rejects_swing_invalid_direction_and_untrusted_range(self):
        profile = {
            "actuators": {
                "boom": {
                    "action_index": 0,
                    "max_speed_positive": 0.04,
                    "max_speed_negative": 0.02,
                    "range": [-0.1, 0.1],
                },
                "stick": {"action_index": 1},
                "bucket": {"action_index": 2},
                "swing": {"action_index": 3},
            }
        }
        with self.assertRaisesRegex(ValueError, "allowed actuator"):
            build_manual_jog_action(
                sample_state(), profile, actuator="swing", direction=1,
                allowed_actuators=("boom", "stick", "bucket"), speed_fraction=0.1,
            )
        with self.assertRaisesRegex(ValueError, "direction"):
            build_manual_jog_action(
                sample_state(), profile, actuator="boom", direction=0,
                allowed_actuators=("boom", "stick", "bucket"), speed_fraction=0.1,
            )
        with self.assertRaisesRegex(ValueError, "firmware_safety_bounds"):
            build_manual_jog_action(
                sample_state(), profile, actuator="boom", direction=1,
                allowed_actuators=("boom", "stick", "bucket"), speed_fraction=0.1,
            )

    def test_dynamic_waypoint_values_follow_current_tip_and_index(self):
        trajectory = {
            "frame_id": "machine_root_ros",
            "waypoints_base": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            "waypoint_count": 3,
            "tube_radius": 0.1,
        }
        profile = {
            "observation_schema": {
                "waypoint_lookahead": 3,
                "normalizers": {"distance_normalizer": 2.0, "tube_radius": 0.1},
            }
        }

        values = build_dynamic_waypoint_values(
            trajectory, profile, bucket_tip_ros=(0.5, 0.0, 0.0), current_index=1
        )

        self.assertEqual(values[:9], [0.25, 0.0, 0.0, 0.75, 0.0, 0.0, 0.75, 0.0, 0.0])
        self.assertAlmostEqual(values[9], 0.5)
        self.assertEqual(values[11], 0.0)


if __name__ == "__main__":
    unittest.main()
