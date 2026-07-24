import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from runtime_bridge.apps.fixed_action_player import build_arg_parser
from runtime_bridge.fixed_actions import (
    FixedActionExecutor,
    FixedActionProfileError,
    fixed_action_contract_sha256,
    load_fixed_action_profile,
    physical_velocity_action_from_normalized,
)
from runtime_bridge.protocol import MachineStatePacket, decode_packet, encode_packet


def sample_profile():
    return {
        "action_order": ["boom", "stick", "bucket", "swing"],
        "actuators": {
            "boom": {
                "range": [-0.1, 0.1],
                "sign": 1,
                "max_speed_positive": 0.04,
                "max_speed_negative": 0.02,
            },
            "stick": {
                "range": [-0.2, 0.2],
                "sign": 1,
                "max_speed_positive": 0.05,
                "max_speed_negative": 0.05,
            },
            "bucket": {
                "range": [-0.3, 0.1],
                "sign": 1,
                "max_speed_positive": 0.03,
                "max_speed_negative": 0.06,
            },
            "swing": {
                "range": [None, None],
                "sign": 1,
                "max_speed_positive": 0.6,
                "max_speed_negative": 0.6,
                "calibrated": False,
            },
        }
    }


def sample_state(boom=0.0, stick=0.0, bucket=-0.1, control_enabled=True):
    return MachineStatePacket(
        seq=1,
        stamp_ms=1000,
        safety={
            "estop": False,
            "stm32_alive": True,
            "sensor_valid": True,
            "control_enabled": control_enabled,
            "fault_flags": [],
        },
        actuator_state={
            "boom": {"position_m": boom, "velocity_mps": 0.0},
            "stick": {"position_m": stick, "velocity_mps": 0.0},
            "bucket": {"position_m": bucket, "velocity_mps": 0.0},
            "swing": {"position_rad": 0.0, "velocity_rad_s": 0.0},
        },
        joint_state={"position_rad": {"swing": 0.0, "boom": 0.0, "arm": 0.0, "bucket": 0.0}},
    )


def sample_fixed_action_profile(machine_profile: bytes, urdf: bytes):
    return {
        "schema_version": "fixed_action_profile.v1",
        "profile_id": "test_actions_v1",
        "machine_id": "scale_excavator_v1",
        "action_order": ["boom", "stick", "bucket", "swing"],
        "validation_status": "candidate",
        "validation_evidence": None,
        "machine_profile_sha256": hashlib.sha256(machine_profile).hexdigest(),
        "urdf_sha256": hashlib.sha256(urdf).hexdigest(),
        "controller": {
            "kp": 1.5,
            "min_action": 0.08,
            "max_action": 0.2,
            "tolerance": 0.03,
            "step_timeout_s": 3.0,
            "hold_s": 0.15,
        },
        "start_envelopes": {
            "dig": {
                "normalized_actuator_position": {
                    "boom": [-0.5, 0.5],
                    "stick": [-1.0, 1.0],
                    "bucket": [-1.0, 1.0],
                },
                "bucket_pitch_deg": [-90.0, 90.0],
                "swing_rad": [-0.5, 0.5],
            },
            "dump": {
                "normalized_actuator_position": {
                    "boom": [-1.0, 1.0],
                    "stick": [-1.0, 1.0],
                    "bucket": [-1.0, 1.0],
                },
                "bucket_pitch_deg": [-180.0, 180.0],
                "swing_rad": [-1.57, 1.57],
            },
        },
        "actions": {
            "dig": [
                {
                    "step_id": "dig_probe",
                    "label": "probe",
                    "delta_by_actuator": {
                        "boom": 0.5, "stick": 0.0, "bucket": 0.0, "swing": 0.0
                    },
                },
                {
                    "step_id": "dig_curl",
                    "label": "curl",
                    "delta_by_actuator": {
                        "boom": 0.0, "stick": 0.0, "bucket": -1.0, "swing": 0.0
                    },
                },
            ],
            "dump": [
                {
                    "step_id": "dump_open",
                    "label": "open",
                    "delta_by_actuator": {
                        "boom": 0.0, "stick": 0.0, "bucket": 1.0, "swing": 0.0
                    },
                },
            ],
        },
    }


def load_sample_fixed_action_profile(mutator=None):
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    machine_profile = json.dumps(
        {"machine_id": "scale_excavator_v1", **sample_profile()},
        sort_keys=True,
    ).encode()
    urdf = b"<robot name='test'/>"
    payload = sample_fixed_action_profile(machine_profile, urdf)
    if mutator is not None:
        mutator(payload)
    machine_profile_path = root / "machine_profile.json"
    urdf_path = root / "machine.urdf"
    profile_path = root / "fixed_actions.json"
    machine_profile_path.write_bytes(machine_profile)
    urdf_path.write_bytes(urdf)
    profile_path.write_text(json.dumps(payload), encoding="utf-8")
    expected_sha256 = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    return temporary, load_fixed_action_profile(
        profile_path,
        machine_profile_path=machine_profile_path,
        urdf_path=urdf_path,
        expected_sha256=expected_sha256,
    )


class FixedActionTest(unittest.TestCase):
    def test_deployed_candidate_uses_unity_deltas_and_requested_constant_sixty_percent_speed(self):
        project_root = Path(__file__).resolve().parents[2]
        payload = json.loads(
            (project_root / "runtime_bridge/config/fixed_actions.json").read_text(
                encoding="utf-8"
            )
        )
        machine_profile = json.loads(
            (project_root.parent / "shared/machine_profile.json").read_text(
                encoding="utf-8"
            )
        )

        self.assertEqual(payload["controller"]["min_action"], 0.6)
        self.assertEqual(payload["controller"]["max_action"], 0.6)
        self.assertEqual(
            [step["delta_by_actuator"] for step in payload["actions"]["dig"]],
            [
                {"boom": 0.5, "stick": 0.0, "bucket": 0.0, "swing": 0.0},
                {"boom": 0.0, "stick": 0.0, "bucket": -1.4, "swing": 0.0},
                {"boom": -0.5, "stick": 0.2, "bucket": 0.0, "swing": 0.0},
            ],
        )
        self.assertEqual(
            [step["delta_by_actuator"] for step in payload["actions"]["dump"]],
            [
                {"boom": 0.0, "stick": 0.0, "bucket": 1.4, "swing": 0.0},
                {"boom": 0.0, "stick": 0.0, "bucket": -1.4, "swing": 0.0},
            ],
        )
        self.assertEqual(
            physical_velocity_action_from_normalized(
                [0.0, 0.0, 0.6, 0.0], machine_profile
            ),
            [0.0, 0.0, 0.02052, 0.0],
        )
        self.assertEqual(
            physical_velocity_action_from_normalized(
                [0.0, 0.0, -0.6, 0.0], machine_profile
            ),
            [0.0, 0.0, -0.02514, 0.0],
        )

    def test_shipped_step_timeout_covers_slowest_fixed_action_with_response_margin(self):
        project_root = Path(__file__).resolve().parents[2]
        payload = json.loads(
            (project_root / "runtime_bridge/config/fixed_actions.json").read_text(
                encoding="utf-8"
            )
        )
        machine_profile = json.loads(
            (project_root.parent / "shared/machine_profile.json").read_text(
                encoding="utf-8"
            )
        )
        controller = payload["controller"]
        action_order = payload["action_order"]
        slowest_ideal_s = 0.0

        for steps in payload["actions"].values():
            for step in steps:
                for actuator_name in action_order:
                    delta = float(step["delta_by_actuator"][actuator_name])
                    if delta == 0.0 or actuator_name == "swing":
                        continue
                    actuator = machine_profile["actuators"][actuator_name]
                    range_min, range_max = actuator["range"]
                    travel_m = max(
                        abs(delta) - float(controller["tolerance"]),
                        0.0,
                    ) * (float(range_max) - float(range_min)) / 2.0
                    speed_key = (
                        "max_speed_positive" if delta > 0.0 else "max_speed_negative"
                    )
                    command_speed_mps = (
                        float(controller["max_action"]) * float(actuator[speed_key])
                    )
                    slowest_ideal_s = max(
                        slowest_ideal_s,
                        travel_m / command_speed_mps,
                    )

        self.assertGreaterEqual(
            float(controller["step_timeout_s"]),
            slowest_ideal_s * 1.25,
            "单步超时必须覆盖最慢固定动作的理论行程，并保留至少 25% 响应余量",
        )

    def test_shipped_measured_directional_deadzones_zero_each_axis_at_breakaway_threshold(self):
        machine_profile = json.loads(
            (Path(__file__).resolve().parents[2].parent / "shared/machine_profile.json")
            .read_text(encoding="utf-8")
        )
        actuators = machine_profile["actuators"]
        positive_thresholds = [
            actuators[name]["command_deadzone_positive_normalized"]
            for name in ("boom", "stick", "bucket", "swing")
        ]
        negative_thresholds = [
            -actuators[name]["command_deadzone_negative_normalized"]
            for name in ("boom", "stick", "bucket", "swing")
        ]

        self.assertEqual(
            physical_velocity_action_from_normalized(positive_thresholds, machine_profile),
            [0.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            physical_velocity_action_from_normalized(negative_thresholds, machine_profile),
            [0.0, 0.0, 0.0, 0.0],
        )

    def test_loads_versioned_fixed_action_profile_bound_to_machine_and_urdf(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)

        self.assertEqual(profile.profile_id, "test_actions_v1")
        self.assertEqual(profile.validation_status, "candidate")
        self.assertEqual(profile.controller.max_action, 0.2)
        self.assertEqual(profile.action_order, ("boom", "stick", "bucket", "swing"))
        self.assertEqual(profile.sequence("dig")[0].label, "probe")
        self.assertEqual(
            profile.sequence("dump")[0].delta_normalized_qpos,
            (0.0, 0.0, 1.0, 0.0),
        )
        self.assertEqual(len(profile.sha256), 64)

    def test_fixed_action_start_envelope_rejects_unvalidated_joint_configuration(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)

        accepted = profile.evaluate_start(
            "dig", sample_state(), sample_profile(), bucket_pitch_rad=0.0
        )
        rejected = profile.evaluate_start(
            "dig", sample_state(boom=0.09), sample_profile(), bucket_pitch_rad=0.0
        )

        self.assertTrue(accepted.allowed)
        self.assertFalse(rejected.allowed)
        self.assertEqual(rejected.reason, "dig_boom_outside_start_envelope")

    def test_rejects_profile_when_machine_profile_digest_does_not_match(self):
        with self.assertRaisesRegex(FixedActionProfileError, "machine_profile_sha256"):
            load_sample_fixed_action_profile(
                lambda payload: payload.update(machine_profile_sha256="0" * 64)
            )

    def test_rejects_profile_when_urdf_digest_does_not_match(self):
        with self.assertRaisesRegex(FixedActionProfileError, "urdf_sha256"):
            load_sample_fixed_action_profile(
                lambda payload: payload.update(urdf_sha256="0" * 64)
            )

    def test_rejects_profile_when_runtime_pin_does_not_match(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        machine_profile = json.dumps(
            {"machine_id": "scale_excavator_v1", **sample_profile()}, sort_keys=True
        ).encode()
        urdf = b"<robot/>"
        (root / "machine_profile.json").write_bytes(machine_profile)
        (root / "machine.urdf").write_bytes(urdf)
        (root / "fixed_actions.json").write_text(
            json.dumps(sample_fixed_action_profile(machine_profile, urdf)), encoding="utf-8"
        )
        with self.assertRaisesRegex(FixedActionProfileError, "expected_sha256"):
            load_fixed_action_profile(
                root / "fixed_actions.json",
                machine_profile_path=root / "machine_profile.json",
                urdf_path=root / "machine.urdf",
                expected_sha256="0" * 64,
            )

    def test_rejects_unknown_profile_fields_and_wrong_action_order(self):
        invalid_mutators = (
            lambda payload: payload.update(validation_state="field_validated"),
            lambda payload: payload.update(action_order=["swing", "boom", "stick", "bucket"]),
        )
        for mutator in invalid_mutators:
            with self.subTest(mutator=mutator), self.assertRaises(FixedActionProfileError):
                load_sample_fixed_action_profile(mutator)

    def test_rejects_invalid_status_or_action_step(self):
        invalid_mutators = (
            lambda payload: payload.update(validation_status="assumed"),
            lambda payload: payload["actions"]["dig"][0].update(delta_by_actuator={}),
            lambda payload: payload["actions"]["dump"][0]["delta_by_actuator"].update(
                bucket=float("nan")
            ),
            lambda payload: payload.update(
                validation_status="field_validated", validation_evidence=None
            ),
        )
        for mutator in invalid_mutators:
            with self.subTest(mutator=mutator), self.assertRaises(FixedActionProfileError):
                load_sample_fixed_action_profile(mutator)

    def test_field_validated_profile_requires_real_evaluation_report_with_matching_digest(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        report = root / "EvaluationReport" / "fixed_action_validation.md"
        report.parent.mkdir()
        machine_profile = json.dumps(
            {"machine_id": "scale_excavator_v1", **sample_profile()}, sort_keys=True
        ).encode()
        urdf = b"<robot/>"
        payload = sample_fixed_action_profile(machine_profile, urdf)
        contract_sha256 = fixed_action_contract_sha256(payload)
        report.write_text(
            "# Fixed action field validation\n\n"
            "fixed_action_profile_id: test_actions_v1\n"
            f"fixed_action_contract_sha256: {contract_sha256}\n"
            "experiment_run_id: field_action_001\n",
            encoding="utf-8",
        )
        payload.update(
            validation_status="field_validated",
            validation_evidence={
                "validated_at": "2026-07-17T12:00:00+08:00",
                "validated_by": "field_operator",
                "evaluation_report": "EvaluationReport/fixed_action_validation.md",
                "evaluation_report_sha256": hashlib.sha256(report.read_bytes()).hexdigest(),
                "experiment_run_ids": ["field_action_001"],
                "validated_phases": ["dig", "dump"],
                "max_validated_normalized_command": 0.2,
                "action_contract_sha256": contract_sha256,
            },
        )
        machine_profile_path = root / "machine_profile.json"
        urdf_path = root / "machine.urdf"
        profile_path = root / "fixed_actions.json"
        machine_profile_path.write_bytes(machine_profile)
        urdf_path.write_bytes(urdf)
        profile_path.write_text(json.dumps(payload), encoding="utf-8")

        profile = load_fixed_action_profile(
            profile_path,
            machine_profile_path=machine_profile_path,
            urdf_path=urdf_path,
            expected_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
            workspace_root=root,
        )
        self.assertEqual(profile.validation_status, "field_validated")

        report.write_text("unrelated but digest-matched report\n", encoding="utf-8")
        payload["validation_evidence"]["evaluation_report_sha256"] = hashlib.sha256(
            report.read_bytes()
        ).hexdigest()
        profile_path.write_text(json.dumps(payload), encoding="utf-8")
        with self.assertRaisesRegex(FixedActionProfileError, "未绑定当前动作契约"):
            load_fixed_action_profile(
                profile_path,
                machine_profile_path=machine_profile_path,
                urdf_path=urdf_path,
                expected_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                workspace_root=root,
            )

        report.write_text("tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(FixedActionProfileError, "报告文件不一致"):
            load_fixed_action_profile(
                profile_path,
                machine_profile_path=machine_profile_path,
                urdf_path=urdf_path,
                expected_sha256=hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                workspace_root=root,
            )

    def test_fixed_action_player_has_no_motion_enable_or_sender_option(self):
        parser = build_arg_parser()
        defaults = parser.parse_args(["dig"])

        self.assertEqual(set(vars(defaults)), {"action", "config"})
        with self.assertRaises(SystemExit):
            parser.parse_args(["dig", "--enable-motion"])

    def test_denormalizes_action_to_physical_velocity(self):
        action = physical_velocity_action_from_normalized([1.0, -1.0, 0.5, -0.5], sample_profile())

        self.assertEqual(action, [0.04, -0.05, 0.015, -0.3])

    def test_directional_command_deadzones_zero_small_swing_but_keep_larger_swing_proportional(self):
        profile = sample_profile()
        profile["actuators"]["swing"]["command_deadzone_positive_normalized"] = 0.2
        profile["actuators"]["swing"]["command_deadzone_negative_normalized"] = 0.3

        action = physical_velocity_action_from_normalized(
            [0.0, 0.0, 0.0, -0.5], profile
        )

        self.assertEqual(action, [0.0, 0.0, 0.0, -0.3])
        self.assertEqual(
            physical_velocity_action_from_normalized([0.0, 0.0, 0.0, 0.2], profile),
            [0.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            physical_velocity_action_from_normalized([0.0, 0.0, 0.0, -0.25], profile),
            [0.0, 0.0, 0.0, 0.0],
        )

    def test_command_deadzone_rejects_invalid_profile_value(self):
        profile = sample_profile()
        profile["actuators"]["swing"]["command_deadzone_positive_normalized"] = 1.0

        with self.assertRaisesRegex(ValueError, "swing.command_deadzone_positive_normalized"):
            physical_velocity_action_from_normalized([0.0, 0.0, 0.0, 0.5], profile)

    def test_policy_to_physical_velocity_matches_unity_single_entry_clamp(self):
        action = physical_velocity_action_from_normalized(
            [1.25, -1.5, 0.5, -0.5], sample_profile()
        )

        self.assertEqual(action, [0.04, -0.05, 0.015, -0.3])

    def test_policy_direction_is_independent_from_profile_observation_sign(self):
        profile = sample_profile()
        profile["actuators"]["boom"]["sign"] = -1
        profile["actuators"]["boom"]["deploy_observation_sign"] = -1

        action = physical_velocity_action_from_normalized(
            [1.0, 0.0, 0.0, 0.0], profile
        )

        self.assertEqual(action, [0.04, 0.0, 0.0, 0.0])

    def test_live_policy_actions_preserve_onnx_direction_before_stm32(self):
        profile = json.loads(
            (Path(__file__).resolve().parents[2].parent / "shared/machine_profile.json")
            .read_text(encoding="utf-8")
        )

        action = physical_velocity_action_from_normalized(
            [1.0, -1.0, 1.0, -1.0], profile
        )

        self.assertGreater(action[0], 0.0)
        self.assertLess(action[1], 0.0)
        self.assertGreater(action[2], 0.0)
        self.assertLess(action[3], 0.0)

    def test_shipped_machine_profile_has_no_pc_encoder_direction_metadata(self):
        profile = json.loads(
            (Path(__file__).resolve().parents[2].parent / "shared/machine_profile.json")
            .read_text(encoding="utf-8")
        )

        for actuator in ("boom", "stick", "bucket"):
            deploy = profile["actuators"][actuator]["deploy_position_observation"]
            self.assertNotIn("command_to_encoder_velocity_sign", deploy)

    def test_dig_first_step_generates_orin_compatible_physical_velocity_packet(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        executor = FixedActionExecutor(profile.sequence("dig"), sample_profile())

        packet, status = executor.step(sample_state(), now_s=0.0, seq=7, valid_for_ms=100)
        decoded = decode_packet(encode_packet(packet))

        self.assertFalse(status.done)
        self.assertEqual(decoded.action_type, "normalized_velocity_command")
        self.assertEqual(decoded.action_order, ("boom", "stick", "bucket", "swing"))
        self.assertAlmostEqual(decoded.action[0], 0.03)
        self.assertEqual(decoded.action[1:], [0.0, 0.0, 0.0])

    def test_equal_min_and_max_action_keeps_speed_constant_until_tolerance(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        executor = FixedActionExecutor(
            profile.sequence("dig"),
            sample_profile(),
            min_action=0.6,
            max_action=0.6,
            tolerance=0.03,
        )

        far_packet, _ = executor.step(
            sample_state(boom=0.0), now_s=0.0, seq=1, valid_for_ms=100
        )
        near_packet, _ = executor.step(
            sample_state(boom=0.04), now_s=0.1, seq=2, valid_for_ms=100
        )
        reached_packet, _ = executor.step(
            sample_state(boom=0.048), now_s=0.2, seq=3, valid_for_ms=100
        )

        self.assertEqual(far_packet.action[0], 0.024)
        self.assertEqual(near_packet.action[0], 0.024)
        self.assertEqual(reached_packet.action, [0.0, 0.0, 0.0, 0.0])

    def test_fixed_action_preserves_servo_error_direction_like_onnx_action(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        machine_profile = sample_profile()
        machine_profile["actuators"]["boom"]["deploy_position_observation"] = {
            "source": "stm32_absolute_cable_encoder",
            "range": [-0.1, 0.1],
            "status": "firmware_safety_bounds",
        }
        executor = FixedActionExecutor(profile.sequence("dig"), machine_profile)

        packet, _ = executor.step(
            sample_state(), now_s=0.0, seq=7, valid_for_ms=100
        )

        self.assertGreater(packet.action[0], 0.0)

    def test_dump_sequence_opens_bucket_then_reports_done_after_target_reached(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        executor = FixedActionExecutor(profile.sequence("dump"), sample_profile(), tolerance=0.03)

        first_packet, first_status = executor.step(sample_state(bucket=-0.1), now_s=0.0, seq=1, valid_for_ms=100)
        self.assertFalse(first_status.done)
        self.assertGreater(first_packet.action[2], 0.0)

        done_packet, done_status = executor.step(sample_state(bucket=0.1), now_s=0.2, seq=2, valid_for_ms=100)
        self.assertFalse(done_status.done)
        self.assertEqual(done_packet.action, [0.0, 0.0, 0.0, 0.0])

    def test_dump_opens_true_machine_bucket_from_closed_encoder_endpoint(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        machine_profile = sample_profile()
        machine_profile["actuators"]["bucket"]["deploy_position_observation"] = {
            "source": "stm32_absolute_cable_encoder",
            "range": [0.06, 0.16],
            "status": "firmware_safety_bounds",
        }
        machine_profile["actuators"]["bucket"]["deploy_observation_sign"] = -1
        executor = FixedActionExecutor(
            profile.sequence("dump"),
            machine_profile,
            min_action=0.6,
            max_action=0.6,
            tolerance=0.03,
        )

        packet, status = executor.step(
            sample_state(bucket=0.16005),
            now_s=0.0,
            seq=1,
            valid_for_ms=100,
        )

        self.assertFalse(status.done)
        self.assertGreater(packet.action[2], 0.0)
        self.assertEqual(packet.action[0:2] + packet.action[3:], [0.0, 0.0, 0.0])

    def test_step_timeout_fails_closed_without_advancing(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        executor = FixedActionExecutor(
            profile.sequence("dig"),
            sample_profile(),
            step_timeout_s=0.2,
        )
        executor.step(sample_state(), now_s=0.0, seq=1, valid_for_ms=100)

        timeout_packet, timeout_status = executor.step(
            sample_state(), now_s=0.21, seq=2, valid_for_ms=100
        )
        repeated_packet, repeated_status = executor.step(
            sample_state(), now_s=0.37, seq=3, valid_for_ms=100
        )

        self.assertEqual(timeout_packet.action, [0.0, 0.0, 0.0, 0.0])
        self.assertTrue(timeout_status.failed)
        self.assertEqual(timeout_status.phase, "failed")
        self.assertEqual(timeout_status.reason_code, "STEP_TIMEOUT")
        self.assertFalse(timeout_status.done)
        self.assertEqual(executor.step_index, 0)
        self.assertEqual(repeated_packet.action, [0.0, 0.0, 0.0, 0.0])
        self.assertTrue(repeated_status.failed)
        self.assertEqual(repeated_status.reason_code, "STEP_TIMEOUT")

    def test_relative_target_outside_normalized_range_is_clamped_like_unity(self):
        temporary, profile = load_sample_fixed_action_profile()
        self.addCleanup(temporary.cleanup)
        executor = FixedActionExecutor(profile.sequence("dig"), sample_profile())

        packet, status = executor.step(
            sample_state(boom=0.09), now_s=0.0, seq=1, valid_for_ms=100
        )

        self.assertGreater(packet.action[0], 0.0)
        self.assertFalse(status.failed)
        self.assertEqual(status.phase, "running")


if __name__ == "__main__":
    unittest.main()
