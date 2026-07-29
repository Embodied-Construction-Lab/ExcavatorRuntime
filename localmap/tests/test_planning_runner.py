import sys
import io
import json
import subprocess
import tempfile
import unittest
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
from types import MappingProxyType
from unittest.mock import patch


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from apps.planning.run_planning_once import (
    build_arg_parser,
    build_planning_commands,
    execute_planning_commands,
    execute_prepared_run,
    execute_staged_run,
    invalidate_outputs,
    outputs_for_scope,
    PlanningCommand,
    PreparedPlanningRun,
    prepare_orin_edge_planning_run,
    prepare_mission_planning_run,
    publish_prepared_run,
)
from localmap_core.planning_intent import PlanningIntent
from localmap_core.planning_inputs import LivePlanningInputs
from localmap_core.planning_profile import PlanningOutputs, load_planning_profile


class PlanningRunnerTest(unittest.TestCase):
    def test_cli_uses_mission_phase_and_non_executable_scope(self):
        args = build_arg_parser().parse_args(
            ["--mission", "mission.json", "--phase", "dig"]
        )

        self.assertEqual(
            set(vars(args)),
            {"mission", "phase", "planning_scope", "profile", "dry_run"},
        )
        self.assertEqual(args.phase, "dig")
        self.assertEqual(args.planning_scope, "preview_global")
        self.assertFalse(args.dry_run)

    def test_execution_profile_can_explicitly_disable_only_reachable_workspace(self):
        profile = load_planning_profile()
        intent = PlanningIntent(
            target_id="dig_01",
            target_kind="dig",
            task_mode="MoveToDig",
        )

        commands = build_planning_commands(
            profile,
            intent,
            python=Path("/usr/bin/python3"),
        )

        self.assertEqual([command.name for command in commands], ["obstacles", "request", "trajectory", "observation"])
        by_name = {command.name: command.argv for command in commands}
        self.assertIn("-2.0", by_name["obstacles"])
        self.assertIn("--target-kind", by_name["request"])
        self.assertIn("dig", by_name["request"])
        self.assertIn("MoveToDig", by_name["request"])
        self.assertIn("-1.0", by_name["trajectory"])
        self.assertNotIn("--reachable-workspace", by_name["trajectory"])
        self.assertIn("--disable-reachable-workspace", by_name["trajectory"])
        self.assertIn("--workspace-disable-reason", by_name["trajectory"])
        self.assertIn("operator_temporary_workspace_invalid", by_name["trajectory"])
        self.assertIn("0.45", by_name["trajectory"])
        self.assertIn("--current-index", by_name["observation"])

    def test_prepares_commands_from_fresh_inputs_and_mission_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.8,
                        "frame_id": "machine_root_ros",
                        "dig_targets": [],
                        "dump_targets": [],
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 999.9,
                        "frame_id": "machine_root_ros",
                        "status": "live_from_tf",
                        "position_m": [0.1, 0.2, 0.3],
                    }
                ),
                encoding="utf-8",
            )
            profile = load_planning_profile()
            profile = replace(
                profile,
                inputs=replace(
                    profile.inputs,
                    live_local_map=local_map_path,
                    live_bucket_tip=bucket_tip_path,
                ),
            )

            staging_dir = root / "staging"
            mission_path = LOCALMAP_DIR.parent / "mission" / "config" / "excavation_cycle.json"
            mission_data = json.loads(mission_path.read_text(encoding="utf-8"))
            prepared = prepare_mission_planning_run(
                profile,
                mission_path=mission_path,
                phase="dig",
                planning_scope="preview_global",
                now_s=1000.0,
                python=Path("/usr/bin/python3"),
                staging_dir=staging_dir,
            )

        by_name = dict((command.name, command.argv) for command in prepared.commands)
        request = by_name["request"]
        self.assertIn("dig", request)
        self.assertIn("MoveToDig", request)
        self.assertIn(f"{mission_data['mission_id']}:dig", request)
        self.assertEqual(
            prepared.snapshot.local_map["dig_targets"][-1]["position_m"],
            tuple(mission_data["targets"]["dig"]["position_m"]),
        )
        mission_reference = prepared.snapshot.local_map["dig_targets"][-1]["mission"]
        self.assertEqual(mission_reference["id"], mission_data["mission_id"])
        self.assertEqual(mission_reference["phase"], "dig")
        self.assertEqual(len(mission_reference["sha256"]), 64)
        self.assertEqual(prepared.published_artifacts, ("local_map", "request", "trajectory"))
        self.assertIn(str(staging_dir / "local_map.snapshot.json"), by_name["obstacles"])
        self.assertIn(str(staging_dir / "bucket_tip.snapshot.json"), request)
        self.assertNotIn(str(local_map_path), by_name["obstacles"])
        self.assertNotIn(str(bucket_tip_path), request)

    def test_orin_edge_run_is_execution_strict_and_publishes_live_latest_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.8,
                        "frame_id": "machine_root_ros",
                        "dig_targets": [],
                        "dump_targets": [],
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 999.9,
                        "frame_id": "machine_root_ros",
                        "status": "live_from_tf",
                        "position_m": [0.1, 0.2, 0.3],
                    }
                ),
                encoding="utf-8",
            )
            profile = load_planning_profile()
            profile = replace(
                profile,
                inputs=replace(
                    profile.inputs,
                    live_local_map=local_map_path,
                    live_bucket_tip=bucket_tip_path,
                ),
                outputs=replace(
                    profile.outputs,
                    directory=root / "live_latest",
                    local_map=root / "live_latest" / "local_map.json",
                    request=root / "live_latest" / "request.json",
                    trajectory=root / "live_latest" / "trajectory.json",
                    observation_slice=root / "live_latest" / "observation.json",
                ),
            )

            prepared = prepare_orin_edge_planning_run(
                profile,
                mission_path=LOCALMAP_DIR.parent / "mission" / "config" /
                "excavation_cycle.json",
                phase="dig",
                now_s=1000.0,
                python=Path("/usr/bin/python3"),
                staging_dir=root / "staging",
            )

        self.assertEqual(prepared.final_outputs, profile.outputs)
        self.assertEqual(
            prepared.published_artifacts,
            ("local_map", "request", "trajectory", "observation_slice"),
        )
        trajectory_command = next(
            command for command in prepared.commands if command.name == "trajectory"
        )
        self.assertIn("execution_strict", trajectory_command.argv)
        self.assertIn("--workspace-disable-reason", trajectory_command.argv)

    def test_preview_outputs_are_isolated_from_future_control_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = load_planning_profile()
            profile = replace(
                profile,
                outputs=replace(
                    profile.outputs,
                    directory=root / "live_latest",
                    local_map=root / "live_latest" / "local_map.json",
                    request=root / "live_latest" / "request.json",
                    trajectory=root / "live_latest" / "trajectory.json",
                    observation_slice=root / "live_latest" / "observation.json",
                ),
            )
            preview = outputs_for_scope(profile, "preview_global")
            preview.directory.mkdir()
            preview.trajectory.write_text("stale", encoding="utf-8")
            profile.outputs.directory.mkdir()
            profile.outputs.observation_slice.write_text("control", encoding="utf-8")

            invalidate_outputs(preview)

            self.assertFalse(preview.trajectory.exists())
            self.assertEqual(profile.outputs.observation_slice.read_text(), "control")

    def test_dry_run_prints_steps_without_starting_subprocesses(self):
        commands = (PlanningCommand("example", ("python3", "step.py")),)
        output = io.StringIO()

        with patch("apps.planning.run_planning_once.subprocess.run") as run_process:
            with redirect_stdout(output):
                execute_planning_commands(commands, dry_run=True)

        run_process.assert_not_called()
        self.assertIn("[example] python3 step.py", output.getvalue())

    def test_failed_step_does_not_publish_partial_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "staging"
            final = root / "final"
            staging.mkdir()
            final.mkdir()

            def outputs(path):
                return PlanningOutputs(
                    directory=path,
                    local_map=path / "local_map.json",
                    request=path / "request.json",
                    trajectory=path / "trajectory.json",
                    observation_slice=path / "observation.json",
                )

            staging_outputs = outputs(staging)
            final_outputs = outputs(final)
            final_outputs.observation_slice.write_text("previous", encoding="utf-8")
            prepared = PreparedPlanningRun(
                commands=(
                    PlanningCommand("first", ("python3", "first.py")),
                    PlanningCommand("second", ("python3", "second.py")),
                ),
                snapshot=LivePlanningInputs(
                    local_map=MappingProxyType({"frame_id": "machine_root_ros"}),
                    bucket_tip=MappingProxyType({"frame_id": "machine_root_ros"}),
                ),
                local_map_snapshot=staging / "local_map.snapshot.json",
                bucket_tip_snapshot=staging / "bucket_tip.snapshot.json",
                staging_outputs=staging_outputs,
                final_outputs=final_outputs,
            )

            with patch(
                "apps.planning.run_planning_once.subprocess.run",
                side_effect=(None, subprocess.CalledProcessError(1, ["second.py"])),
            ):
                with self.assertRaises(subprocess.CalledProcessError):
                    execute_prepared_run(prepared, dry_run=False)

            self.assertFalse(final_outputs.local_map.exists())
            self.assertEqual(final_outputs.observation_slice.read_text(encoding="utf-8"), "previous")

    def test_staged_execution_requires_an_explicit_publish(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "staging"
            final = root / "final"
            staging.mkdir()

            def outputs(path):
                return PlanningOutputs(
                    directory=path,
                    local_map=path / "local_map.json",
                    request=path / "request.json",
                    trajectory=path / "trajectory.json",
                    observation_slice=path / "observation.json",
                )

            staging_outputs = outputs(staging)
            final_outputs = outputs(final)
            for path in (
                staging_outputs.local_map,
                staging_outputs.request,
                staging_outputs.trajectory,
                staging_outputs.observation_slice,
            ):
                path.write_text(path.name, encoding="utf-8")
            prepared = PreparedPlanningRun(
                commands=(),
                snapshot=LivePlanningInputs(
                    local_map=MappingProxyType({"frame_id": "machine_root_ros"}),
                    bucket_tip=MappingProxyType({"frame_id": "machine_root_ros"}),
                ),
                local_map_snapshot=staging / "local_map.snapshot.json",
                bucket_tip_snapshot=staging / "bucket_tip.snapshot.json",
                staging_outputs=staging_outputs,
                final_outputs=final_outputs,
            )

            execute_staged_run(prepared)

            self.assertFalse(final_outputs.trajectory.exists())
            publish_prepared_run(prepared)
            self.assertEqual(
                final_outputs.trajectory.read_text(encoding="utf-8"),
                "trajectory.json",
            )


if __name__ == "__main__":
    unittest.main()
