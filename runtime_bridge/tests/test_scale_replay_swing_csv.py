import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PROJECT_ROOT / "runtime_bridge" / "apps" / "scale_replay_swing_csv.py"
SPEC = importlib.util.spec_from_file_location("scale_replay_swing_csv", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ScaleReplaySwingCsvTest(unittest.TestCase):
    def test_scales_swing_and_clamps_to_unity_envelope_without_changing_other_axes(self):
        source = """# profile_action_order=boom|stick|bucket|swing
sample_index,timestamp_s,swing_action_cmd,boom_v_ref_mps,stick_v_ref_mps,bucket_v_ref_mps,swing_v_ref_radps
0,0,0.1,0.01,-0.02,0.03,0.1
1,0.05,-0.1,0.01,-0.02,0.03,-0.3
2,0.1,0,0,0,0,0
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            output_path = root / "output.csv"
            input_path.write_text(source, encoding="utf-8")

            rows, peak = MODULE.transform_csv(
                input_path, output_path, scale=3.0, max_abs_radps=0.6
            )
            with output_path.open(encoding="utf-8-sig", newline="") as output_file:
                text = output_file.read()
            parsed = list(csv.DictReader(line for line in text.splitlines() if not line.startswith("#")))

        self.assertEqual(rows, 3)
        self.assertEqual(peak, 0.6)
        self.assertIn("# derived_transform=swing_scale_then_clamp", text)
        self.assertEqual(parsed[0]["boom_v_ref_mps"], "0.01")
        self.assertEqual(parsed[0]["stick_v_ref_mps"], "-0.02")
        self.assertEqual(parsed[0]["bucket_v_ref_mps"], "0.03")
        self.assertEqual(parsed[0]["swing_v_ref_radps"], "0.3")
        self.assertEqual(parsed[0]["swing_action_cmd"], "0.5")
        self.assertEqual(parsed[1]["swing_v_ref_radps"], "-0.6")
        self.assertEqual(parsed[1]["swing_action_cmd"], "-1")
        self.assertEqual(parsed[2]["swing_v_ref_radps"], "0")

    def test_can_create_swing_only_replay(self):
        source = """timestamp_s,boom_action_cmd,stick_action_cmd,bucket_action_cmd,swing_action_cmd,boom_v_ref_mps,stick_v_ref_mps,bucket_v_ref_mps,swing_v_ref_radps
0,0.1,-0.2,0.3,0.1,0.01,-0.02,0.03,0.1
1,0,0,0,0,0,0,0,0
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            output_path = root / "output.csv"
            input_path.write_text(source, encoding="utf-8")

            MODULE.transform_csv(
                input_path,
                output_path,
                scale=3.0,
                max_abs_radps=0.6,
                zero_other_axes=True,
            )
            with output_path.open(encoding="utf-8-sig", newline="") as output_file:
                text = output_file.read()
            parsed = list(csv.DictReader(line for line in text.splitlines() if not line.startswith("#")))

        self.assertIn("# derived_transform=swing_only_scale_then_clamp", text)
        self.assertEqual(
            [parsed[0][column] for column in (
                "boom_action_cmd", "stick_action_cmd", "bucket_action_cmd",
                "boom_v_ref_mps", "stick_v_ref_mps", "bucket_v_ref_mps",
            )],
            ["0"] * 6,
        )
        self.assertEqual(parsed[0]["swing_v_ref_radps"], "0.3")

    def test_rejects_nonzero_terminal_swing(self):
        source = """timestamp_s,swing_action_cmd,swing_v_ref_radps
0,0.1,0.1
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.csv"
            input_path.write_text(source, encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "explicit zero"):
                MODULE.transform_csv(
                    input_path, root / "output.csv", scale=3.0, max_abs_radps=0.6
                )


if __name__ == "__main__":
    unittest.main()
