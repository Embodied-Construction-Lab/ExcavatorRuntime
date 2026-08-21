import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path


LOCALMAP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOCALMAP_DIR))

from localmap_core.planning_inputs import (
    load_live_planning_inputs,
    wait_for_live_planning_inputs,
)
from localmap_core.planning_profile import load_planning_profile


class PlanningInputsTest(unittest.TestCase):
    def test_waits_for_startup_to_replace_stale_live_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        "timestamp_s": 999.9,
                        "frame_id": "machine_root_ros",
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 900.0,
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
            now_s = [1000.0]
            sleep_count = [0]

            def sleep(interval_s):
                now_s[0] += interval_s
                sleep_count[0] += 1
                if sleep_count[0] == 2:
                    bucket_tip_path.write_text(
                        json.dumps(
                            {
                                "stamp_s": now_s[0],
                                "frame_id": "machine_root_ros",
                                "status": "live_from_tf",
                                "position_m": [0.4, 0.5, 0.6],
                            }
                        ),
                        encoding="utf-8",
                    )

            snapshot = wait_for_live_planning_inputs(
                profile,
                timeout_s=1.0,
                poll_interval_s=0.1,
                now=lambda: now_s[0],
                sleep=sleep,
            )

        self.assertEqual(snapshot.bucket_tip["position_m"], (0.4, 0.5, 0.6))
        self.assertEqual(sleep_count[0], 2)

    def test_wait_for_live_inputs_times_out_with_the_last_rejection_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = load_planning_profile()
            profile = replace(
                profile,
                inputs=replace(
                    profile.inputs,
                    live_local_map=root / "missing-local-map.json",
                    live_bucket_tip=root / "missing-bucket-tip.json",
                ),
            )
            now_s = [1000.0]

            def sleep(interval_s):
                now_s[0] += interval_s

            with self.assertRaisesRegex(
                ValueError,
                "did not become fresh before timeout.*local_map live输入不存在",
            ):
                wait_for_live_planning_inputs(
                    profile,
                    timeout_s=0.2,
                    poll_interval_s=0.1,
                    now=lambda: now_s[0],
                    sleep=sleep,
                )

    def test_loads_fresh_machine_root_inputs_as_immutable_snapshot(self):
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

            snapshot = load_live_planning_inputs(profile, now_s=1000.0)

        self.assertEqual(snapshot.local_map["frame_id"], "machine_root_ros")
        self.assertEqual(snapshot.bucket_tip["position_m"], (0.1, 0.2, 0.3))
        with self.assertRaises(TypeError):
            snapshot.local_map["frame_id"] = "fake_base"

    def test_rejects_stale_live_bucket_tip(self):
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
                    }
                ),
                encoding="utf-8",
            )
            bucket_tip_path.write_text(
                json.dumps(
                    {
                        "stamp_s": 998.0,
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

            with self.assertRaisesRegex(ValueError, "bucket_tip.stamp_s.*过期"):
                load_live_planning_inputs(profile, now_s=1000.0)

    def test_accepts_live_local_map_written_one_cycle_ago(self):
        """LocalMap按5帧落盘；正常调度抖动不能使一次规划随机失败。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            local_map_path = root / "local_map.json"
            bucket_tip_path = root / "bucket_tip.json"
            local_map_path.write_text(
                json.dumps(
                    {
                        "schema_version": "local_map.v1",
                        # 真实失败样本：规划读取时LocalMap为529.6 ms旧。
                        "timestamp_s": 999.4704,
                        "frame_id": "machine_root_ros",
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

            snapshot = load_live_planning_inputs(profile, now_s=1000.0)

        self.assertEqual(snapshot.local_map["timestamp_s"], 999.4704)


if __name__ == "__main__":
    unittest.main()
