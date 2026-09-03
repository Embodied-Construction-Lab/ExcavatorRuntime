# TADPS selector live-capture bridge

This bridge records the complete candidate set immediately before the legacy
`excavator_dig_point` selector performs temporal selection. It is evidence-only:
the default is disabled, it does not publish motion commands, and it does not
change the selector's existing result topics or scoring.

The authoritative bridge artifacts are:

- `excavator_dig_point_tadps_candidate_trace.v1.patch`
- `excavator_dig_point_tadps_candidate_trace.v1.json`
- `../tadps_live_capture.py`
- `../scripts/capture_tadps_candidate_frames.py`

The descriptor binds the patch, original edited files, patched source/config
files, and runtime contract to lowercase SHA-256 digests. The capture CLI refuses
to start unless the supplied selector package matches the patched digests. A
capture is valid only when it ends with `capture_manifest.json`; malformed,
cross-run, nonzero-first-index, internally dropped, or reordered frames leave an
unblessed JSONL trace and no manifest. The reliable volatile transport and
selector shutdown drain reduce tail loss, but v1 has no terminal-count record;
the manifest attests the observed contiguous sequence and does not prove that an
unseen final frame never existed.

## Apply and build the legacy selector bridge

Run these commands from the ROS workspace containing the legacy package. The
patch paths are relative to its `src` directory.

```bash
cd /home/zhaoshuai/workspace_excavator/excavator_perception/src
git apply --unidiff-zero --check \
  /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/mission/bridges/excavator_dig_point_tadps_candidate_trace.v1.patch
git apply --unidiff-zero \
  /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/mission/bridges/excavator_dig_point_tadps_candidate_trace.v1.patch

cd /home/zhaoshuai/workspace_excavator/excavator_perception
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select excavator_dig_point
colcon test --packages-select excavator_dig_point
colcon test-result --verbose
```

If the patch is already applied, do not apply it again. The live-capture CLI
below verifies the post-patch source and configuration before subscribing.

## Capture one pile-stage sequence

Use the same non-empty sequence ID in both terminals. Start the read-only logger
first so a volatile ROS message cannot be missed.

Terminal 1 — evidence capture:

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
python3 mission/scripts/capture_tadps_candidate_frames.py \
  --output-directory /absolute/evidence/tadps/pile-stage-001 \
  --sequence-id pile-stage-001 \
  --selector-source-root \
    /home/zhaoshuai/workspace_excavator/excavator_perception/src/excavator_dig_point
```

Terminal 2 — explicitly opt in the selector trace:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_excavator/excavator_perception/install/setup.zsh
ros2 launch excavator_dig_point full_pipeline.launch.py \
  start_rviz:=false \
  candidate_trace_enabled:=true \
  candidate_trace_sequence_id:=pile-stage-001
```

Stop the selector after the intended pile stage, wait until the capture output is
quiet, then press `Ctrl-C` in the capture terminal. Keep both capture files, the
append-only
`candidate_frames.jsonl` and `capture_manifest.json`, together with any
operator/run metadata stored alongside them. Do not use a directory from an
earlier run; creation is exclusive by design.

## Export the frozen replay

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 mission/scripts/export_tadps_candidate_replay.py \
  --input-jsonl /absolute/evidence/tadps/pile-stage-001/candidate_frames.jsonl \
  --output-dir /absolute/evidence/tadps/pile-stage-001-replay
```

`downstream_planner_accepted` is deliberately `null`: this seam captures the
selector's preselection evidence and does not invent planner outcomes. Therefore
planner-acceptance metrics remain unavailable until a separately versioned
planner-result join is implemented and captured.
