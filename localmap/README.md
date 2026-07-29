# LocalMap：右手 ROS 生产链路

唯一 ROS 坐标根为 `machine_root_ros`：`+X` 前、`+Y` 左、`+Z` 上。雷达、FK、
LocalMap、workspace、轨迹和 RViz 都使用该坐标系。Unity 左手 `machine_root` 不在
ROS TF 或 LocalMap JSON 中出现；它只在 `runtime_bridge/unity_observation_adapter.py`
组装 ONNX 38 维 observation 时转换。

## 启动

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch waji_description display.launch.py
```

另开终端：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
RUN_BUCKET_TIP_BRIDGE=1 \
RUN_TRAJECTORY_MARKERS=1 \
localmap/scripts/run_perception_stack.sh
```

启动脚本总是加载 `config/perception.json`，其中包含唯一的右手 frame、外参、目标、
裁剪边界和 topic。不要用环境变量替换这些坐标语义。

RViz Fixed Frame 使用 `machine_root_ros`，主要话题：

- `/localmap/machine_root_ros_points`
- `/bucket_tip_pose_machine_root_ros`
- `/localmap/bucket_tip_machine_root_ros_pose`
- `/occupied_cells_vis_array`

## 用 Mission 文件调整目标并预览路径

唯一目标来源是 `mission/config/excavation_cycle.json`。其中 `dig.position_m` 是挖掘点，
`dump.position_m` 是倾倒点，均为 `machine_root_ros` 右手坐标（米）。初始值只是占位值，
在现场确认前保持 `target_status: "placeholder"`。

1. 启动目标标记发布器；文件保存后 RViz 会自动刷新：

   ```bash
   cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
   source /opt/ros/jazzy/setup.bash
   source ros2_ws/install/setup.bash
   /usr/bin/python3 mission/apps/publish_mission_markers.py
   ```

2. 对挖掘点运行全局预览规划：

   ```bash
   localmap/scripts/run_planning_once.sh \
     --mission mission/config/excavation_cycle.json \
     --phase dig \
     --planning-scope preview_global
   ```

   倾倒点把 `--phase dig` 改为 `--phase dump`。`preview_global` 跳过 bucket-tip
   可达域，但仍检查 live 输入新鲜度、frame、规划边界和碰撞。它只写入
   `localmap/exports/live_preview/`，不生成策略 observation，不发送动作。

3. 在另一个终端持续显示最新预览轨迹：

   ```bash
   /usr/bin/python3 localmap/scripts/publish_trajectory_markers.py \
     --trajectory localmap/exports/live_preview/trajectory_command.preview_global.json \
     --topic /localmap/preview_bucket_tip_markers
   ```

如果预览路径合理，再验证同一 Mission 目标是否位于真实可达域：

```bash
localmap/scripts/run_planning_once.sh \
  --mission mission/config/excavation_cycle.json \
  --phase dig \
  --planning-scope workspace_strict
```

`workspace_strict` 会启用对应任务模式的可达域，但仍只写入独立的
`localmap/exports/live_validation/`，不生成 observation slice、不发送动作，也不授权
真机运动。第一阶段 Mission shadow/replay 入口见 `mission/README.md`。

## 为 Orin Shadow 准备严格轨迹

PC 感知栈和 Orin Shadow 均稳定后，可冻结当前 LocalMap 与 Bucket Tip，生成一份
`execution_strict` 轨迹及 SHA-256 交接清单：

```bash
localmap/scripts/prepare_orin_edge_trajectory.sh \
  --mission mission/config/excavation_cycle.json \
  --phase dump
```

输出位于：

```text
localmap/exports/live_latest/trajectory_command.simple_rrt.json
localmap/exports/live_latest/orin_edge_trajectory_handoff.json
```

该入口不会连接 Orin、不会创建动作发送器，也不会复制文件。清单会校验 Mission
来源、右手 `machine_root_ros`、执行资格、首路点与冻结铲尖距离、终点与 Mission
目标距离以及轨迹 SHA。轨迹复制到 Orin 后仍须先重启 Orin Shadow 并检查审计日志；
清单用于调试时核对轨迹来源和 SHA，不是 commissioning 阶段的额外运动门槛。
清单生成成功本身也不向真机发送动作。

## 重要边界

- `extrinsics_rslidar_to_machine_root_ros.derived.v1.json` 和右手 target/workspace 由旧
  Unity 数据代数重表达，尚不是新的现场测量；不得把它们当作 Bucket Tip 标定通过证据。
- 真机标定仍以 `EvaluationReport/2026-07-15_bucket_tip_real_machine_coordinate_diagnosis.md`
  的静止姿态 protocol 为准。
- 默认不启动运动。只有冻结 38D observation 与真机静止标定均通过后，才评审运动授权。

## 回归

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=AiryLidar/localmap \
python3 -m pytest AiryLidar/localmap/tests -q
```
