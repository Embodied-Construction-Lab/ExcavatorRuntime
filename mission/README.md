# Excavation Mission

`config/excavation_cycle.json` 是挖掘点与倾倒点的唯一文件入口。两个 `position_m`
都使用 ROS 右手坐标 `machine_root_ros`：`+X` 前、`+Y` 左、`+Z` 上，单位为米。

当前现场配置使用 `rviz_adjusted`，供唯一真机入口 `live_commissioning` 使用。

`config/excavation_dig_point_catalog.v1.json` 是 V3-B 固定挖掘点的唯一入口。它保存非空
有序点位和点集；点数、点名与分组都不写死在代码中。Operator/RViz 与 WebUI/固定闭环读取
同一文件，不再维护旧三点候选表。

## RViz 中调整坐标

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 mission/apps/publish_mission_markers.py \
  --dig-point-catalog mission/config/excavation_dig_point_catalog.v1.json
```

保存 JSON 后，`/mission/target_markers` 会更新：橙色为近端 dig、蓝色为远端 dig、紫色为 dump。
默认 RViz 配置已启用 `Mission Targets`，会显示目录中的全部带 ID 候选点。

## 纯离线 replay

```bash
python3 mission/apps/run_mission_replay.py
```

默认 replay 使用相互匹配的只读夹具 `replays/mission.placeholder.json` 和
`replays/nominal.placeholder.json`，不依赖可由现场编辑的活动 Mission。结果写入
`mission/exports/replay_latest/mission_events.json`。入口没有运动参数，不创建
UDP action sender，输出中的 `action_datagrams` 必须恒为 `0`。replay 会验证完整阶段：

```text
规划到挖掘点 → 跟踪 → 稳定 → 挖掘 → 装载验证 →
从挖掘后的实时铲尖重新规划 → 跟踪 → 稳定 → 倾倒 → 空斗验证 → ReturnHome
```

任一轨迹超时、原语失败或验证失败都会进入 `failed`，不会把超时当作完成。

## ROS Action 离线 Shadow 运行时

当前已提供以下长期运行 ROS Interface：

```text
/planning/plan          airy_excavator_interfaces/action/Plan
/excavator/follow       airy_excavator_interfaces/action/Follow
/excavator/return_home  airy_excavator_interfaces/action/ReturnHome
```

`ExecuteDig` 与 `ExecuteDump` 的接口和 Machine State 输入契约仍在 Step 4，尚未伪造为
“已执行”。当前所有已实现行为固定为 `execution_mode=shadow`，使用无 sender Backend；
Result 中 `action_datagrams` 必须是 `0`。

Panel 的 Tests 标签页只提供离线 JointState 滑块；live profile 中禁用，不控制真机。
真机 `Follow`、`ExecuteDig` 与 `ExecuteDump` 全部通过 Orin Edge Runtime 的同一执行权和
Command Sink。

编译：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
/usr/bin/colcon build --packages-select \
  waji_description airy_excavator_interfaces airy_localmap airy_mission_runtime \
  airy_mission_panel airy_excavator_bringup \
  --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.zsh
```

### 单命令 RViz Panel（推荐）

完全离线验证时，不启动 Orin、STM32 或雷达。以下命令一次启动 URDF/TF、嵌入式关节测试、
Fixture Plan、Follow/ReturnHome Shadow Server、Mission Snapshot 发布器和一个 RViz：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch airy_excavator_bringup operator.launch.py profile:=fixture_shadow
```

该离线入口使用启动终端当前的默认 ROS/DDS 通信环境，不在 launch 内覆盖环境。为避免与同一
ROS 图中的现场节点重名，所有离线节点、Action、状态、JointState、TF 和可视化话题都显式
位于 `/offline` 命名空间，例如 `/offline/joint_states`、`/offline/planning/plan` 和
`/offline/mission/runtime_status`；Panel 与 RViz 的 remapping 由统一 launch 自动完成。
Panel 应显示 `FIXTURE / SHADOW / READY`，且 `action_datagrams=0`。当前可用操作是：

- `Plan + Follow DIG`、`Plan + Follow DUMP`：把本次 Plan Result 直接交给 Follow；
- `Return Home`：观察滑块是否进入所选命名位姿；
- `Cancel Panel Operation`：只取消该 Panel 自己提交的 Goal，不是急停；
- 离线 fixture 中的 `ExecuteDig`、`ExecuteDump`、`Full Mission` 不发送真机动作；
  端侧 commissioning 使用 Orin Action Gateway，多点演示由命令行客户端顺序提交。

Panel 使用三个标签页：`Actions` 放 Mission Action 和结果，`Logs` 放节点告警历史，`Tests`
放四关节测试滑块。安全状态栏始终位于标签页外。Tests 以 0.01 rad 分辨率、10 Hz 发布
`[swing_joint, boom_joint, arm_joint, bucket_joint]`；只允许在本离线 fixture 入口的
fixture/shadow/no-motion 契约满足时工作。

Panel 下方的 `Log History — Warnings / Errors (/rosout)` 只显示 ROS 标准日志中的 WARN、
ERROR 和 FATAL，每行明确给出 Time、Level、Module（logger/node 名）和 Message，最多保留
100 条，可用 `Clear` 清空；容量满时优先淘汰旧 WARN，避免告警洪泛挤掉 ERROR/FATAL。
这里显示的是历史记录，不表示故障仍在持续。它用于快速定位节点级异常，但不能替代进程日志：
节点启动失败、进程退出和 Python/C++
traceback 仍以启动 `operator.launch.py` 的终端为准，完整日志保存在
`~/.ros/log/latest/`（实际目录也会由 launch 启动时第一行打印）。

离线 No-Motion Backend 不会主动移动模型。Follow 通常在当前点完成第一段后等待滑块输入，
最终可能以 `TIMEOUT` 或 `STALE_BUCKET_TIP` 安全失败；这不是发送控制。ReturnHome 可先把滑块
调到非零，再提交 Goal，然后点击滑块 `Reset` 验证成功。

从另一个终端诊断时，只需 source 同一 ROS overlay。无显示环境的 CI 冒烟可以追加
`start_rviz:=false`；操作者正常启动时不要设置该参数。

### 真机只读 Live Shadow

统一入口的 `live_shadow` profile 会启动 Orin 状态接收与 `/joint_states` 发布、FK、雷达感知、
LocalMap/OctoMap、Machine Behavior NoMotionBackend、Mission Snapshot、Panel 和 RViz：

**现场安全警告**：当前参考 STM32 固件在上电且编码器/IMU 有效后会先自动执行 Homing；这是
预期启动阶段，不受 Orin `control_enabled` 或 PC NoMotionBackend 控制。必须先按现场安全流程
监督 Homing 完成，确认机构停止稳定，再启动后续 live shadow/规划检查。若本次不允许 Homing，
则必须关闭并泄放液压能量或硬件隔离阀输出。完整准入检查见
`../EvaluationReport/2026-07-17_unified_operator_live_shadow_and_control_gate.md`。

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch airy_excavator_bringup operator.launch.py profile:=live_shadow
```

启动前必须关闭单独运行的 `pc_runtime_bridge.py` 和 `run_perception_stack.sh`，避免重复占用状态
UDP 端口或雷达设备。该 profile 固定为 `input_source=live`、`execution_mode=shadow`、
`motion_backend=none`，不创建动作 sender，Tests 滑块关闭。当前尚无 Live Plan Action Adapter，
因此 Panel 的 Plan + Follow 会保持不可用；这不是启动故障，也不得用 fixture planner 替代。

`live_motion` 不受支持并会在 launch 展开前拒绝。选择 profile 不是运动授权。

### 拆分启动（仅诊断）

需要隔离某个节点时，仍可分别在终端启动：

```bash
# 终端1：URDF、TF、Bucket Tip
ros2 launch waji_description display.launch.py

# 终端2：空地图夹具规划（明确 map_source=fixture_empty）
ros2 launch airy_localmap fixture_planning.launch.py

# 终端3：共享执行权的 Follow + ReturnHome Shadow Executor
ros2 launch airy_mission_runtime machine_behaviors_shadow.launch.py
```

拆分诊断不再启动独立滑块；需要模拟 `/joint_states` 时使用统一 offline operator 的 `Tests`
标签页，防止两个测试发布者并存。

诊断客户端：

```bash
ros2 run airy_mission_runtime send_plan_goal dig
ros2 run airy_mission_runtime send_hold_follow_goal
ros2 run airy_mission_runtime send_return_home_goal --pose transport_home
```

验证同一 Goal 的 Plan Result 直接交给 Follow（不从预览 Topic 或 JSON 抓取轨迹）：

```bash
ros2 run airy_mission_runtime run_plan_follow_shadow dig
```

该命令只接受摘要一致、Mission provenance 一致、尚未过期且
`execution_eligible=false` 的 Trajectory Snapshot，并验证终点属于对应 Mission target。提交
Plan 前和提交 Follow 前都必须收到满足 `execution_mode=shadow`、`motion_backend=none`、
`sender_constructed=false`、`motion_authorized=false` 的 Runtime Status。Follow 仍使用
No-Motion Backend；不移动滑块时会在规划轨迹的 `tracking_timeout_s` 后以跟踪超时结束，这
属于预期的 fail-closed 行为。超时或 Ctrl-C 会取消已经接受的 Goal 并等待终态；成功、失败
或取消均必须保持 `action_datagrams=0`。

真机 `live_commissioning` 已运行且 Orin Edge Runtime ready 时，可用对应的单阶段客户端执行一次
execution-strict Follow：

```bash
ros2 run airy_mission_runtime run_plan_follow_live dig \
  --mission /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/mission/config/excavation_cycle.json
```

该入口要求 Runtime Status 为 `control/orin_edge` 且已显式授权，只接受
`execution_eligible=true` 的同一 Mission Plan Result，并在返回成功前要求
`quiescence_confirmed=true`。发送 Plan 前还会按 `airy_localmap` 安装的
`config/planning.json` 等待 LocalMap 和 bucket-tip 输入满足新鲜度门限；超时时不会创建
Plan/Follow 客户端。仅诊断其他配置时可用 `--planning-profile <path>` 覆盖。
ACT 示教前的 RL 定位通常由 `excavator-il` 的
`scripts/collect_guided_episode.py` 调用，不需要人工重复执行本命令。

`transport_home` 当前是全零关节角的 `placeholder`，仅供离线观察。点击滑块的 Reset 后，
ReturnHome 应在连续满足容差 0.3 秒后返回 `SUCCEEDED`；这不代表已经完成返回路径规划、
自碰撞检查或真机 Home 标定。

Trajectory Snapshot 的 SHA 覆盖 waypoint、时间戳、超时、输入来源和地图来源等全部行为字段。
Follow 会拒绝内容被改动但 SHA 未更新的 Goal，并在 `/mission/runtime_status` 记录
`TRAJECTORY_PROVENANCE_MISMATCH`。

## 实时轨迹 shadow

完成一次 `preview_global` 规划并保持 bucket-tip 文件桥运行后：

```bash
python3 mission/apps/run_trajectory_shadow.py
```

程序逐个新 Bucket Tip 时间戳推进 waypoint，实时打印距离并重新计算 observation 的
索引 15..26，结果写入 `mission/exports/shadow_latest/trajectory_shadow.json`。它只接受
`execution_eligible=false` 的轨迹；输入过期、frame 错误或跟踪超时会失败退出，全程
`action_datagrams=0`。

使用历史 rosbag 代替雷达时，感知栈必须显式启用 replay 时间适配，否则 LocalMap 会保留
录包时刻并被规划新鲜度门限拒绝：

```bash
RUN_RSLIDAR=0 \
REPLAY_RESTAMP_CLOUD=1 \
RUN_BUCKET_TIP_BRIDGE=1 \
RUN_TRAJECTORY_MARKERS=0 \
localmap/scripts/run_perception_stack.sh
```

`REPLAY_RESTAMP_CLOUD=1` 只允许用于离线 rosbag；真机感知必须不设置该变量，以保留传感器
源时间戳。
