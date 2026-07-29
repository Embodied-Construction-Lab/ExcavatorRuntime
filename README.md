# ExcavatorRuntime

ExcavatorRuntime 是缩比挖掘机真机侧的感知、局部地图、bucket-tip 规划实验工程。

当前只做：

- RoboSense Airy 雷达接入
- 点云从 `rslidar` 转到 `machine_root`
- 实时 `LocalMap` / OctoMap
- 简单 bucket-tip RRT 避障轨迹
- FK bucket tip 坐标桥接
- RViz 可视化

默认 fixture/live shadow 不做：

- 不发送 PWM
- 不发送 UDP 真机控制命令；只有显式 live motion profile 才构造唯一 Command Sink
- 不修改 Unity 旧场景或旧 prefab
- 不修改 ONNX observation 维度

## 目录

```text
runtime/                 # 当前雷达运行配置
ros2_ws/                 # ROS2 overlay workspace
rslidar_sdk/             # Airy 版 RoboSense SDK 源码，只读使用
kinematics/              # 挖掘机 FK / TF，输入关节角，输出 bucket tip
localmap/                # 感知、LocalMap、OctoMap、RRT、RViz marker 工具
runtime_bridge/          # PC 与 Orin/STM32 的 UDP 状态/动作中转协议
rviz/                    # RViz 配置
docs/                    # 雷达接线、端口、防火墙等运行笔记
```

## 1. 准备环境

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
```

首次或源码变化后编译 overlay：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## 1.1 统一 RViz Operator（推荐入口）

离线验证只需一个终端，启动 FK、fixture planner、shadow Actions、Mission、Panel 和 RViz：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch airy_excavator_bringup operator.launch.py profile:=fixture_shadow
```

连接 Orin 和雷达但绝不发送动作时使用：

```bash
ros2 launch airy_excavator_bringup operator.launch.py profile:=live_shadow
```

当前 `live_commissioning` 已切换到端侧 Follow 和固定动作：

```text
PC 感知/目标/规划 → 完整 Trajectory Snapshot → Orin FK/38D/ONNX/跟踪
                                               → loopback Action Relay → STM32
PC ExecuteDig/ExecuteDump → 动作名称 → Orin 本地固定动作闭环
                                      → loopback Action Relay → STM32
```

PC 仍运行 RViz Panel、雷达感知和规划，但不会构造物理速度 UDP sender，也不会加载 ONNX。
先启动 Orin 的 `remote_control`，再在 PC 启动：

```bash
ros2 launch airy_excavator_bringup operator.launch.py \
  profile:=live_commissioning \
  motion_authorization:=ALLOW_LIVE_MACHINE_MOTION \
  orin_host:=192.168.0.55 \
  orin_port:=18083
```

Gateway 只接受现有 `/planning/plan` 输出的完整、不可变且 SHA 匹配的
`TrajectorySnapshot`。Orin 启动后保持 Idle；Panel 点击 `Plan + Follow DIG/DUMP` 后才在 Orin
创建新的 Follow runtime。取消、TCP 断开、状态拒绝、完成、超时、异常和进程退出均由 Orin
先经本地 Action Relay 提交零命令，再返回 Result。

Panel Ready 的权威状态来自 Orin 5 Hz `status`，不是 PC 猜测。预期显示：

```text
motion_backend=orin_edge
follow_control_mode=edge_onnx
motion_gate_reason=ready
```

尚未完成现场标定的可达域在 commissioning 中继续使用
`disabled_by_operator` provenance，不阻断本阶段全局 Bucket Tip 规划。远程接口已迁移
`Follow`、`ExecuteDig` 和 `ExecuteDump`；固定动作只发送行为名称，速度闭环位于 Orin。
`ReturnHome` 仍未迁移。`Full Mission` 已由 PC 调度现有的端侧
`Follow`、`ExecuteDig` 和 `ExecuteDump`，不新增动作发送链路。

所有几何、目标和可达域证据完成后，改用严格入口：

```bash
ros2 launch airy_excavator_bringup operator.launch.py \
  profile:=live_production \
  motion_authorization:=ALLOW_LIVE_MACHINE_MOTION
```

`live_production` 会重新强制执行器位置范围、`field_validated` Mission target 和
`field_validated` execution workspace。它目前仍是旧 PC Command Sink 路径，仅作为未完成迁移的
历史入口；端侧 Follow 联调只能使用 `live_commissioning`，两种 control profile 禁止同时启动。
旧 `live_control` profile 已删除，避免同一名称同时代表联调与生产。

不要在这些命令前设置 DDS Domain 环境变量。当前仓库保留两类证据状态：

- `mission/config/excavation_cycle.json` 当前保存本轮现场 RViz 核对后的 `rviz_adjusted` dig/dump
  坐标，只允许 `live_commissioning` 使用；移动挖机、雷达或作业区后必须重新核对并更新状态；
- `runtime_bridge/config/fixed_actions.json` 仍是 `candidate`，且由 runtime config 固定整文件 SHA；
  没有完整现场证据时不能成为 `field_validated`。

`placeholder` 会锁定 Follow；当前仓库基线已经由操作者升级为 `rviz_adjusted`，因此会解锁
commissioning Follow，但 production 仍只接受 `field_validated`。当前端侧
`live_commissioning` 开放 `Plan + Follow DIG/DUMP` 和独立
`ExecuteDig/ExecuteDump`。固定动作与 Follow 互斥，取消、失败、超时和完成均由 Orin
先发送终态零命令。`ReturnHome` 要等对应 Orin 行为接口迁移后再启用。

### 多挖掘点现场演示

`mission/config/excavation_demo.json` 是演示程序的唯一入口。`dig_points` 按文件顺序执行，
每个点都完成以下完整循环后才进入下一个点：

```text
Follow DIG → ExecuteDig → Follow DUMP → ExecuteDump
```

每个 `point_id` 必须唯一；全部坐标均为 `machine_root_ros` 右手系米制坐标。默认文件已经配置
`dig_01`、`dig_02`、`dig_03` 三个现场演示点。调整或增加点位时，在 `dig_points` 数组中修改或
复制对象，并更新 `point_id` 与 `position_m`。公共倾倒点只需修改 `dump_target`。live Operator
会在 `Mission Targets` 中显示全部 dig 点和公共 dump 点；橙色标签包含各自的 `point_id`。

演示程序由实时规划器在 PC Operator 启动时一次性加载。修改坐标后不需要同步到 Orin，也不需要
重新编译，但必须重启 PC 的 `operator.launch.py`，使规划器与客户端读取同一文件 SHA。然后在
第二个 PC 终端运行：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

ros2 run airy_mission_runtime run_excavation_demo \
  --program /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/mission/config/excavation_demo.json \
  --repeat 1
```

`--repeat 2` 表示把整个点位列表重复两轮。脚本每次只提交一个 `/mission/run_cycle` Goal，
等待 `FollowDig → Dig → FollowDump → Dump` 返回成功且确认静止后再提交下一个；任一步拒绝、
失败、超时或取消都会停止演示，不会跳过失败点继续运动。

旧 PC `udp_policy` backend 的 Panel → Tests 中提供 boom/stick/bucket 三轴
`Action − / Action +` Hold-to-Jog：按住才发送单轴命令，松开或 RViz 失焦立即取消；
它不是当前 Orin Edge commissioning 路径的一部分。Panel 心跳丢失、Machine State 过期、
安全状态关闭或达到最长按住时间时，旧 PC Command Sink 自动发送终态零命令。
速度比例、周期、心跳和最长按住时间只从 `runtime_bridge/config/runtime.json` 的
`manual_jog` section 读取。当前 live 配置把单次硬上限固定为 `1000 ms`；Panel 会直接
显示服务端的精确拒绝原因，并在 Result 中保留拉线长度 `before / after / delta`。Swing 因现场
速度/限位尚未验证，不在该诊断入口开放。

旧 PC Follow supervision 的参数只来自 `runtime.json` 的 `follow_control`：在
`live_production` 中 Panel 点击 `Plan + Follow`
后会在操作期间自动维持 175 ms 租约心跳，操作结束后恢复按钮。ONNX 的四轴 `[-1,1]` 输出不做
轴屏蔽或符号变换，只按 `machine_profile.json` 的正/负速度幅值转换为 m/s、rad/s；STM32 负责真机
低层方向适配。Command Sink 仍以保持 ONNX 符号的完整方向性物理包络二次校验。Follow 持续到轨迹
到达、轨迹自身 timeout、操作者取消、安全状态关闭或监督心跳
丢失；不再使用与任务无关的一秒总时长上限。Machine State 新鲜度门限为配置化的 500 ms，以容纳
现场 8–10 Hz 状态流的正常调度抖动；Orin 对每条动作仍执行独立 100 ms lease。每次策略决策记录到
`runtime_bridge/exports/action_journal/follow_canary/`，最新 38 维观测、原始/应用后动作和物理命令写入
`runtime_bridge/exports/latest_observation.json`。Panel 顶部会明确显示
`LIVE / COMMISSIONING / READY` 或 `LIVE / PRODUCTION / READY`，避免把联调状态误认为生产准入。

当前端侧 commissioning 既可分段验证 `Plan + Follow DIG`、`ExecuteDig`、
`Plan + Follow DUMP` 和 `ExecuteDump`，也可使用上述多点演示命令调用同一组 Action。
Panel 的 `Full Mission` 按钮暂未作为 commissioning 入口开放。
实现与离线证据见
`EvaluationReport/2026-07-26_pc_plan_orin_edge_follow_implementation.md`。

如果 OctoMap 未安装：

```bash
sudo apt install -y ros-jazzy-octomap-server ros-jazzy-octomap-msgs ros-jazzy-octomap-ros
```

## 2. 配置雷达网口

当前 Airy 雷达配置：

```text
雷达 IP: 192.168.1.200
上位机 IP: 192.168.1.103
MSOP: 6700
DIFOP: 7789
IMU: 6688
```

临时配置网口：

```bash
sudo ip link set enp130s0 up
sudo ip addr flush dev enp130s0
sudo ip addr add 192.168.1.103/24 dev enp130s0
ip -brief addr show enp130s0
ping -I 192.168.1.103 -c 3 192.168.1.200
```

如果防火墙拦截 UDP：

```bash
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6700
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6688
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 7789
```

## 3. 启动感知栈

一个终端启动：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/ExcavatorRuntime
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

OCTOMAP_RESET_INTERVAL_S=1.0 \
OCTOMAP_RESOLUTION=0.05 \
OCTOMAP_MAX_RANGE=4.0 \
OCTOMAP_POINT_CLOUD_MIN_X=-1.5 \
OCTOMAP_POINT_CLOUD_MAX_X=3.0 \
OCTOMAP_POINT_CLOUD_MIN_Y=-0.42 \
OCTOMAP_POINT_CLOUD_MAX_Y=1.00 \
OCTOMAP_POINT_CLOUD_MIN_Z=-0.5 \
OCTOMAP_POINT_CLOUD_MAX_Z=4.0 \
localmap/scripts/run_perception_stack.sh
```

这个脚本会启动：

```text
rslidar_sdk                         -> /rslidar_points, /rslidar_imu_data
transform_live_cloud_to_base.py     -> /localmap/machine_root_points
run_live_local_map_node.py          -> localmap/exports/live_latest/local_map.live.json
octomap_server_node                 -> /occupied_cells_vis_array
publish_reachable_workspace_markers -> /localmap/reachable_workspace_markers
```

常用检查：

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /localmap/machine_root_points
ros2 topic hz /occupied_cells_vis_array
python3 -m json.tool localmap/exports/live_latest/local_map.live.json | sed -n '1,60p'
```

基础链路与产物结构检查：

```bash
localmap/scripts/run_smoke_check.sh
```

如果还要按 `planning.json` 的时效规则验证当前输入确实可规划：

```bash
localmap/scripts/run_smoke_check.sh \
  --run-planning-phase dig \
  --mission mission/config/excavation_cycle.json
```

## 4. 打开 RViz

另一个终端：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh
rviz2 -d rviz/airy_points.rviz
```

主要看这些 display：

```text
/localmap/machine_root_ros_points          # machine_root_ros 下的实时点云
/occupied_cells_vis_array                  # OctoMap 占据栅格
/localmap/reachable_workspace_markers      # bucket tip 可达区域
/localmap/preview_bucket_tip_markers       # 非执行全局预览轨迹
/mission/target_markers                    # Mission dig/dump 目标
Axes: machine_root_ros                     # 坐标原点和方向
```

## 5. 可选：启动 bucket tip FK

如果只看雷达和 OctoMap，可以跳过本节。

启动 FK / TF：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

ros2 launch waji_description display.launch.py
```

FK 的输入是关节角，不是油缸位移：

```text
/joint_states: sensor_msgs/msg/JointState
swing_joint, boom_joint, arm_joint, bucket_joint
单位: rad
```

测试输入：

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'],
  position: [0.0, 0.3, -0.8, 0.5]
}" -r 10
```

单独启动 bucket tip bridge：

```bash
python3 localmap/apps/planning/bridge_bucket_tip_from_tf.py \
  --input-topic /bucket_tip_pose_machine_root_ros \
  --output-topic /localmap/bucket_tip_machine_root_ros_pose \
  --bridge localmap/config/bucket_tip_tf_bridge.machine_root_ros.identity.v1.json \
  --output-json localmap/exports/live_latest/bucket_tip.live.json
```

或者在感知栈里一起启动：

```bash
RUN_BUCKET_TIP_BRIDGE=1 localmap/scripts/run_perception_stack.sh
```

检查 bucket tip：

```bash
ros2 topic echo /bucket_tip_pose_machine_root_ros --once
ros2 topic echo /localmap/bucket_tip_machine_root_ros_pose --once
python3 -m json.tool localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

## 6. 跑一次规划

感知栈运行后，在新终端执行一次规划：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

localmap/scripts/run_planning_once.sh \
  --mission mission/config/excavation_cycle.json \
  --phase dig \
  --planning-scope preview_global
```

挖掘点和倾倒点只从 Mission 文件读取，`--phase` 唯一推导任务模式。规划算法参数保存在
`localmap/config/planning.json`；共享的frame、live路径、OctoMap topic和bounds由它引用的
`localmap/config/perception.json` 派生，现场运行不再通过环境变量逐项覆盖。倾倒点预览使用：

```bash
localmap/scripts/run_planning_once.sh \
  --mission mission/config/excavation_cycle.json \
  --phase dump \
  --planning-scope preview_global
```

只验证 live 输入和展示内部步骤、不发布规划产物：

```bash
localmap/scripts/run_planning_once.sh \
  --mission mission/config/excavation_cycle.json \
  --phase dig \
  --planning-scope preview_global \
  --dry-run
```

输出：

```text
localmap/exports/live_preview/local_map.octomap_obstacles.json
localmap/exports/live_preview/rrt_star_request.octomap_obstacles.json
localmap/exports/live_preview/trajectory_command.preview_global.json
```

这些产物均为 `execution_eligible=false`，不生成 observation slice，也不发送动作。

发布轨迹到 RViz：

```bash
python3 localmap/scripts/publish_trajectory_markers.py \
  --trajectory localmap/exports/live_preview/trajectory_command.preview_global.json \
  --topic /localmap/preview_bucket_tip_markers
```

只检查本地 JSON 产物，不检查 ROS topic：

```bash
python3 localmap/apps/diagnostics/run_smoke_check.py --skip-ros
```

### 不连接雷达：历史 rosbag 回放

历史 bag 的 header 是录制时刻。离线感知栈必须显式重打消息 header，不能放宽规划的实时
新鲜度门限：

```bash
RUN_RSLIDAR=0 \
REPLAY_RESTAMP_CLOUD=1 \
RUN_BUCKET_TIP_BRIDGE=1 \
RUN_TRAJECTORY_MARKERS=0 \
localmap/scripts/run_perception_stack.sh
```

另一个终端循环回放：

```bash
ros2 bag play bags/airy_repositioned_20260707_152018 --loop
```

`REPLAY_RESTAMP_CLOUD=1` 仅用于离线 bag；连接真机雷达时必须省略。

## 7. 可选：本机模拟 Orin 中转

终端 1，启动 PC 通信诊断入口：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --reply-zero
```

终端 2，启动 mock Orin relay：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/mock_orin_relay.py
```

如果要把 Orin 状态发布成 ROS2 `/joint_states`，PC 侧加：

```bash
--publish-joint-states
```

现场只接收状态并发布 `/joint_states`，同时每 100 个有效状态包打印一次：

```bash
/usr/bin/python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --publish-joint-states \
  --print-every 100
```

`--print-every 10` 表示每 10 包打印一次，`--print-every 0` 关闭周期打印；不传时使用
`runtime_bridge/config/runtime.json` 的 `diagnostics.print_every`。该参数不改变接收、记录或
`/joint_states` 发布频率。

连接真实 Orin 时，PC 侧使用：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py --reply-zero
```

这一步只回发零动作，用于验证 Orin -> PC 状态包和 PC -> Orin 动作包通道。网络、动作有效期和日志采样均读取运行配置；不加 `--reply-zero` 时只接收和记录状态。

启动 ONNX policy bridge：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

.venv_runtime/bin/python runtime_bridge/apps/pc_policy_bridge.py
```

网络、模型、机型 profile、waypoint 产物、超时和日志采样统一读取 `runtime_bridge/config/runtime.json`，现场启动不再重复输入。任务切换使用 `--task-mode CarryMaterial`；使用其他部署配置时只需增加 `--config <path>`。

它会接收 Orin `machine_state_v1`，发布 `/joint_states` 给 FK，读取 `/bucket_tip_observation`，组装 38 维 observation 并运行 ONNX。该脚本是只读诊断入口，已经移除 `--enable-motion` 和 UDP sender，只记录/打印未发送的候选动作。真机动作只能通过统一 RViz Operator 的 Action Server 和唯一 Command Sink 发送。
ONNX 输出在 PC 内部按训练语义视为 `[-1, 1]` 策略动作，候选动作会按 `shared/machine_profile.json` 反归一化为物理速度。其中 `boom/stick/bucket` 单位 m/s，`swing` 单位 rad/s。
反归一化只选择动作正负方向对应的速度幅值，不改变四轴符号；低层方向适配由 STM32 负责。
默认安全判定仍会计算：如果 `estop=true`、`sensor_valid=false`、`stm32_alive=false`、`control_enabled=false` 或存在 `fault_flags`，记录的候选动作会变为零，但任何情况下都不会由该诊断脚本发送。

每个真正成功发往 Orin 的 UDP 动作都会异步追加到本地会话日志：

```text
runtime_bridge/exports/action_journal/<发送入口>.<UTC启动时间>.<PID>.partNNNN.jsonl
```

每条记录包含 PC 记录时间、发送入口、Orin 目标地址、可读 packet、精确 payload 的 Base64、字节数和 SHA-256。日志写盘在线程中完成，不把磁盘延迟加入控制循环；程序启动时会打印本次日志路径。队列满或写盘失败后，下一帧动作会在 `sendto` 之前被拒绝，进程返回错误，不允许长期“继续控制但停止记录”。

容量策略统一位于 `runtime_bridge/config/runtime.json` 的 `action_journal` section。正式配置按每个文件 64 MiB、保留 16 个文件轮转，总量约 1 GiB，超过后删除最旧记录；mock 配置单独写入 `runtime_bridge/exports/action_journal_mock/`。`pc_policy_bridge.py` 不发送动作；`pc_runtime_bridge.py` 只有显式增加诊断参数 `--reply-zero` 时才发送零动作并生成发送日志。

到达挖掘点或倾倒点后，可以临时用固定动作脚本执行挖掘/倾倒。该脚本不做路径规划，后续由外部 planner 决定何时启动：

```bash
python3 runtime_bridge/apps/fixed_action_player.py dig
```

倾倒动作：

```bash
python3 runtime_bridge/apps/fixed_action_player.py dump
```

固定动作步骤、控制增益、阈值和超时统一位于版本化的
`runtime_bridge/config/fixed_actions.json`，并绑定 machine ID、动作顺序、当前 machine profile SHA、
当前 URDF SHA 和整份动作文件 SHA。配置 schema 为 `runtime_bridge_config_v10`；旧配置会明确拒绝。
上述独立脚本只计算和打印，已经彻底移除 `--enable-motion` 和 UDP sender；真机固定动作只能通过
统一 RViz Operator 的 ExecuteDig/ExecuteDump Action 进入唯一 Command Sink。commissioning 允许
candidate 动作通过这两个独立按钮执行；production 和 Full Mission 仍要求现场证据完整且动作
profile 为 `field_validated`；多点演示命令在 commissioning 中复用已开放的 candidate
固定动作。当前 `fixed_action_profile.v2` 使用绝对归一化执行器目标，
不再把 Follow 的结束误差叠加到固定动作上。Dig 根据现场液压系统更适合同步驱动的表现合并为
两段：第一段将 boom/stick/bucket 送到 `+0.60/-0.15/+0.90`，第二段将三轴送到
`-0.35/+0.15/-0.85`。Dump 将 bucket 依次送到 `+0.90/-0.85`。
配置中未列出的轴在该段输出零命令；
单段达到 tolerance 后进入下一段，超时则以零命令失败退出。
Dig/Dump 伺服在 Orin 本地使用恒定 `0.6` 归一化速度，并与 ONNX 动作使用同一物理速度转换：
Orin 只按动作正负选择对应物理速度幅值并保持 Unity 符号；PC 和 Orin 都不保存执行器方向取反
属性，低层方向适配由 STM32 负责。

每次只执行一个 ExecuteDig 或 ExecuteDump，等待动作结束和至少 1 秒日志落盘后，可把实际
PC→Orin Action Journal 的最近一次运动会话导出为 Orin/STM32 回放 CSV：

```bash
JOURNAL="$(find runtime_bridge/exports/action_journal -maxdepth 1 -name '*.jsonl' \
  -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"

python3 runtime_bridge/apps/export_orin_policy_log_to_open_loop_csv.py \
  --input "$JOURNAL" \
  --input-format pc-journal \
  --latest-session \
  --output ../EvaluationReport/captures/execute_dig.pc_to_orin.csv \
  --phase ExecuteDig \
  --mode FixedAction
```

导出 Dump 时只需把输出文件和 `--phase` 改为 `execute_dump...csv` 与 `ExecuteDump`。工具按超过
1500 ms 的命令间隔划分会话，只选择最后一个包含非零动作的会话，并额外附加终态零命令；因此
应在每次按钮执行后立即单独导出，不要连续点击两个动作后再导出。
现场报告必须位于工作区 `EvaluationReport/`，并逐行记录 `fixed_action_profile_id`、
`fixed_action_contract_sha256` 和每个 `experiment_run_id`；加载器同时核对报告 SHA 与动作契约，
因此不能用无关报告或只改 `validation_status` 解锁。

首次使用前需要当前 Python 环境安装 ONNX Runtime：

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv_runtime
uv pip install --python .venv_runtime/bin/python onnxruntime
```

协议说明见：

```text
docs/runtime_bridge_protocol.md
```

## 8. 离线 bag

录一段雷达数据：

```bash
mkdir -p bags
timeout --signal=INT 20s ros2 bag record \
  -o bags/airy_$(date +%Y%m%d_%H%M%S) \
  /rslidar_points \
  /rslidar_imu_data
```

检查 bag：

```bash
python3 localmap/scripts/inspect_bag_points.py \
  bags/<bag_name> \
  --frames 3
```

## 关键坐标系

```text
rslidar       # 原始雷达坐标
machine_root  # 统一感知和规划坐标，对齐 Unity MachineRoot
```

当前外参：

```text
localmap/config/extrinsics_rslidar_to_machine_root.measured.json
```

bucket tip 实时 JSON：

```text
localmap/exports/live_latest/bucket_tip.machine_root.live.json
```

更详细的雷达端口、防火墙、DIFOP、RViz 说明见：

```text
docs/lidar_runtime_notes.md
localmap/README.md
kinematics/README.md
```
