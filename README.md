# ExcavatorRuntime

> [!WARNING]
> 本 README 第 0 节保留的是早期 `192.168.0.*` PC 主控架构说明，仅用于历史追溯，
> 不可作为当前现场启动手册。当前 V3-B/ICRA 实验分支以 Orin Resident Mission 为运动权威，
> PC/Orin 直连地址为 `192.168.50.1/192.168.50.2`。请只按
> [笔记/相关命令.md](笔记/相关命令.md) 中经过当前配置核对的入口操作；旧命令不得直接用于真机。

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

## 0. 强化学习 Orin + PC 真机测试速查

当前测试链路：

```text
STM32 Machine State
  → Orin 本地 FK / 38D Observation / RL ONNX / waypoint 推进
  → Orin 本地 Action Relay
  → STM32

PC 雷达 / LocalMap / Dig-Dump Target / Bucket Tip 规划
  → 低频 Trajectory Snapshot
  → Orin
```

当前现场网络和串口：

```text
PC:   192.168.0.220
Orin: 192.168.0.55
STM32 serial on Orin: /dev/ttyTHS1
Behavior RPC: TCP 18083
```

不要在以下命令前设置 `ROS_DOMAIN_ID`。启动顺序固定为：

```text
1. STM32 上电并完成 Homing
2. Orin Runtime
3. PC Operator
4. 检查状态
5. 在 RViz Panel 点击 RL Follow
```

### 0.1 Orin 终端：启动端侧 RL Follow

确认 `deploy/edge_runtime.remote.json` 中：

```json
{
  "mode": "remote_control",
  "remote_behavior": {
    "bind_port": 18083,
    "allowed_client_host": "192.168.0.220"
  }
}
```

然后启动：

```bash
cd ~/workspace_/excavator-orin-runtime
conda activate excavator-orin

mkdir -p deploy/logs
test -f deploy/edge_runtime.remote.json || \
  cp deploy/edge_runtime.remote.example.json deploy/edge_runtime.remote.json
python -m json.tool deploy/edge_runtime.remote.json >/dev/null

run_tag=$(date +%Y%m%d_%H%M%S)

python orin_state_sender.py \
  --serial-port /dev/ttyTHS1 \
  --control-enabled \
  --pc-host 192.168.0.220 \
  --edge-config deploy/edge_runtime.remote.json \
  --edge-motion-authorization ALLOW_EDGE_MACHINE_MOTION \
  --print-every 100 \
  2>&1 | tee "deploy/logs/rl_follow_${run_tag}_stdout.log"
```

预期启动日志包含：

```text
REMOTE EDGE CONTROL ARMED IDLE
behavior RPC 0.0.0.0:18083 from 192.168.0.220
```

同一台 Orin 上只能运行一个 `orin_state_sender.py`，因为它独占 STM32 串口。重启前检查：

```bash
pgrep -af orin_state_sender.py
```

### 0.2 PC 终端：启动感知、规划、Panel 和 RViz

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

ros2 launch airy_excavator_bringup operator.launch.py \
  profile:=live_commissioning \
  motion_authorization:=ALLOW_LIVE_MACHINE_MOTION \
  orin_host:=192.168.0.55 \
  orin_port:=18083
```

该命令已经统一启动：

- Orin Machine State → `/joint_states` 的 PC 状态桥；
- FK、RobotModel 和 Bucket Tip 显示；
- 雷达、LocalMap 和 OctoMap；
- Dig/Dump Target 和 Bucket Tip 规划器；
- Orin Edge Gateway；
- Mission Actions、RViz Panel 和 RViz。

不需要再分别启动 FK、感知栈、规划器或另一个 RViz。

### 0.3 PC 第二终端：运动前检查

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

ping -c 3 192.168.0.55
ros2 topic hz /joint_states
ros2 topic echo /bucket_tip_pose_machine_root_ros --once
ros2 topic echo /mission/runtime_status --once
```

RViz Panel 顶部必须显示：

```text
LIVE / COMMISSIONING / READY
```

运行状态应包含：

```text
motion_backend: orin_edge
motion_gate_reason: ready
```

如果不是 `READY`，不要反复点击按钮；先根据 Panel 的 gate reason 和 Logs 标签定位问题。

### 0.4 只测试强化学习 Follow

在 RViz Panel → `Actions` 中：

1. 点击 `Plan + Follow DIG`：PC 规划到 Dig 点，Orin 使用 RL ONNX 实时跟踪；
2. 等待 Result 完成且按钮恢复；
3. 点击 `Plan + Follow DUMP`：PC 规划到 Dump 点，Orin 使用同一个 RL ONNX 实时跟踪。

`ExecuteDig` 和 `ExecuteDump` 是 Orin 固定动作，不属于 RL Follow 测试。一次只执行一个行为。
`Cancel Panel Operation` 只取消当前软件行为，**不是急停**；现场异常使用真机急停。

### 0.5 测试后保留日志

Orin：

```bash
ls -lh deploy/logs/rl_follow_${run_tag}_stdout.log deploy/logs/edge_runtime.jsonl
python -m edge_runtime.audit deploy/logs/edge_runtime.jsonl
```

PC 的 ROS launch 日志自动保存在：

```text
~/.ros/log/latest/
```

出现问题时至少保留：Orin stdout、`edge_runtime.jsonl`、PC `~/.ros/log/latest/`、测试目标、
现场视频和大致发生时间。

### 0.6 RL Follow 到 ACT 采集的交接信息

ACT 数据的初始姿态应覆盖 RL Follow 到达 Dig 目标后的终止分布。现有链路可直接提供以下证据：

- 权威 Dig 目标：`mission/config/excavation_cycle.json` 的 `targets.dig`，坐标系为
  `machine_root_ros`，单位为米；
- Follow 运行反馈：`/excavator/follow` 的 Feedback 包含实时 `bucket_tip`、
  `current_waypoint_index` 和 `distance_m`；Result 包含 `final_waypoint_index`、
  `final_distance_m` 与 `quiescence_confirmed`，Panel 的 Action 结果会显示终态；
- 最新机器状态：`runtime_bridge/exports/latest_state.json`，包含四关节位置/速度与安全状态；
- 最新铲尖位置：ROS topic `/bucket_tip_pose_machine_root_ros`，同时由 bridge 写入
  `localmap/exports/live_latest/bucket_tip.live.json`。

Follow 完成且 Result 为成功后，可在 PC Operator 仍运行时检查：

```bash
python3 -m json.tool mission/config/excavation_cycle.json
python3 -m json.tool runtime_bridge/exports/latest_state.json
python3 -m json.tool localmap/exports/live_latest/bucket_tip.live.json
ros2 topic echo /bucket_tip_pose_machine_root_ros --once
```

进入 ACT 采集前必须先停止 Orin 的 RL Runtime，确认它已经释放 `/dev/ttyTHS1`，再启动
`excavator-il` Collector；切换进程期间不要改变机构姿态。此时直接采集即可继承真实 RL 终止姿态。
若需要人工覆盖终点附近的扰动分布，使用 `excavator-il` 引导脚本的可选“预定位”阶段；预定位
发生在正式 Episode 创建前，不进入 ACT 训练数据。不要把一个固定关节角复制到全部 Episode，
而应保留 Follow 实际终点误差、铲尖位置和关节姿态的合理变化。

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

`live_commissioning` 是唯一真机 Operator profile，也是经过现场验证的最终运行入口。
旧 `live_production`、`live_control` 和 PC `udp_policy` 执行路线均已删除，不提供别名或兼容回退。
规划产生完整 Trajectory Snapshot，Orin 独占 ONNX/fixed action 高频闭环与 STM32 Command Sink。

不要在这些命令前设置 DDS Domain 环境变量。当前仓库保留两类证据状态：

- `mission/config/excavation_cycle.json` 当前保存本轮现场 RViz 核对后的 `rviz_adjusted` dig/dump
  坐标，只允许 `live_commissioning` 使用；移动挖机、雷达或作业区后必须重新核对并更新状态；
- Orin 本地 fixed action profile 及其 digest 由 `excavator-orin-runtime` 管理，PC 只发送行为名称。

`placeholder` 会锁定 Follow；当前仓库基线已经由操作者升级为 `rviz_adjusted`。当前端侧
`live_commissioning` 开放 `Plan + Follow DIG/DUMP` 和独立
`ExecuteDig/ExecuteDump`。固定动作与 Follow 互斥，取消、失败、超时和完成均由 Orin
先发送终态零命令。`ReturnHome` 要等对应 Orin 行为接口迁移后再启用。

### V3-B 目录驱动的多点现场显示与闭环

`mission/config/excavation_dig_point_catalog.v1.json` 是固定挖掘点的唯一入口。点位按
`dig_points` 对象顺序排列，并由 `dig_groups` 中的点集选择循环范围：

```text
Follow DIG → ExecuteDig → Follow DUMP → ExecuteDump
```

每个 point key 必须唯一；全部坐标均为 `machine_root_ros` 右手系米制坐标。V3-B WebUI
从目录读取当前点数、点集和起始点并提交给 Orin 常驻闭环；
运行期间点集冻结。PC Operator 只读发布 `/mission/target_markers`，近端为橙色、远端为蓝色，
并继续用 `/localmap/planned_bucket_tip_markers` 显示 Orin 当前实际三点轨迹。修改目录后需要
同步 PC/Orin 对应部署并重启 UI/RViz，不保留旧三点运行回退。

Panel → Tests 只保留离线 JointState 滑块；它在 live profile 中被禁用，不发送真机动作。
ONNX 的四轴 `[-1,1]` 输出、38 维 Observation、waypoint 推进和 fixed action 都由 Orin
在本地执行并记录。PC 不缩放、不取反，也不发送连续 Physical Velocity Command。
Panel 顶部以 `LIVE / COMMISSIONING / READY` 表示唯一真机路线已经就绪。

当前端侧 commissioning 既可分段验证 `Plan + Follow DIG`、`ExecuteDig`、
`Plan + Follow DUMP` 和 `ExecuteDump`，也可使用上述多点演示命令调用同一组 Action。
Panel 的 `Full Mission` 与分段按钮调用同一组 Orin Edge Action。
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

终端 1，启动 PC 只读状态桥：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --publish-joint-states
```

终端 2，启动 mock Orin relay：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/mock_orin_relay.py
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

真实 Orin 下不需要另启这个脚本；`operator.launch.py profile:=live_commissioning`
已经启动接收状态、发布 `/joint_states` 的 PC bridge，以及唯一的 Orin Edge Action Gateway。
PC bridge 是严格只读的：不加载 ONNX、不生成动作、不打开到 Orin 18082 的 UDP sender。

ONNX Follow、fixed action、动作日志和 STM32 发送均由
`excavator-orin-runtime` 管理。PC 中保留的 CSV 导出工具只用于读取历史日志或生成实验产物，
不属于活动控制路线。

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
