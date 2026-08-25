# Resident 双策略切换

## 目标与版本

在一个常驻硬件 Owner 内切换 RL 与 ACT，避免每个阶段重新加载模型、重新初始化 CUDA 和重复抢占串口，实现低于 1 秒且可测量的策略交接。

- **V3-A 当前主线**：固定 DIG/DUMP 点与固定轨迹在 Orin 本地常驻闭环执行；
  PC 只负责 start/cancel/status/heartbeat、WebUI 和证据。
- **V2 冻结备份**：PC 仍参与在线 Plan/Follow 和分段编排，仅用于回退与论文对照。

## 组件

- `ResidentMotionCore`：authority、generation、阶段和 step budget；
- `ResidentCommandSink`：唯一 STM32 写边界、候选租约与遥测 ACK；
- `ResidentActDataLink`：owner 与 ACT Worker 的 Unix socket；
- ACT Worker：模型/CUDA 常驻、相机观测和最新帧推理，不打开串口；
- Resident control：`activate_rl`、`activate_act(max_steps)`、`terminal_disarm`；
- `ResidentFixedCycleCoordinator`：在 Orin 内推进固定轨迹、RL/ACT 切换与多铲循环；
- PC V3-A Adapter：只运行启动、取消、状态轮询、400 ms heartbeat 和证据封存。
- PC `live_shadow` RViz：读取 Orin 状态与严格 V3-A status 中的活动三点轨迹，
  写入原子只读轨迹文件并发布 Marker；不具备规划或运动控制权。

## 交接不变量

1. 旧 source 的终态零必须经 STM32 telemetry ACK；
2. 新 source 取得新的 `control_generation`；
3. 旧/未来 generation candidate 只能被动拒绝，不能撤销当前 authority；
4. 目标第一条非零命令要有序号/模式 ACK；
5. candidate/mission lease 过期、Worker 断开或安全状态失效立即归零；
6. 一个策略 Active 时，外部 Behavior 不能越权抢占。

## V3-A 当前一铲

```text
Orin FOLLOW_DIG
→ Orin ACT_DIG（130 个有效 10 Hz step）
→ Orin FOLLOW_DUMP
→ Orin EXECUTE_DUMP
→ 若还有本轮铲数，直接 Orin FOLLOW_DIG 到下一挖点
```

自动多铲按 `dig_01 → dig_02 → dig_03` 循环。V3-A 当前只提供一次启动的
自动单铲/多铲闭环；V2 的 PC 分阶段按钮已不是主线入口。

V3-A 不再依赖 PC 在阶段之间启动规划进程或通过 Behavior RPC 下发新轨迹。
所有固定轨迹均来自 Orin 上经严格验证的 V3-A plan；短期不包含自动挖点选择。

V3-A 固定 Follow 使用两级 waypoint 到达容差：生成起点与最终终点
保持 `0.25 m`，只有 Orin 本地插值的中间点使用 `0.40 m`。这使轨迹
控制器在保留 DIG/DUMP 终点精度的同时，不再为精确追逐共线中点产生
不必要停顿。旧 Follow 快照没有新字段时自动退化为单一容差，V2 不受影响。

V3-A 活动主链路不使用 RRT*。每段 Follow 启动时，Orin 把实时铲尖
和固定终点扩展成三点预瞧：实时起点、回转圆弧中点、固定终点。
圆弧中点在 `machine_root_ros` 下使用最短回转角、水平半径和高度
分别插值，代替原先不符合回转机构的 Cartesian 直线中点。

WebUI 启动自动 Mission 前会先启动 PC `live_shadow` RViz。Follow 活动时，
Orin 把控制器实际使用的起点、圆弧中点、终点、当前 waypoint 序号和当前容差
作为只读状态回传；PC 仅将其显示为红色轨迹和黄色 waypoint。进入 ACT、固定
倾倒或终态时轨迹会清除，避免把上一段旧轨迹误认为当前指令。

## 时延证据

权威 handoff latency 定义为“旧 source 终态零 STM32 ACK → 新 source 第一条非零 STM32 ACK”。模型加载和 PC 规划等待应单独报告。Experiment Run 记录 generation、方向、序号和时间戳，不能只截图一个可覆盖的 latest 值。
