# Resident 双策略切换

## 目标

在一个常驻硬件 Owner 内切换 RL 与 ACT，避免每个阶段重新加载模型、重新初始化 CUDA 和重复抢占串口，实现低于 1 秒且可测量的策略交接。

## 组件

- `ResidentMotionCore`：authority、generation、阶段和 step budget；
- `ResidentCommandSink`：唯一 STM32 写边界、候选租约与遥测 ACK；
- `ResidentActDataLink`：owner 与 ACT Worker 的 Unix socket；
- ACT Worker：模型/CUDA 常驻、相机观测和最新帧推理，不打开串口；
- Resident control：`activate_rl`、`activate_act(max_steps)`、`terminal_disarm`；
- PC Hybrid Mission：RL 到点、ACT 挖掘、RL 倾倒、RL 返回和多铲循环。

## 交接不变量

1. 旧 source 的终态零必须经 STM32 telemetry ACK；
2. 新 source 取得新的 `control_generation`；
3. 旧/未来 generation candidate 只能被动拒绝，不能撤销当前 authority；
4. 目标第一条非零命令要有序号/模式 ACK；
5. candidate/mission lease 过期、Worker 断开或安全状态失效立即归零；
6. 一个策略 Active 时，外部 Behavior 不能越权抢占。

## 当前一铲

```text
RL Follow DIG
→ ACT 130 steps 挖掘
→ prepared RL Follow DUMP
→ ExecuteDump
→ RL Follow 下一 DIG
```

自动多铲按 `dig_01 → dig_02 → dig_03` 循环。UI 分阶段模式用于逐段验收，一次执行单铲/多铲用于观测真实连续性。

## 时延证据

权威 handoff latency 定义为“旧 source 终态零 STM32 ACK → 新 source 第一条非零 STM32 ACK”。模型加载和 PC 规划等待应单独报告。Experiment Run 记录 generation、方向、序号和时间戳，不能只截图一个可覆盖的 latest 值。
