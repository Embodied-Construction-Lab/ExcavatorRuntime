# RL 轨迹跟踪

## 职责

RL 负责自由空间和带载阶段的斗尖轨迹跟踪，不负责土体接触挖掘技能。输入是 38 维可测状态与前视 waypoint，输出是四维物理速度 `[boom, stick, bucket, swing]`。

## 部署链

```text
PC Plan trajectory
→ TCP 18083 Behavior RPC
→ Orin Trajectory Controller
→ 127.0.0.1:18082 policy action
→ Resident STM32 sink
```

Orin 的活动 backend 当前为 `onnx_rl`；策略替换 Seam 还允许传统 `cartesian_p` Controller 做对照实验。两者必须返回同一 velocity-reference 契约，不能自行打开串口。

## 关键观察

- waypoint 是局部趋势，不应机械地在每个中间点完全停住；
- 中间点与端点允许不同容差；
- Follow 成功必须以真实状态和 trajectory contract 判定；
- PC/Orin 时间不同步会直接造成 state/map stale；
- RL 字段历史上叫 `normalized_velocity_command`，线上值实际已是物理速度，禁止二次归一化。

独立 RL 调试命令见 [相关命令](../相关命令.md) 第 5 节；日常混合任务不要手工启动另一份 RL Runtime。
起点、终点和圆弧中间点的容差含义、修改位置与双端同步步骤见
[轨迹到达容差调节](轨迹到达容差调节.md)。
