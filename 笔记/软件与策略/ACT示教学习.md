# ACT 示教学习

## 当前策略契约

```text
输入：front RGB 3×480×640 + 11 维本体状态
输出：4 维 MANUAL_ACTION [boom, stick, bucket, swing]
运行：10 Hz，有界 step budget（混合任务当前 130）
```

ACT 学习入土、切削、收斗和提升等难建模接触技能。其输出是归一化杆量，不是物理速度。当前部署是 `swing_zero_200000` 系列模型。

## 数据采集

正式新数据为双 RGB：`front` 观察挖掘区，`dump` 观察运转/倾倒区；同时记录 STM32、本体状态与专家动作。当前旧 ACT 推理仍只消费 `front`，双路数据用于下一阶段完整挖运倒策略和对照实验。

## 策略替换 Seam

`DigPolicy` 统一“状态/RGB → MANUAL_ACTION”。默认 provider 为 `lerobot_act`；未来 Diffusion Policy 只需实现同一接口和模型证据，不应修改 Resident owner、STM32 sink 或 Mission。

## 训练正确性

- 按 `soil_reset_block_id` 原子切分，避免同一土壤状态泄漏到验证集；
- checkpoint、deployment manifest、配置和代码 commit 都要进入 Experiment Run；
- 训练 loss 不能代替真机任务成功率；
- 部署前先 Shadow，再发动机关闭短 Motion，最后真机有界执行。

完整采集/训练/部署命令见 [相关命令](../相关命令.md) 第 7、8、10 节。
