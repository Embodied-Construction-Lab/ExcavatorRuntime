# Web UI 与任务编排

## 入口

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/excavator-il
conda activate excavator-il
python scripts/run_collection_ui.py \
  --config config/collection_ui.v3a-commissioning.pc.json
```

页面地址：`http://127.0.0.1:8088/`。

这是 2026-08-25 当前 V3-A 现场验收入口。需要保留终端时追加
`--no-browser`。该配置使用 candidate 固定轨迹并携带独立
commissioning acknowledgement；不能把它当作已晋升的 field 配置。

## 页面职责

- 选择 RL/人工/直接定位或仅遥操作；
- 读取 Orin 权威 Campaign 的下一条 slot；
- 管理 Collector、双 RGB 预览和 STM32 遥测；
- V2 模式管理 AiryLidar Operator/RViz；
- 启动、取消和显示 Orin 本地 V3-A 固定点闭环；
- 精确停止本轮进程并等待终态零。

UI 是编排器，不直接打开相机或串口。它不会用页面按钮绕过 motion authorization、deadman 或 STM32 安全边界。

V3-A 当前组合不启动 PC RViz。不能直接复用带 Behavior 控制权的
`live_commissioning` Operator，否则可能与 Orin 本地固定闭环争抢运动权。
如果 V3-A 需要 RViz 录像，应单独接入只读状态/TF/感知显示模式，
不得启动第二个 STM32 Owner 或外部 Behavior Gateway。

## V3-A 页面状态

```text
FOLLOW_DIG   → 1·RL 到挖点
ACT_DIG      → 2·ACT 挖掘
FOLLOW_DUMP  → 3·RL 倾倒
EXECUTE_DUMP → 3·RL 倾倒（固定张斗动作）
下一轮 FOLLOW_DIG → 4·RL 返回
```

发动机关闭时不能用机械动作判断阶段，应以页面的“当前阶段”、阶段高亮和
`V3-A local status` 日志为准。安全取消的通过标准是 `CANCELLED`、
`resident terminal zero acknowledged` 且无 traceback。

## 正式采集门禁

正式 `demonstration` 必须精确匹配 Campaign 的 `task_variant`、`soil_reset_block_id`、`dig_point_id`。UI 在预运动前和 Episode 创建前都会重新检查，防止多人操作或重录时 slot 漂移。

## 使用原则

- “安全停止”是受控软件停止，不是硬件急停；
- 不在另一个终端手工启动同类 Runtime；
- 页面显示“等待 Collector”时相机/遥测为空属于正常 idle；
- 浏览器刷新不应被当作任务停止；任务状态以后端 Supervisor 为准。
