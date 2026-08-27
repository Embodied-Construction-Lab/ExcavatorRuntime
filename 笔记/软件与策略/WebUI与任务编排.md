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
- 一次启动并记录一条 Episode，人工填写本条标签；
- 管理 Collector、双 RGB 预览和 STM32 遥测；
- V2 模式管理 AiryLidar Operator/RViz；
- 启动、取消和显示 Orin 本地 V3-A 固定点闭环；
- 精确停止本轮进程并等待终态零。

UI 是编排器，不直接打开相机或串口。它不会用页面按钮绕过 motion authorization、deadman 或 STM32 安全边界。

V3-A 自动 Mission 会使用 PC `live_shadow` RViz，只读显示状态、雷达和 Orin 实际跟踪
的三点轨迹。它不生成轨迹、不发送运动命令，也不会启动第二个 STM32 Owner 或外部
Behavior Gateway。不能改用带 Behavior 控制权的 `live_commissioning` Operator，否则
可能与 Orin 本地固定闭环争抢运动权。

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

## 单条采集边界

- 一次点击只创建一条 `demonstration` Episode，不自动推进固定 200 条计划。
- task/soil/dig/zone/repeat/note 是本条数据标签；UI 不再用 Campaign slot 阻止启动。
- 实际使用的 AiryLidar 目标配置仍要记录 commit/path/SHA，采集期间目标漂移会在录制前拒绝。
- 成功、失败和重录由操作者逐条确认；训练候选仍以终态元数据和 raw validator 为准。

## 使用原则

- “安全停止”是受控软件停止，不是硬件急停；
- 不在另一个终端手工启动同类 Runtime；
- 页面显示“等待 Collector”时相机/遥测为空属于正常 idle；
- 浏览器刷新不应被当作任务停止；任务状态以后端 Supervisor 为准。
