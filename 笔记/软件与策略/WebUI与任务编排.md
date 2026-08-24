# Web UI 与任务编排

## 入口

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/excavator-il
conda activate excavator-il
python scripts/run_collection_ui.py
```

页面地址：`http://127.0.0.1:8088/`。

## 页面职责

- 选择 RL/人工/直接定位或仅遥操作；
- 读取 Orin 权威 Campaign 的下一条 slot；
- 管理 Collector、双 RGB 预览和 STM32 遥测；
- 管理 AiryLidar Operator/RViz；
- 分阶段或自动运行 RL+ACT Mission；
- 精确停止本轮进程并等待终态零。

UI 是编排器，不直接打开相机或串口。它不会用页面按钮绕过 motion authorization、deadman 或 STM32 安全边界。

## 正式采集门禁

正式 `demonstration` 必须精确匹配 Campaign 的 `task_variant`、`soil_reset_block_id`、`dig_point_id`。UI 在预运动前和 Episode 创建前都会重新检查，防止多人操作或重录时 slot 漂移。

## 使用原则

- “安全停止”是受控软件停止，不是硬件急停；
- 不在另一个终端手工启动同类 Runtime；
- 页面显示“等待 Collector”时相机/遥测为空属于正常 idle；
- 浏览器刷新不应被当作任务停止；任务状态以后端 Supervisor 为准。
