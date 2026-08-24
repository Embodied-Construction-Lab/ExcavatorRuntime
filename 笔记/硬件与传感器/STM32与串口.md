# STM32 与串口

## 权威链路

```text
Orin /dev/ttyTHS1 @ 460800
上行：stm32_control_telemetry.v2，约 20 Hz
RL 下行：stm32_velocity_command.v1
ACT/手柄下行：stm32_manual_command.v1
新传感器状态：约 10 Hz
```

串口必须由 RL Runtime、Collector、ACT Resident owner 三者之一独占。活动固件位于 `F407/data_celect`。

## 动作语义

```text
[boom, stick, bucket, swing]
```

RL 写物理速度；ACT/手柄写归一化杆量。未知 schema、过期、重复/乱序命令和传感器失效必须归零。

## 每日诊断

发动机关闭、双杆回中、deadman 释放，并先拔掉充电器：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/excavator-il
conda activate excavator-il
python scripts/diagnose_stm32_link.py
```

通过：`passed=true`、解析失败 0、序号间断 0、18～22 Hz、最大周期 ≤80 ms。

再运行零命令 soak：

```bash
python scripts/run_zero_command_soak.py
```

## 已确认现场问题

挖掘机充电会自动断开 STM32 与挖掘机侧链路。此时可能出现非 ASCII、NUL、CSV 字段数错误、字符被替换、状态无效和频率下降。正确处理是拔掉充电器、复位/重启后重新诊断，而不是修改 CSV parser 或降低频率门限。

## 资源冲突

```bash
fuser -v /dev/ttyTHS1 2>&1 || true
pgrep -af '[o]rin_state_sender|[r]esident_act_runtime|[e]xcavator-il collect' || true
```

日常由 UI 精确管理当前 Owner。不要使用模糊 `pkill`，避免杀掉无关实验或遗漏子进程。
