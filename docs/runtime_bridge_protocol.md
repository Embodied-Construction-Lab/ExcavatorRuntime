# Runtime Bridge 协议

第一阶段架构：

```text
STM32 -> Orin -> UDP machine_state_v1 -> PC ExcavatorRuntime
PC ExcavatorRuntime -> UDP policy_action -> Orin -> STM32
```

当前只实现中转协议、本机 mock 和 PC 侧接收发布，不发送真机 PWM。

## 固定地址

```text
PC_IP   = 192.168.2.127
ORIN_IP = 192.168.2.88

Orin -> PC: 192.168.2.127:18081
PC -> Orin: 192.168.2.88:18082
```

默认端口：

```text
Orin -> PC state: 18081/udp
PC -> Orin action: 18082/udp
```

## Orin -> PC: machine_state_v1

```json
{
  "type": "machine_state_v1",
  "schema_version": "1.0",
  "seq": 12345,
  "stamp_ms": 1780000000000,
  "stm32_stamp_ms": 850104,
  "source": "orin",
  "machine_id": "scale_excavator_v1",
  "safety": {
    "estop": false,
    "stm32_alive": true,
    "sensor_valid": true,
    "control_enabled": false,
    "fault_flags": []
  },
  "actuator_state": {
    "boom": {"position_m": 0.012, "velocity_mps": 0.001},
    "stick": {"position_m": -0.018, "velocity_mps": 0.0},
    "bucket": {"position_m": 0.006, "velocity_mps": -0.002},
    "swing": {"position_rad": 0.25, "velocity_rad_s": 0.01}
  },
  "joint_state": {
    "position_rad": {
      "swing": 0.25,
      "boom": 0.42,
      "arm": -0.75,
      "bucket": 0.36
    }
  }
}
```

注意：

- `seq` 是包序号，无单位。
- `stamp_ms` 是 Orin 发出这一帧时的系统时间，单位 ms。
- `stm32_stamp_ms` 是 STM32 开机后的 tick（ms），不是 epoch，不能和 Orin/PC wall clock 相减；它用于追溯同一源采样。PC 解码和日志必须保留它。
- PC 发布 `/joint_states` 时把 Orin `stamp_ms` 写入 ROS header；FK/TF 与 Bucket Tip pose 保留该时间。ROS `Header` 没有 `seq` 字段，因此 `seq` 的端到端关联仍需要专用的显式 provenance Interface，不能写入 frame 名或伪造关节。
- `actuator_state` 后续进入 ONNX 38 维 observation，不要归一化。
- `joint_state.position_rad` 是 FK 计算 bucket tip 用的关节角，单位 rad。
- 第一阶段 Orin 不需要发送 `joint_state.velocity_rad_s` 和 `raw_sensor`；PC 侧会把缺失的关节角速度补为 0。
- 当前 STM32 传感器频率是 10Hz，所以 Orin 第一版按 10Hz 有新数据就发。
- 第一阶段 `control_enabled=false`，只联调链路，不让动作真正进入 STM32 控制。

## PC -> Orin: policy_action

```json
{
  "type": "policy_action",
  "schema_version": "1.0",
  "seq": 456,
  "stamp_ms": 1780000000100,
  "action_order": ["boom", "stick", "bucket", "swing"],
  "action": [0.0, 0.0, 0.0, 0.0],
  "action_type": "normalized_velocity_command",
  "valid_for_ms": 100
}
```

注意：

- `pc_runtime_bridge.py --reply-zero` 只发零动作，用于联调链路；默认只接收状态。
- 为兼容 Orin 端解析，`action_type` 字段必须保持 `normalized_velocity_command`。
- `pc_policy_bridge.py` 是只读 ONNX 诊断入口：输出仍是 `[-1, 1]` 策略动作，并计算按 `shared/machine_profile.json` 反归一化后的候选物理速度，但不包含 UDP sender。真机发送只允许通过统一 Operator 的 Action Server 和唯一 Command Sink。
- 当前 `action` 顺序是 `boom, stick, bucket, swing`；前三个单位 m/s，`swing` 单位 rad/s。这里字段名沿用旧协议，数值语义以本条为准。
- PC 反归一化只按 ONNX 输出正负选择对应速度幅值，四轴符号必须保持不变；真机低层方向换算由 STM32 负责。
- Orin 必须检查 `valid_for_ms` 和本地接收时间，超时动作应丢弃并置零。
- Orin 必须检查 `estop=false`、`control_enabled=true`、`sensor_valid=true`、`stm32_alive=true` 后才能转发动作。
- Orin 不应再把 `action` 当作 `[-1, 1]` 归一化量解释。

## 本机回环测试

终端 1，启动 PC 侧：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --reply-zero
```

终端 2，启动 mock Orin：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/mock_orin_relay.py
```

如果要让 PC 侧把状态发布成 ROS2 `/joint_states`：

```bash
source /opt/ros/jazzy/setup.zsh
source ros2_ws/install/setup.zsh

python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --reply-zero \
  --publish-joint-states \
  --print-every 100
```

`--print-every N` 只控制每 N 个有效状态包输出一行诊断日志；`0` 关闭周期打印。不传该参数时
继续使用 runtime 配置中的 `diagnostics.print_every`。

## 连接真实 Orin

PC 侧：

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar
python3 runtime_bridge/apps/pc_runtime_bridge.py --reply-zero
```

这条命令会按 `runtime_bridge/config/runtime.json` 监听 Orin 的 `machine_state_v1`，写出 `runtime_bridge/exports/latest_state.json`，并回发零动作。零动作只用于通信联调。

## PC→Orin发送记录

策略动作、固定动作和诊断零动作只有在 UDP `sendto` 成功后才会写入本地 JSONL。目录与轮转策略由
`runtime_bridge_config_v10` 的 `action_journal` section 指定；正式配置目录默认为：

```text
runtime_bridge/exports/action_journal/
```

每次进程启动创建独立会话文件，记录格式为 `pc_orin_action_send_v1`。其中 `packet` 方便人工检查，
`payload_base64` 是实际发送字节的可回放副本，`payload_sha256` 用于复测前校验内容未变化。
正式配置每个文件最大64 MiB并保留16个文件；超过保留数量时删除最旧文件，避免长期运行耗尽磁盘。
队列满或写盘失败会使后续动作在发送前失败并结束发送进程。由于只有成功发送才入日志，未启用动作发送的dry-run不会产生容易误解的“已发送”记录。

执行器位置上下界的强制等级由统一 Operator 的 `control_stage` 决定，不再由 runtime config 中的
独立 bypass 开关控制。`commissioning` 只把尚未标定的上下界降级为诊断；`production` 强制范围。
两种阶段都阻断非有限位置、过期/无效 Machine State、关闭的 Safety State 和越界物理速度命令。
