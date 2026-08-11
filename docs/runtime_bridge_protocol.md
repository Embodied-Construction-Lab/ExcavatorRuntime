# PC—Orin 运行协议

当前活动架构：

```text
STM32 → Orin Machine State → UDP/18081 → PC 只读状态桥 → JointState/TF/规划/RViz
PC Plan/Execute Goal → TCP/18083 Orin Edge Gateway → Orin ONNX/fixed action → loopback/18082 → STM32
```

PC 不运行 ONNX 高频闭环，不生成 Physical Velocity Command，也不向 Orin 的动作端口发送
零命令或诊断命令。`live_commissioning` 是唯一真机 Operator profile。

## Orin → PC：`machine_state_v1`

默认 PC 监听 `0.0.0.0:18081/udp`。状态包必须包含：

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
    "control_enabled": true,
    "fault_flags": []
  },
  "actuator_state": {
    "boom": {"position_m": 0.12, "velocity_mps": 0.001},
    "stick": {"position_m": 0.15, "velocity_mps": 0.0},
    "bucket": {"position_m": 0.16, "velocity_mps": -0.002},
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

- `seq` 是状态包序号。
- `stamp_ms` 是 Orin epoch 毫秒时间，PC 写入 ROS JointState header。
- `stm32_stamp_ms` 是 STM32 开机 tick，只能追溯同一源采样，不能与 epoch 相减。
- `actuator_state` 供 Orin 本地策略闭环和 PC 诊断；`joint_state.position_rad` 供 PC FK/RViz。
- `stick` 与 ROS `arm_joint` 的映射必须显式处理。

PC 只读状态桥配置位于 `runtime_bridge/config/runtime.json`：

```bash
/usr/bin/python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --publish-joint-states \
  --print-every 100
```

它只接收、校验、记录状态并可发布 `/joint_states`，没有动作 sender。

## PC → Orin：远程 Machine Behavior

PC 通过可靠 TCP RPC 提交：

- 完整且 digest 匹配的 `TrajectorySnapshot`，用于 `Follow`；
- 行为名称 `ExecuteDig` 或 `ExecuteDump`；
- cancel 请求。

Orin 返回接受/拒绝、Feedback、Result 与 5 Hz runtime status。连接断开、取消、失败、完成、
超时或异常时，Orin 必须先通过本地唯一 Command Sink 提交终态零命令。

远程 RPC 默认端口为 `18083/tcp`。Orin 的连续动作 relay 仅绑定 loopback
`127.0.0.1:18082/udp`，PC 不应访问该端口。

## Orin → STM32：Physical Velocity Command

Orin 本地 ONNX 输出顺序固定为 `[boom, stick, bucket, swing]`。输出按 Machine Profile
反归一化后，前三轴单位为 `m/s`，swing 单位为 `rad/s`。上层不取反；低层方向适配和最终
物理限位由 STM32 负责。

历史字段 `action_type="normalized_velocity_command"` 与实际物理速度语义不一致，属于协议兼容债务；
不得据字段名再次归一化。

## 本机状态桥测试

终端 1：

```bash
python3 runtime_bridge/apps/pc_runtime_bridge.py \
  --config runtime_bridge/config/runtime.mock.json \
  --publish-joint-states
```

终端 2：

```bash
python3 runtime_bridge/apps/mock_orin_relay.py
```

该测试只验证 Orin 状态包到 PC `/joint_states`，不会产生动作。
