# AiryLidar：感知、规划与 RViz

AiryLidar 是 PC 侧 Operator。它不拥有 STM32 串口，负责把状态、点云、目标和规划组织成可供 Orin 执行的行为。

## 三种 Profile

| Profile | 用途 | 运动 |
|---|---|---|
| `fixture_shadow` | 离线界面/学习 | 否 |
| `live_shadow` | 真机状态、雷达、RViz | 不发送非零动作 |
| `live_commissioning` | 真机规划、Follow 与 Mission | 显式授权后可能运动 |

Web UI 的“启动 RL + RViz”会管理活动 Operator。混合 Mission 中若尚未启动，后端会自动启动并等待就绪。

## 核心模块

- `runtime_bridge`：接收 Orin `machine_state_v1`，发布 `/joint_states`；
- `kinematics`：URDF、RobotModel、FK 和 bucket tip；
- `localmap`：雷达外参、点云裁剪、占据地图与 planning inputs；
- `mission`：目标、Plan/Follow、固定动作和 RViz Panel；
- `runtime`：统一感知子进程编排和日志。

## 目标与 waypoint

- `mission/config/excavation_demo.json`：`dig_01/02/03`、公共 dump 和多铲顺序；
- `mission/config/excavation_cycle.json`：单个 dig/dump 与任务容差；
- 中间 waypoint 可配置比端点更宽的到达容差，减少逐点停顿；端点仍保持精确交接。

修改目标后重启 Operator/Web UI。动态目标通过 PC RPC 下发，不需要为每个目标手工复制 Orin 配置。
