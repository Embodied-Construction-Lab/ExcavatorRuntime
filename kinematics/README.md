# 挖掘机运动学

`waji_description` 是当前 FK 的权威实现。权威文件是仓库内
`waji_description/urdf/waji.urdf`，包含重新实测的物理尺寸、关节零位和 tip
固定姿态。工作区根目录的历史 `urdf/` 项目已退出活动范围；禁止从它、旧手写 FK
或 `excavator_geometry.yaml` 回写这些数值。

ROS 内部统一使用右手 `machine_root_ros`：`+X` 前、`+Y` 左、`+Z` 上。
测量 URDF 保留其原生、同为右手的 `fk_root` 链接名。启动文件显式发布单位变换
`machine_root_ros -> fk_root`，所以最终 Bucket Tip 的唯一消费 topic 是：

```text
/bucket_tip_pose_machine_root_ros    geometry_msgs/msg/PoseStamped
```

该消息和动态 TF 都保留输入 `/joint_states.header.stamp`；没有 joint state 时不发布动态
姿态。Unity 左手变换只能在 runtime bridge 组装 ONNX observation 时发生，不能进入 ROS TF。

## 构建

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select waji_description \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 真机只读 FK 启动

`pc_runtime_bridge.py --publish-joint-states` 是 `/joint_states` 的唯一生产来源。启动新 FK：

```bash
ros2 launch waji_description display.launch.py
```

这不会发送控制命令。真机 ROS domain 中不得启用 RViz Panel 的 embedded joint tests 或
任何手工 `/joint_states` publisher。

验收只看右手话题：

```bash
ros2 topic echo /bucket_tip_pose_machine_root_ros --once
ros2 run tf2_ros tf2_echo machine_root_ros bucket_tip
```

按 `EvaluationReport/2026-07-15_bucket_tip_real_machine_coordinate_diagnosis.md` 的静止多姿态
协议采样，以重新测量的物理铲尖基准验证此 URDF。
