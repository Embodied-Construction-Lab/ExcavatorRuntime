# waji_description

此 ROS 2 包以重新测量的 [`urdf/waji.urdf`](urdf/waji.urdf) 为唯一 FK 几何来源。
工作区根目录的历史 `urdf/` 项目已退出活动范围；不要从该目录或旧手写 FK
反向覆盖这里的 offset、sign 或 tip RPY。

模型本身的 `fk_root` 是右手 ROS 坐标（`+X` 前、`+Y` 左、`+Z` 上）。
`display.launch.py` 通过显式单位 TF 将它置于唯一系统根 `machine_root_ros` 下，启动：

```bash
ros2 launch waji_description display.launch.py
```

它运行 `robot_state_publisher` 与只读 Bucket Tip pose publisher，输入为现有
`/joint_states`，输出为：

```text
/bucket_tip_pose_machine_root_ros
machine_root_ros -> fk_root -> base_link -> swing_link -> boom_link -> arm_link -> bucket_link -> bucket_tip
```

`PoseStamped.header.stamp` 从 TF transform 原样复制，因而保持 `/joint_states` 的源时间。
`/joint_states` 在生产运行时只由 state bridge 提供。`rviz.launch.py` 仅用于可视化。

## 仅模型验证：RViz Panel Tests

独立 Tk slider 已移除，避免维护第二套 `/joint_states` 测试发布器。离线模型验证统一使用
RViz Mission Panel 的 `Tests` 标签页；它只在隔离的 fixture/shadow launch 中显式启用：

```bash
ros2 launch airy_excavator_bringup operator.launch.py profile:=fixture_shadow
```

`Tests` 提供 swing/boom/arm/bucket 四个滑块、弧度/角度显示、10 Hz 连续发布、单次发布和
Reset。统一离线 launch 将测试流映射为 `/offline/joint_states`。Panel 默认不创建测试
publisher；检测到 live、非 shadow、运动 Backend、动作发送器、非零数据报、状态过期或
第二个同名 JointState publisher 时均禁止测试发布。
