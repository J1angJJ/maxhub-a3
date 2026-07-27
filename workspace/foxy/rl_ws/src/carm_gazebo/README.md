# carm_gazebo

`carm_gazebo` 是 CArm / MAXHUB A3 的 Gazebo Classic 仿真入口。当前阶段只负责启动空世界并把 `carm_a3_description` 的 URDF 模型加载进 Gazebo，用于验证模型、坐标系和后续控制链路。

## 启动

在容器内：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch carm_gazebo spawn_a3.launch.py
```

启动带 ROS 2 control 的 Gazebo：

```bash
ros2 launch carm_gazebo spawn_a3_control.launch.py
```

发送一个 6 关节位置目标：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, 0.3, -0.3, 0.1, 0.1, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"
```

查看关节状态：

```bash
timeout 3s ros2 topic echo /joint_states
```

`spawn_a3.launch.py` 默认使用 headless `gzserver` 并暂停物理仿真，适合只检查模型加载。`spawn_a3_control.launch.py` 默认不暂停物理仿真，否则 Foxy 的 controller spawner 可能停在 configuring 阶段。两者都会禁用 Gazebo 在线模型库查询，并把 `carm_a3_description` 的 mesh 目录加入 `GAZEBO_MODEL_PATH`。需要图形界面时，可另开终端进入同一容器后运行：

```bash
gzclient
```

## 现状

- 已接入 Gazebo Classic 11、`gazebo_ros` 和 `robot_state_publisher`。
- 已为 6 个主臂关节接入 `gazebo_ros2_control`、`joint_state_broadcaster` 和 `joint_trajectory_controller`。
- 夹爪关节暂未纳入控制器，第一阶段 reaching 只控制 `joint1` 到 `joint6`。
- 下一步建议新增 Gazebo Gymnasium 环境，把动作转换为 `/arm_controller/joint_trajectory`，从 `/joint_states` 读回关节状态并复用现有 TCP/reward 计算。

## 并行说明

当前轻量 Gymnasium reaching 环境已经支持 `--num-envs` 并行。Gazebo 并行会涉及多个 `gzserver` 进程、ROS namespace、端口和临时 world/model 名称隔离，建议在单实例控制闭环稳定后再打开。
