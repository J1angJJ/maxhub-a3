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

默认使用 headless `gzserver`，启动时会暂停物理仿真，禁用 Gazebo 在线模型库查询，并把 `carm_a3_description` 的 mesh 目录加入 `GAZEBO_MODEL_PATH`。需要图形界面时，可另开终端进入同一容器后运行：

```bash
gzclient
```

## 现状

- 已接入 Gazebo Classic 11、`gazebo_ros` 和 `robot_state_publisher`。
- 当前 URDF 还没有 `ros2_control` / `gazebo_ros2_control` 传动配置，暂时不能通过控制器驱动关节。
- 下一步建议给机械臂 6 个关节补充仿真用 transmission、`ros2_control` 插件和 `joint_trajectory_controller` 配置，再把 Gymnasium 环境从解析运动学替换为 Gazebo step/reset 闭环。

## 并行说明

当前轻量 Gymnasium reaching 环境已经支持 `--num-envs` 并行。Gazebo 并行会涉及多个 `gzserver` 进程、ROS namespace、端口和临时 world/model 名称隔离，建议在单实例控制闭环稳定后再打开。
