# MAXHUB A3 Foxy 工作区

这个目录是 MAXHUB A3 的 ROS 2 Foxy 开发入口，当前聚焦强化学习方向。

## 目录结构

```text
workspace/foxy/
├── docker/
│   └── foxy-maxhub-a3/   # 项目镜像与 compose 配置
└── rl_ws/                # ROS2 colcon 工作区
    └── src/
        ├── carm_api/
        ├── carm_a3_description/
        └── carm_rl_bringup/
```

## 进入容器

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose run --rm foxy-maxhub-a3 bash
```

带相机：

```bash
docker compose -f compose.yaml -f compose.camera.yaml run --rm foxy-maxhub-a3 bash
```

带 NVIDIA GPU/DRI 图形设备：

```bash
docker compose -f compose.yaml -f compose.gpu.yaml run --rm foxy-maxhub-a3 bash
```

相机和 GPU 同时启用：

```bash
docker compose -f compose.yaml -f compose.camera.yaml -f compose.gpu.yaml run --rm foxy-maxhub-a3 bash
```

## 编译 rl_ws

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

## 查看机器人模型

宿主机允许本地 Docker 容器访问 X11：

```bash
xhost +local:docker
```

进入容器后编译并打开 RViz2：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch carm_a3_description display.launch.py
```

只发布 `robot_description` 和 TF，不打开 RViz：

```bash
ros2 launch carm_a3_description robot_state_publisher.launch.py
```

## 仿真方向

当前先把真实机械臂 SDK 接口和 URDF 描述迁到 Foxy。下一步建议按下面顺序推进：

1. Gazebo Classic：优先验证 URDF、关节轴、碰撞体、TF 和基础控制接口，和 Foxy 生态最贴近。
2. MuJoCo：适合强化学习训练，需要从 URDF 整理 actuator、joint limit、collision 和 MJCF 资产。
3. Isaac Sim / Isaac Lab：适合更重的视觉与并行仿真，但依赖 NVIDIA 图形栈和更大的镜像，建议单独建实验容器。

## 已知建模事项

当前 URDF 已增加无质量虚拟根 `world -> base_link`，用于避免 KDL 把带惯性的 `base_link` 当作根节点。关节信息和初步 RL 动作空间记录在 `carm_a3_description/docs/joints.md`。

## 迁移边界

当前已迁移机械臂 SDK 的 ROS2 基础接口、bringup 参数和机器人描述包。ROS1 中的视觉、手眼、抓取任务和安全运动层暂不整体搬运；后续应按 RL 需要逐步迁移，并优先补安全执行层。
