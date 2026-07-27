# rl_ws

这是 MAXHUB A3 在 ROS 2 Foxy 下的强化学习开发工作区。当前路线是先保留官方 SDK/ROS2 demo 的薄封装，再逐步补齐机器人描述、Gazebo reaching 环境、轻量 Gymnasium 基线和实机外设接口。

## 当前内容

```text
rl_ws/
└── src/
    ├── carm_api/           # 官方 ROS2 demo 迁移基底，已改为参数化 IP 且默认不自动 ready
    ├── carm_rl_bringup/    # 本项目的 ROS2 参数和 launch 入口
    ├── carm_a3_description/ # A3 URDF、mesh、RViz2 展示入口
    ├── carm_a3_vision/     # ROS1 USB 相机节点的 Foxy 迁移版
    ├── carm_gazebo/        # Gazebo Classic 11 模型加载和 ros2_control 配置
    ├── carm_rl_env/        # 不依赖 Gazebo 的 Gymnasium reaching 基线
    └── carm_rl_gazebo/     # Gazebo reaching 训练、评估和轨迹追踪入口
```

## 编译

在 Foxy 项目容器内：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

只编译基础迁移包：

```bash
colcon build --symlink-install --packages-select carm_api carm_rl_bringup carm_a3_vision
source install/setup.bash
```

只编译 Gazebo/RL 相关包：

```bash
colcon build --symlink-install --packages-select carm_a3_description carm_gazebo carm_rl_env carm_rl_gazebo
source install/setup.bash
```

## 基础检查

```bash
ros2 pkg list | grep -E 'carm_api|carm_rl_bringup|carm_a3_description|carm_a3_vision|carm_gazebo|carm_rl_env|carm_rl_gazebo'
ros2 launch carm_rl_bringup carm_api.launch.py
```

`carm_api` 当前默认：

- `carm_ip: 192.168.31.60`
- `auto_ready_on_start: false`
- `register_callbacks_on_start: true`

启动节点不会主动 ready。真正接入实机训练前，需要再补一个安全执行层，避免 RL 直接向官方 topic 接口发布运动命令。

## USB 相机

相机节点来自 ROS1 `carm_a3_vision` 的 V4L2 实现，当前迁移为 ROS2 Foxy 包 `carm_a3_vision`。默认设备使用稳定的 by-id 路径：

```text
/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0
```

宿主机进入带相机映射的容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose -f compose.yaml -f compose.camera.yaml run --rm foxy-maxhub-a3 bash
```

容器内启动相机节点：

```bash
cd /workspace/rl_ws
source install/setup.bash
ros2 launch carm_a3_vision camera.launch.py
```

检查话题和标定信息：

```bash
ros2 topic hz /carm_a3/camera/image_raw
ros2 topic echo --once /carm_a3/camera/diagnostics
ros2 topic echo --once /carm_a3/camera/camera_info
ros2 run image_tools showimage --ros-args -r image:=/carm_a3/camera/image_raw
```

当前节点编译和 launch 参数检查已通过；运行采集需要容器启动时映射 `/dev/video*` 和 `/dev/v4l`。
