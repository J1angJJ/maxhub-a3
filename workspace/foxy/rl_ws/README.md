# rl_ws

这是 MAXHUB A3 在 ROS 2 Foxy 下的强化学习起步工作区。当前目标是先把机械臂 SDK 的 ROS2 基础接口迁移进来，形成可编译、可检查、默认不主动运动的开发基线。

## 当前内容

```text
rl_ws/
└── src/
    ├── carm_api/          # 官方 ROS2 demo 迁移基底，已改为参数化 IP 且默认不自动 ready
    └── carm_rl_bringup/   # 本项目的 ROS2 参数和 launch 入口
```

## 编译

在 Foxy 项目容器内：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

只编译当前迁移包：

```bash
colcon build --packages-select carm_api carm_rl_bringup --symlink-install
```

## 检查

```bash
ros2 pkg list | grep -E 'carm_api|carm_rl_bringup'
ros2 launch carm_rl_bringup carm_api.launch.py
```

`carm_api` 当前默认：

- `carm_ip: 192.168.31.60`
- `auto_ready_on_start: false`
- `register_callbacks_on_start: true`

启动节点不会主动 ready。真正接入实机训练前，需要再补一个安全执行层，避免 RL 直接向官方 topic 接口发布运动命令。
