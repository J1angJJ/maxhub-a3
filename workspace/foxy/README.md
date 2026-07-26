# MAXHUB A3 Foxy 工作区

这个目录是 MAXHUB A3 的 ROS 2 Foxy 开发入口，当前聚焦强化学习方向。

## 目录结构

```text
workspace/foxy/
├── docker/
│   └── foxy-maxhub-a3/   # 项目镜像与 compose 配置
└── rl_ws/                # ROS2 colcon 工作区
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

## 编译 rl_ws

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

## 迁移边界

当前只迁移机械臂 SDK 的 ROS2 基础接口和 bringup 参数。ROS1 中的视觉、手眼、抓取任务和安全运动层暂不整体搬运；后续应按 RL 需要逐步迁移，并优先补安全执行层。
