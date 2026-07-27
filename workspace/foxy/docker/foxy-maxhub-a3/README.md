# foxy-maxhub-a3 Docker

本文档记录 MAXHUB A3 项目的 ROS 2 Foxy 容器运行配置。项目镜像从共享 Foxy 镜像派生：

```text
ubuntu-env:foxy-source -> ubuntu-env:foxy-user -> foxy-maxhub-a3:latest
```

共享镜像由 `/home/j1angjj/workspace/ubuntu-env/docker/foxy` 维护；本目录只放本项目的薄项目镜像、compose、设备映射、SDK/参考仓库挂载和本机配置模板。

## Files

```text
workspace/foxy/docker/foxy-maxhub-a3/
├── Dockerfile
├── compose.yaml
├── compose.camera.yaml
├── .env.example
└── README.md
```

## Prerequisites

确认共享 Foxy 镜像已经存在：

```bash
docker image inspect ubuntu-env:foxy-source
docker image inspect ubuntu-env:foxy-user
```

如果缺少 `ubuntu-env:foxy-source`，先手动准备源镜像：

```bash
docker pull osrf/ros:foxy-desktop
docker tag osrf/ros:foxy-desktop ubuntu-env:foxy-source
```

如果缺少 `ubuntu-env:foxy-user`，先构建共享环境：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/foxy
./scripts/build.sh
```

构建项目镜像：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose build
```

## Local Env

复制本项目 env 文件：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
cp .env.example .env
```

当前默认约定：

- 宿主 ROS 2 工作区：`/home/j1angjj/workspace/maxhub-a3/workspace/foxy`
- 容器主挂载点：`/workspace`
- 容器 ROS2 工作区：`/workspace/rl_ws`
- 现有 CArm SDK：只读挂载到 `/opt/maxhub-a3/arm_control_sdk`
- 官方 demo 参考仓库：只读挂载到 `/opt/maxhub-a3/reference/carm_demo`
- 机械臂 IP：`192.168.31.60`
- 相机映射：通过 `compose.camera.yaml` 按需打开，当前默认 `VIDEO_DEVICE=/dev/video4`

`.env` 可以按当前机器情况调整，例如 `DISPLAY`、`LIBGL_ALWAYS_SOFTWARE`、`VIDEO_DEVICE`、`ROS_DOMAIN_ID`。串口设备暂不默认映射，后续确实需要时再加实验 overlay。

## Validate Config

只展开 compose 配置，不启动容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose config
```

应确认：

- `image: foxy-maxhub-a3:latest`
- `network_mode: host`
- `/home/j1angjj/workspace/maxhub-a3/workspace/foxy` 挂载到 `/workspace`
- CArm SDK 和官方 demo 为只读挂载
- 默认不映射相机；叠加 `compose.camera.yaml` 后再确认 `/dev/video*` 和 `/dev/v4l`

## Run Shell

进入一次性容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose run --rm foxy-maxhub-a3 bash
```

确认相机已经连接后，叠加相机配置：

```bash
docker compose -f compose.yaml -f compose.camera.yaml run --rm foxy-maxhub-a3 bash
```

检查 ROS 2：

```bash
docker compose run --rm foxy-maxhub-a3 ros2 --help
```

检查 SDK 挂载：

```bash
docker compose run --rm foxy-maxhub-a3 \
  bash -lc 'test -f /opt/maxhub-a3/arm_control_sdk/setup.bash && ls /opt/maxhub-a3/reference/carm_demo/carm_ros2/src'
```

## Desktop GUI

当前 compose 已把宿主机 `DISPLAY`、`QT_X11_NO_MITSHM`、`LIBGL_ALWAYS_SOFTWARE` 和 `/tmp/.X11-unix` 传入容器。宿主机先开放本地 X11 访问：

```bash
xhost +local:docker
```

进入容器后检查图形链路：

```bash
echo $DISPLAY
ls -l /tmp/.X11-unix
rviz2 --help
```

打开机械臂模型：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch carm_a3_description display.launch.py
```

如果 `rviz2` 报 `could not connect to display`，优先检查宿主 `DISPLAY` 是否和 `/tmp/.X11-unix/X*` 对应，以及是否执行过 `xhost +local:docker`。当前默认 `LIBGL_ALWAYS_SOFTWARE=1`，适合没有 `/dev/dri` 的本机容器 RViz2；后续如果宿主提供 DRI/GPU 设备，可在 `.env` 中改为 `0` 并额外映射图形设备。Wayland 桌面通常仍可通过 XWayland 使用这套路径；如果后续要跑 Isaac Sim，建议单独建 NVIDIA 图形容器，不塞进当前 Foxy 项目镜像。

## Workspace

新 ROS 2 包放在：

```text
/home/j1angjj/workspace/maxhub-a3/workspace/foxy/rl_ws/src
```

编译 colcon 工作区：

```bash
docker compose run --rm foxy-maxhub-a3 \
  bash -lc 'cd /workspace/rl_ws && colcon build --symlink-install'
```

如果需要编译官方 `carm_ros2` 参考包，建议先复制到 `/workspace/src` 再改，保留 `/opt/maxhub-a3/reference/carm_demo` 只读。

## Boundaries

这里先只承载 ROS 2/Foxy 的项目运行配置。强化学习训练栈、PyTorch、仿真器、数据集或特定实验依赖后续加到项目镜像或实验 overlay，不放进共享 Foxy 镜像。
