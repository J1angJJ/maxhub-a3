# ROS Noetic Docker Notes

本文档记录在当前 Linux 本机上使用共享 Ubuntu 20.04 + ROS Noetic Docker 环境开发 MAXHUB A3 的建议命令。当前只记录，不自动启动容器。

## Shared Docker Environment

已检查的共享环境路径：

```text
/home/j1angjj/workspace/ubuntu-env/docker/noetic
```

主要文件：

- `Dockerfile`: 基于 `osrf/ros:noetic-desktop-full`，默认构建 `ubuntu-env:noetic-user`。
- `compose.yaml`: 默认 `network_mode: host`、`ipc: host`，适合 ROS1 和局域网机械臂通信。
- `compose.hardware.yaml`: 按需映射相机/串口设备。
- `compose.gpu.yaml`: 按需启用 NVIDIA GPU。
- `scripts/build.sh`: 只构建共享镜像，要求源镜像已提前存在。
- `scripts/run.sh`: 组合 compose 文件并 `docker compose run --rm noetic ...`。

共享镜像定位合理：它只提供 Ubuntu 20.04、ROS Noetic、C++/Python/GUI 调试基础工具，不把 MAXHUB A3 工程代码或厂商 SDK 烘进镜像。厂商 SDK 继续来自本仓库挂载目录。

静态检查结果：

- 源镜像 `osrf/ros:noetic-desktop-full` 已存在于本机 Docker。
- 派生镜像 `ubuntu-env:noetic-user` 尚未构建。
- 使用本项目 env 展开 `compose.yaml` 后，宿主机挂载范围确认为 `/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu:/workspace`。
- 当前 shell 的 `DISPLAY` 会覆盖 env 文件里的 `DISPLAY=:0`；本次 `docker compose config` 展开为 `DISPLAY=:1`。
- 共享 Noetic dev 包清单已显式包含 `v4l-utils`、`ros-noetic-image-view`、`ros-noetic-camera-calibration`、`ros-noetic-cv-bridge`、`ros-noetic-camera-info-manager`、`ros-noetic-tf2-ros`、`python3-opencv` 和 `python3-yaml` 等视觉/TF/标定排查基础包。若这些包是新增的，需要重新构建 `ubuntu-env:noetic-user`。
- 阅读本项目代码后，ROS/apt 依赖已基本由共享 Noetic 镜像覆盖；剩余项目级 Python 依赖集中在 WebSocket fallback 和官方 Python SDK 检查，记录在 `noetic-docker.pip-requirements.txt`。

## Project Env

本项目的示例 env 文件在：

```text
/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env.example
```

首次使用前复制成被 Git 忽略的本地文件：

```bash
cp /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env.example \
   /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env
```

关键约定：

- 宿主机只挂载 `/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu`。
- 容器内挂载点是 `/workspace`。
- 容器内 catkin 工作区是 `/workspace/carm_ws`。
- 因为 `compose.yaml` 使用 host 网络，容器内访问机械臂仍使用 `192.168.31.60`。
- 项目硬件覆盖会把同一个宿主目录额外挂到 `/home/developer/maxhub-a3/workspace/ubuntu`，用于兼容手眼采样等历史默认输出路径；没有扩大宿主挂载范围。

## Build Image

只构建镜像，不启动项目容器：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
./scripts/build.sh
```

如果提示缺少源镜像，先由操作者手动拉取：

```bash
docker pull osrf/ros:noetic-desktop-full
```

确认共享镜像：

```bash
docker image inspect ubuntu-env:noetic-user
```

安装项目 Python 依赖：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  run --rm noetic \
  bash -lc 'python3 -m pip install --user -r /workspace/noetic-docker.pip-requirements.txt'
```

这些依赖不放进共享 Noetic 镜像，避免把 MAXHUB A3 的 Python SDK/WebSocket fallback 绑到其他项目。

## Run Commands

以下命令会启动一次性容器，由操作者亲自执行和排查。

为了避免共享 `compose.hardware.yaml` 默认映射不存在的 `/dev/ttyUSB0`，MAXHUB A3 当前推荐直接使用本仓库的项目覆盖文件：

```text
/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml
```

进入 bash：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  bash
```

带相机硬件映射和 `/dev/v4l` by-id 路径进入 bash：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  -f /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml \
  run --rm noetic bash
```

该项目覆盖文件还会把同一个宿主目录挂到容器内 `/home/developer/maxhub-a3/workspace/ubuntu`。这是为了让 `aruco_handeye_sampler.launch` 默认的 `$(env HOME)/maxhub-a3/workspace/ubuntu/logs/handeye_samples` 写到持久化目录，而不是写进一次性容器层。

检查 ROS 环境：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  run --rm noetic rosversion -d
```

编译本项目 catkin 工作区：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  run --rm noetic \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && cd /workspace/carm_ws && catkin_make'
```

默认只读启动 motion 节点：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  run --rm noetic \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_motion safe_motion.launch'
```

启动相机节点时使用项目硬件覆盖：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  -f /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml \
  run --rm noetic \
  bash -lc 'source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_vision camera.launch'
```

检查相机枚举和格式：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  -f /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml \
  run --rm noetic \
  bash -lc 'ls -l /dev/video4 /dev/v4l/by-id && v4l2-ctl -d /dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 --list-formats-ext'
```

检查项目 Python fallback 依赖：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  run --rm noetic \
  python3 -c 'import carm, websocket, zeroconf, ifaddr; print("project python deps ok")'
```

## Camera Notes

当前宿主机相机枚举：

- `/dev/video0` 到 `/dev/video3`: 内置摄像头相关节点。
- `/dev/video4`: MAXHUB A3 原装 USB 相机 capture 节点。
- `/dev/video5`: 同相机 companion/non-capture 节点。
- 稳定 by-id capture 路径：`/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0`。

共享 `compose.hardware.yaml` 默认只映射 `VIDEO_DEVICE` 这个设备节点。若使用当前仓库默认 `camera.yaml` 中的 by-id 路径，容器里还需要能看到 `/dev/v4l/by-id/...`。有两种保守做法：

1. 临时用 ROS 参数或 Docker 专用配置把 camera device 改为 `/dev/video4`。
2. 另写项目 override，额外挂载 `/dev/v4l:/dev/v4l:ro`，同时仍用 `VIDEO_DEVICE=/dev/video4` 映射真实字符设备。

暂时不要把整个 `/dev` 或 `privileged: true` 加进共享 compose；只有厂商 SDK 或设备访问确实需要时，再写项目专用 override。

## Dependency Notes

本项目代码检查到的关键依赖：

- C++ SDK 节点：`arm_control_sdk`、Poco 和相关 `.so` 已 vendored 在 `/workspace/carm_ws/vendor/arm_control_sdk`，不需要额外挂载。
- 相机节点：直接 V4L2，运行时需要 `/dev/video4` 和 `/dev/v4l/by-id`，项目硬件覆盖已处理。
- 感知/标定：`cv_bridge`、OpenCV、YAML、TF、`image_view`、`camera_calibration` 已放在共享 Noetic dev 包清单。
- 描述/TF：`robot_state_publisher`、`joint_state_publisher`、RViz、`xacro`、`urdf`、`tf2_tools` 已由共享镜像覆盖。
- Python WebSocket fallback：`websocket-client` 是实际 import 依赖。
- 官方 Python SDK 检查：`carm`、`zeroconf`、`ifaddr` 属于 MAXHUB A3 项目级依赖，使用 `noetic-docker.pip-requirements.txt` 安装。

## Current Network Note

当前本机 IP 已从历史 Ubuntu 虚拟机的 `192.168.31.11` 变为 `192.168.31.10/24`。机械臂 IP 仍按 `192.168.31.60` 使用。此前访问异常已确认是急停按钮未复位导致，复位后浏览器访问恢复正常。
