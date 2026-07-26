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

## Run Commands

以下命令会启动一次性容器，由操作者亲自执行和排查。

进入 bash：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  bash
```

带相机/串口硬件映射进入 bash：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  --hardware \
  bash
```

注意：共享 `compose.hardware.yaml` 会同时映射 `VIDEO_DEVICE` 和 `SERIAL_DEVICE`。如果宿主机没有 `/dev/ttyUSB0`，只为了相机使用 `--hardware` 可能失败。MAXHUB A3 当前更推荐使用本仓库里的项目覆盖文件：

```text
/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml
```

相机覆盖文件会映射 `/dev/video4`，并只读挂载 `/dev/v4l`，让仓库默认的 by-id 相机路径在容器内可见。

用项目覆盖文件进入 bash：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
docker compose \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  -f compose.yaml \
  -f /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-maxhub-a3.hardware.compose.yaml \
  run --rm noetic bash
```

检查 ROS 环境：

```bash
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  rosversion -d
```

编译本项目 catkin 工作区：

```bash
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && cd /workspace/carm_ws && catkin_make'
```

默认只读启动 motion 节点：

```bash
./scripts/run.sh \
  --env-file /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/noetic-docker.env \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_motion safe_motion.launch'
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

## Current Network Note

当前本机 IP 已从历史 Ubuntu 虚拟机的 `192.168.31.11` 变为 `192.168.31.10/24`。机械臂 IP 仍按 `192.168.31.60` 使用。此前访问异常已确认是急停按钮未复位导致，复位后浏览器访问恢复正常。
