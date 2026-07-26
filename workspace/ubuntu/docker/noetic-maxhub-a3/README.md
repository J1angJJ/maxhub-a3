# noetic-maxhub-a3 Docker

本文档记录 MAXHUB A3 项目的专属 ROS Noetic 容器环境。它从共享 Noetic 镜像派生一个项目镜像：

```text
ubuntu-env:noetic-user -> noetic-maxhub-a3:latest
```

共享镜像只提供 Ubuntu 20.04、ROS Noetic、C++/Python/GUI/视觉调试基础包；本目录只补 MAXHUB A3 项目自己的 Python 依赖、挂载和启动命令。

## Files

```text
workspace/ubuntu/docker/noetic-maxhub-a3/
├── Dockerfile
├── compose.yaml
├── requirements.txt
├── .env.example
└── README.md
```

- `Dockerfile`: 基于 `ubuntu-env:noetic-user`，安装 `requirements.txt`。
- `requirements.txt`: MAXHUB A3 项目 Python 依赖，主要用于官方 Python SDK 检查和 WebSocket fallback。
- `compose.yaml`: 项目容器运行配置，限制宿主挂载到 `workspace/ubuntu`。
- `.env.example`: 可复制的本地配置模板。

## Prerequisites

先确保共享 Noetic 源镜像和共享开发镜像存在：

```bash
docker image inspect ubuntu-env:noetic-source
docker image inspect ubuntu-env:noetic-user
```

如果缺少 ROS 源镜像：

```bash
docker pull osrf/ros:noetic-desktop-full
docker tag osrf/ros:noetic-desktop-full ubuntu-env:noetic-source
```

如果缺少共享开发镜像，先构建共享环境：

```bash
cd /home/j1angjj/workspace/ubuntu-env/docker/noetic
./scripts/build.sh
```

共享 Noetic dev 包清单应包含常用 ROS 调试依赖，例如 `v4l-utils`、`image_view`、`camera_calibration`、`cv_bridge`、OpenCV、TF、URDF、xacro、diagnostic 和 dynamic_reconfigure。项目特有依赖不要放进共享镜像。

## Local Env

复制本项目 env 文件：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
cp .env.example .env
```

当前默认约定：

- 宿主项目目录：`/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu`
- 容器主挂载点：`/workspace`
- 容器 catkin 工作区：`/workspace/carm_ws`
- 机械臂 IP：`192.168.31.60`
- 当前机械臂相机 capture 节点：`/dev/video4`
- 相机 by-id 路径通过只读挂载 `/dev/v4l` 暴露

`.env` 可以按当前机器情况调整，例如 `DISPLAY`、`VIDEO_DEVICE`、`ROS_HOSTNAME`。

## Build Project Image

构建项目专属镜像：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose build
```

确认镜像：

```bash
docker image inspect noetic-maxhub-a3:latest
```

该镜像会安装：

- `carm==0.1.20260716`
- `websocket-client==1.8.0`
- `zeroconf==0.136.2`
- `ifaddr==0.2.0`

## Validate Config

只展开 compose 配置，不启动容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose config
```

应确认：

- `network_mode: host`
- 宿主只挂载 `/home/j1angjj/workspace/maxhub-a3/workspace/ubuntu`
- `/dev/video4` 被映射
- `/dev/v4l` 以只读方式挂载
- 同一个宿主目录也挂到 `/home/developer/maxhub-a3/workspace/ubuntu`

最后一个挂载用于兼容历史默认输出路径，例如手眼采样默认写到：

```text
$(env HOME)/maxhub-a3/workspace/ubuntu/logs/handeye_samples
```

这不会扩大宿主挂载范围，只是同一目录的第二个容器内入口。

## Run Shell

进入一次性容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose run --rm noetic-maxhub-a3 bash
```

检查 ROS：

```bash
docker compose run --rm noetic-maxhub-a3 rosversion -d
```

检查项目 Python 依赖：

```bash
docker compose run --rm noetic-maxhub-a3 \
  python3 -c 'import carm, websocket, zeroconf, ifaddr; print("project python deps ok")'
```

## Build Workspace

编译 catkin 工作区：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && cd /workspace/carm_ws && catkin_make'
```

进入容器后手动编译：

```bash
source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash
cd /workspace/carm_ws
catkin_make
source devel/setup.bash
```

## Camera Checks

检查相机节点和格式：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'ls -l /dev/video4 /dev/v4l/by-id && v4l2-ctl -d /dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 --list-formats-ext'
```

启动相机 ROS 节点：

```bash
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_vision camera.launch'
```

当前仓库默认相机配置使用 by-id 路径：

```text
/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0
```

## Robot Checks

容器使用 host 网络，机械臂仍按 `192.168.31.60` 访问。

只读启动 motion 节点，默认不运动：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/ubuntu/docker/noetic-maxhub-a3
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_motion safe_motion.launch'
```

如机械臂不可达，先检查：

- 急停按钮是否复位。
- 机械臂是否上电。
- 网线是否接到路由器 LAN 口。
- 浏览器是否能访问 `http://192.168.31.60`。

## Bringup Examples

视觉、模型、手眼和颜色分割链路：

```bash
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_bringup readonly_vision_handeye.launch launch_color_blocks:=true'
```

预抓取观测位规划：

```bash
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_tasks pregrasp_overview.launch'
```

方块抓取只规划：

```bash
docker compose run --rm noetic-maxhub-a3 \
  bash -lc 'source /workspace/carm_ws/vendor/arm_control_sdk/setup.bash && source /workspace/carm_ws/devel/setup.bash && roslaunch carm_a3_tasks block_grasp.launch command:=plan color:=green'
```

## Boundaries

不要在本项目镜像里加入 CUDA、PyTorch、YOLO、MoveIt、navigation、SLAM 或特定新硬件驱动。需要时为具体实验再写派生镜像或额外 compose 覆盖。

不要默认开启：

- `privileged: true`
- 整个 `/dev:/dev`
- 固定 `container_name`
- GPU

这些能力只在明确需要时添加项目/实验专用覆盖文件。
