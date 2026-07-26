# Ubuntu Workspace

本目录用于保存 Ubuntu 20.04 侧的开发记录、环境快照、连接测试和 ROS / SDK 适配代码。早期命令使用虚拟机路径 `/home/noetic/maxhub-a3`；当前迁移目标路径为 `/home/j1angjj/workspace/maxhub-a3`。

## Quick Start

在当前 Linux 工作区执行：

```bash
cd /home/j1angjj/workspace/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
```

ROS Noetic 工作区：

```bash
cd /home/j1angjj/workspace/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
roslaunch carm_a3_motion safe_motion.launch
```

## Files

- `environment.yml`: conda 环境快照。
- `requirements.txt`: pip 依赖快照。
- `README_noetic_docker.md`: 当前 Linux 本机使用共享 Ubuntu 20.04 + ROS Noetic Docker 环境的记录和命令。
- `noetic-docker.env.example`: MAXHUB A3 的 Docker env 示例，只挂载本目录到容器 `/workspace`。
- `noetic-docker.pip-requirements.txt`: 项目专用 Python 依赖，主要用于 WebSocket fallback 和官方 Python SDK 检查。
- `noetic-maxhub-a3.hardware.compose.yaml`: 项目专用 Docker 硬件覆盖，映射当前机械臂相机并暴露 `/dev/v4l` by-id 路径。
- `config/robot.yaml`: 当前机械臂的非敏感配置。
- `scripts/check_network.py`: 只做网络连通性检查。
- `scripts/inspect_carm_sdk.py`: 只导入并检查 `carm` SDK 模块。
- `carm_ws/src/carm_a3_motion`: 当前推荐的统一 SDK 节点，发布状态并提供安全门控运动服务。
- `carm_ws/src/carm_a3_driver`: legacy 只读状态节点，作为兼容回退保留。
- `carm_ws/src/carm_a3_description`: A3 URDF、mesh 和完整机器人 TF。
- `carm_ws/src/carm_a3_vision`: 原装 USB 相机 V4L2 采集节点。
- `carm_ws/src/carm_a3_calibration`: 手眼标定采样、求解和静态 TF 发布。
- `carm_ws/src/carm_a3_perception`: 红/绿方块 HSV 分割。
- `carm_ws/src/carm_a3_bringup`: 组合启动状态、模型、相机、手眼和感知链路。
- `carm_ws/src/carm_a3_tasks`: 观测位初始化和彩色方块抓取任务脚本。

## Safety

默认启动仍必须保持安全门控关闭：可以检查网络、导入 SDK、读取模块信息、读取设备状态、发布图像和做规划；不得默认下发任何运动、回零、使能或夹爪动作。真实运动必须显式开启 `allow_motion`、关闭 `dry_run`，夹爪动作还必须显式开启 `allow_gripper`。
