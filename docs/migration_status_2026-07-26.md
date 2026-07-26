# Migration Status - 2026-07-26

本文档记录从 Windows + Ubuntu 虚拟机开发迁移到当前 Linux 工作区时的交接状态。它只同步文档认知，不改变核心 ROS/C++/Python 代码。

## Current Workspace

当前迁移目标路径：

```text
/home/j1angjj/workspace/maxhub-a3
```

历史文档和命令中仍有较多旧路径：

```text
/home/noetic/maxhub-a3
R:\maxhub-a3
```

这些路径保留作历史记录。后续在当前机器执行命令时，将 `/home/noetic/maxhub-a3` 替换为 `/home/j1angjj/workspace/maxhub-a3`，仓库内相对路径不变。

## Git State When Reviewed

- 当前分支：`master`
- 跟踪远端：`origin/master`
- 迁移梳理前工作区：干净
- 最近提交主线集中在 `carm_a3_tasks/block_grasp.py`、`block_grasp.yaml` 和任务文档，重点是抓取规划、安全接近、IK 跳支规避和失败回观测位。

本机允许 `git add` 和 `git commit`，但不要 `git push`。

## Project Stage

项目不再只是早期只读接入。根据仓库文档和最近提交，当前进度可概括为：

- 机械臂已完成网络改址，当前 IP 为 `192.168.31.60`。
- 厂家 C++ SDK 已 vendored 到 `workspace/ubuntu/carm_ws/vendor/arm_control_sdk`。
- ROS Noetic 工作区位于 `workspace/ubuntu/carm_ws`。
- `carm_a3_motion` 是当前推荐的统一 SDK 节点，负责状态发布和安全门控运动服务。
- `carm_a3_driver` 已降级为 legacy 只读兼容包。
- `carm_a3_description` 已迁入 A3 模型，配合 `/joint_states` 发布完整机械臂 TF。
- `carm_a3_vision` 已接入原装 UVC 相机，默认 `640x480 YUYV 30 fps`，并用 `rotate_180` 修正当前安装方向。
- `carm_a3_calibration` 已保存 `flange -> carm_a3_camera_optical_frame` 手眼外参草案。
- `carm_a3_perception` 已有红/绿方块 HSV 分割和调试图输出。
- `carm_a3_tasks` 已推进到观测位初始化、桌面投影、方块抓取规划、视觉重定位、轨迹优先和失败回观测位。

## Recommended Current Entry Points

编译：

```bash
cd /home/j1angjj/workspace/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

只读状态和基础运动服务，默认不会真实运动：

```bash
cd /home/j1angjj/workspace/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_motion safe_motion.launch
```

视觉、URDF、手眼和颜色分割：

```bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch launch_color_blocks:=true
```

抓取观测位规划：

```bash
roslaunch carm_a3_tasks pregrasp_overview.launch
rosrun carm_a3_tasks grasp_init.py plan
```

方块抓取默认先规划或 approach-only，不要直接开启下探：

```bash
roslaunch carm_a3_tasks block_grasp.launch command:=plan color:=green
roslaunch carm_a3_tasks block_grasp.launch command:=execute color:=green
```

真实下探和夹爪动作必须在人工确认安全后显式开启：

```bash
roslaunch carm_a3_tasks block_grasp.launch \
  command:=execute \
  color:=green \
  tcp_grasp_stage:=center \
  launch_camera_stack:=true \
  launch_motion:=true \
  motion_allow_motion:=true \
  motion_allow_gripper:=true \
  motion_dry_run:=false \
  motion_auto_ready_on_connect:=true \
  motion_register_callbacks_on_connect:=true \
  motion_pre_ready_delay_s:=1.0 \
  extra_args:="--allow-descend --use-gripper"
```

## Known Caution Points

- 启动真实运动前保持只启动一个 motion SDK 节点，避免多个客户端同时发命令。
- C++ SDK 真实运动路径需要官方风格初始化：`set_ready()`、回调注册和短延迟。文档中对应参数是 `auto_ready_on_connect:=true`、`register_callbacks_on_connect:=true`、`pre_ready_delay_s:=1.0`。
- 默认运动 gate 仍应保持保守：`dry_run`、`allow_motion`、`allow_gripper` 都必须显式确认。
- 当前抓取 TCP、接触高度和中心抓取下探仍是调试重点。默认 approach-only 是正确的安全基线。
- 小幅 IK 可用已验证，但部分方向和姿态会无解或跳支；任务层已有分段、轨迹和失败回观测位保护，继续调参时不要绕过这些保护。
- `README_getting_started.md` 中的网络拓扑含 Windows/虚拟机 IP，作为历史实验记录保留；当前机器网络需要重新实测后再覆盖。

## Suggested Next Steps

1. 在当前 Linux 环境中完成 ROS build smoke test，确认 vendored SDK 和 catkin 链接正常。
2. 只运行 `safe_motion.launch` 默认参数，检查 `/joint_states`、`/maxhub_a3/diagnostics` 和 SDK 状态。
3. 检查相机设备节点是否仍为 `/dev/video0`，确认 `camera.launch` 能稳定发布图像。
4. 运行 `readonly_vision_handeye.launch launch_color_blocks:=true`，确认 TF、相机、手眼和红/绿分割同屏正常。
5. 真实运动前先 dry-run `grasp_init.py plan` 和 `block_grasp.launch command:=plan`，再决定是否恢复 approach-only 实机执行。
