# ROS Noetic Bring-up Notes

本文档记录 Ubuntu 20.04 + ROS Noetic 下的下一步开发流程。当前阶段目标是在 `carm_a3_driver` 中建立只读 ROS 节点：连接机械臂、读取状态、发布话题，但不使能、不回零、不运动。

## Recommended Order

不要先原样移植官方 `carm_ros` 节点。官方节点当前默认 IP 为 `10.42.0.101`，并且启动后会调用 `set_ready()`；对第一次 ROS bring-up 来说动作太主动。

推荐顺序：

1. 使用仓库内 vendored 厂家 C++ SDK。
2. 在 ROS Noetic 中编译本仓库的只读 state publisher。
3. 确认 `/joint_states`、`/maxhub_a3/flange_pose`、`/maxhub_a3/diagnostics` 正常。
4. 再参考 `private/carm_demo/carm_ros` 迁移 C++ ROS 节点，并移除自动使能和硬编码 IP。
5. 最后再建立显式确认的低速运动节点。

`carm_a3_driver` 只负责机器人底层接口、状态、诊断和安全运动入口。后续手眼标定、YOLO 抓取协作、强化学习等内容如果展开，建议拆成独立 ROS 包，避免驱动包变成大杂烩。

`carm_a3_vision` 负责原装 USB 相机的 ROS 图像采集、方向校正和后续视觉链路入口。当前版本不依赖 OpenCV、`cv_bridge`、`usb_cam` 或 `v4l2_camera`，直接通过 V4L2 读取 `/dev/video0` 并发布 `rgb8` 图像。

## Prepare Official SDK

厂家 C++ SDK 已移植到 ROS 工作区内：

```bash
cd /home/noetic/maxhub-a3
ls workspace/ubuntu/carm_ws/vendor/arm_control_sdk
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
```

## Build Workspace

在 Ubuntu 虚拟机中执行：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

## Network Check

```bash
cd /home/noetic/maxhub-a3
ping -c 4 192.168.31.60
curl -I http://192.168.31.60
```

## SDK Inspection

```bash
cd /home/noetic/maxhub-a3
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
echo "$arm_control_sdk_DIR"
echo "$LD_LIBRARY_PATH"
```

## Run Read-only ROS Node

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_driver readonly_state.launch
```

终端 3：

```bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic echo -n 1 /maxhub_a3/diagnostics
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /maxhub_a3/flange_pose
```

## Run USB Camera Node

原装相机已通过 `guvcview` 验证 USB 透传和 30 fps 预览。当前物理安装方向导致画面上下颠倒，`carm_a3_vision` 默认启用 `rotate_180: true` 做软件校正。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_vision camera.launch
```

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rostopic hz /carm_a3/camera/image_raw
rostopic echo -n 1 /carm_a3/camera/diagnostics
rostopic echo -n 1 /carm_a3/camera/camera_info
```

如已安装 `image_view`：

```bash
rosrun image_view image_view image:=/carm_a3/camera/image_raw
```

默认配置文件：

```text
workspace/ubuntu/carm_ws/src/carm_a3_vision/config/camera.yaml
```

默认参数：

- 设备：`/dev/video0`
- 分辨率：`640x480`
- 帧率：`30`
- 输入格式：`YUYV`
- 输出 ROS 编码：`rgb8`
- 图像话题：`/carm_a3/camera/image_raw`
- 相机信息话题：`/carm_a3/camera/camera_info`
- 诊断话题：`/carm_a3/camera/diagnostics`
- 软件方向校正：`rotate_180: true`

## Read-only Test Plan

编译成功后，下一步只测试“连接与状态读取”，不做使能、不回零、不运动。

### 1. Fresh Terminal Environment

每个新终端先准备 ROS 和 SDK 环境：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
```

确认环境变量：

```bash
echo "$arm_control_sdk_DIR"
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep arm_control_sdk
rospack find carm_a3_driver
```

期望：

- `arm_control_sdk_DIR` 指向 `workspace/ubuntu/carm_ws/vendor/arm_control_sdk`。
- `LD_LIBRARY_PATH` 中包含 `arm_control_sdk/lib` 和 `arm_control_sdk/poco/lib`。
- `rospack find carm_a3_driver` 能找到 `workspace/ubuntu/carm_ws/src/carm_a3_driver`。

### 2. Network Reachability

```bash
cd /home/noetic/maxhub-a3
ping -c 4 192.168.31.60
curl -I http://192.168.31.60
```

期望：

- `ping` 无丢包或基本稳定。
- `curl` 能返回 HTTP 响应头。

如果失败，先检查虚拟机桥接网络、Ubuntu IP 是否在 `192.168.31.0/24` 网段、机械臂是否接回路由器。

### 3. Start roscore

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

期望：`roscore` 正常启动，无端口占用报错。

### 4. Start Read-only Driver

终端 2：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_driver readonly_state.launch
```

期望：

- 日志显示目标控制器为 `192.168.31.60:8090`。
- 日志明确提示节点不会调用 ready、enable、motion、stop 或 gripper API。
- `/maxhub_a3/diagnostics` 中出现 `connected=true` 或相近状态。

如果出现动态库错误，重新 source SDK：

```bash
cd /home/noetic/maxhub-a3
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep arm_control_sdk
```

### 5. Inspect Topics

终端 3：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rostopic list
rostopic echo -n 1 /maxhub_a3/diagnostics
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /maxhub_a3/flange_pose
```

期望话题：

- `/joint_states`
- `/maxhub_a3/flange_pose`
- `/maxhub_a3/diagnostics`

`/joint_states` 期望包含 6 个关节名和 6 个位置值。  
`/maxhub_a3/flange_pose` 期望包含位置和四元数。  
`/maxhub_a3/diagnostics` 期望包含连接状态、自由度、伺服状态、控制器状态和速度百分比。

### 6. Record First Result

测试成功后建议保存一次只读 bring-up 记录：

```bash
cd /home/noetic/maxhub-a3
mkdir -p workspace/ubuntu/logs/readonly_test
date > workspace/ubuntu/logs/readonly_test/date.txt
rostopic list > workspace/ubuntu/logs/readonly_test/topics.txt
rostopic echo -n 1 /maxhub_a3/diagnostics > workspace/ubuntu/logs/readonly_test/diagnostics.txt
rostopic echo -n 1 /joint_states > workspace/ubuntu/logs/readonly_test/joint_states.txt
rostopic echo -n 1 /maxhub_a3/flange_pose > workspace/ubuntu/logs/readonly_test/flange_pose.txt
```

注意：`logs/` 默认被 `.gitignore` 忽略。需要长期保留的结论请整理成文档，不直接提交原始日志。

### 7. Stop Test

按顺序停止：

1. 终端 2 中 `Ctrl+C` 停止 `roslaunch`。
2. 终端 1 中 `Ctrl+C` 停止 `roscore`。
3. 不需要对机械臂执行额外动作，因为只读节点没有使能或运动。

## Next Gate

只有当以下条件全部满足后，再进入下一阶段：

- `catkin_make` 可重复成功。
- 网络可达稳定。
- 只读节点可重复启动。
- `/joint_states` 和 `/maxhub_a3/flange_pose` 数据看起来连续、非空、数量正确。
- `/maxhub_a3/diagnostics` 中没有错误状态。
- 实机急停和机械固定状态已确认。

下一阶段建议先做“显式参数保护的低速只读增强/安全服务”，不要直接开放运动话题。

视觉链路的下一阶段建议是相机内参标定，之后再做眼在手上的手眼标定。

## Test Log

### 2026-07-18 Read-only ROS Bring-up Passed

Ubuntu 20.04 + ROS Noetic 下已完成首次只读链路测试：

- `carm_ws` 工作区编译成功。
- `carm_a3_driver` 可通过 `roslaunch carm_a3_driver readonly_state.launch` 启动。
- 机械臂网络地址 `192.168.31.60` 可访问。
- 只读节点可连接底层 C++ SDK。
- `/joint_states` 可输出关节状态。
- `/maxhub_a3/flange_pose` 可输出末端位姿。
- `/maxhub_a3/diagnostics` 可输出诊断状态。
- 测试过程中未执行使能、回零、运动或夹爪控制。

本次截图留档位置：

```text
R:\maxhub-a3\private\log\屏幕截图 2026-07-18 183236.png
```

截图位于 `private/` 下，不进入 Git。

### 2026-07-18 USB Camera ROS Stream Passed

Ubuntu 20.04 + ROS Noetic 下已完成原装 USB 相机 ROS 图像链路测试：

- `carm_a3_vision` 可通过 `roslaunch carm_a3_vision camera.launch` 启动。
- `/carm_a3/camera/image_raw` 发布稳定，`rostopic hz` 实测约 `29.86-30.04 Hz`。
- `/carm_a3/camera/diagnostics` 输出：

```text
camera_started,device=/dev/video0,width=640,height=480,fps=30,rotate_180=true
```

- `/carm_a3/camera/camera_info` 可输出，当前尺寸为 `640x480`，`frame_id` 为 `carm_a3_camera_optical_frame`。
- 当前尚未做相机内参标定，因此 `camera_info` 中 `K/R/P/D` 仍为空或全零，属于预期状态。
- `image_view` 能正常显示图像；退出时出现 OpenCV GTK 窗口销毁异常：

```text
Can't destroy non-registered window
```

该异常发生在 `image_view` 退出阶段，不影响当前对相机话题、帧率和图像显示的判断。后续可优先使用 `rqt_image_view` 或继续用 `rostopic hz` 做链路检查。

## Safety

当前只读节点不会调用：

- `set_ready()`
- `set_servo_enable()`
- `set_control_mode()`
- `move_joint()`
- `move_pose()`
- `move_line_joint()`
- `move_line_pose()`
- `set_gripper()`

后续任何运动节点必须默认禁用运动，并要求显式参数和人工确认。
