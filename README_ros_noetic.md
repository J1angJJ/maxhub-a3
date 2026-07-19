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

当前 TF 状态：

- `carm_a3_description` 已迁入厂家/下载模型，可通过 `robot_state_publisher` 根据 `/joint_states` 发布完整机械臂关节链 TF。
- `carm_a3_driver` 单独启动时仍可根据 SDK 末端位姿发布最小动态 TF：`base_link -> flange`。
- `carm_a3_bringup` 默认关闭 driver 的最小直连 `base_link -> flange`，改用 URDF + `/joint_states` 发布完整链路。
- `carm_a3_vision` 的图像 `frame_id` 为 `carm_a3_camera_optical_frame`。
- `carm_a3_calibration` 已保存并验证 `flange -> carm_a3_camera_optical_frame` 手眼静态 TF。
- 当前 URDF 来自 `carm_a3.zip`，应在 RViz 和实机姿态下继续核对关节方向、零位和 mesh 姿态。

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

如果 `readonly_state_node` 链接阶段出现 `libPocoNet.so.71 not found` 或 `undefined reference to Poco::...`，先确认本终端已经加载厂家 SDK 环境：

```bash
cd /home/noetic/maxhub-a3
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep arm_control_sdk
cd workspace/ubuntu/carm_ws
catkin_make
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
rosservice list | grep /carm_a3/camera/set_camera_info
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
- 相机信息写入服务：`/carm_a3/camera/set_camera_info`
- 标定文件：`workspace/ubuntu/carm_ws/src/carm_a3_vision/config/camera_info.yaml`
- 诊断话题：`/carm_a3/camera/diagnostics`
- 软件方向校正：`rotate_180: true`

### Camera Modes

`v4l2-ctl -d /dev/video0 --list-formats-ext` 实测支持：

| Format | Resolution | FPS |
| --- | --- | --- |
| MJPG | 320x240 | 30 |
| MJPG | 640x480 | 30 |
| MJPG | 800x600 | 30 |
| MJPG | 1024x768 | 30 |
| MJPG | 1280x720 | 30 |
| MJPG | 1280x1024 | 30 |
| MJPG | 1920x1080 | 30 |
| YUYV | 320x240 | 30 |
| YUYV | 640x480 | 30 |
| YUYV | 800x600 | 21 |
| YUYV | 1024x768 | 6 |
| YUYV | 1280x720 | 9 |
| YUYV | 1280x1024 | 6 |
| YUYV | 1920x1080 | 6 |

当前 `carm_a3_vision` 节点只实现了 `YUYV -> rgb8`，因此默认继续使用已经验证并完成内参标定的 `640x480 YUYV 30fps`。如果后续需要 `1280x720` 或 `1920x1080` 的 30fps，需要先给节点增加 MJPG 解码，或改用成熟的 ROS 相机驱动包。

注意：相机内参和分辨率绑定。当前 `camera_info.yaml` 只适用于 `640x480`，切换到其他分辨率后应重新标定。

## TF Check

启动 `carm_a3_driver` 后检查 TF：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rostopic echo -n 1 /tf
rosrun tf tf_echo base_link flange
```

期望：

- `/tf` 中能看到 `base_link -> flange`。
- `tf_echo base_link flange` 能持续输出平移和四元数。

这是 driver 单独启动时的最小 TF，用于早期 bring-up 和故障排查。

当前 bringup 已加入 URDF 模型。启动：

```bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch
```

检查完整链路：

```bash
rosrun tf tf_echo base_link link6
rosrun tf tf_echo base_link flange
rosrun tf tf_echo base_link carm_a3_camera_optical_frame
```

## Camera Intrinsic Calibration

相机内参标定应在手眼标定之前完成。当前仓库已保存 `640x480` 标定文件，并且 `carm_a3_vision` 启动时会默认加载：

```text
workspace/ubuntu/carm_ws/src/carm_a3_vision/config/camera_info.yaml
```

启动相机节点后检查：

```bash
rostopic echo -n 1 /carm_a3/camera/camera_info
```

期望 `K/D/R/P` 不再是全零。

### Install Tools

```bash
sudo apt update
sudo apt install ros-noetic-camera-calibration ros-noetic-camera-info-manager ros-noetic-image-view ros-noetic-rqt-image-view
sudo apt install ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-rviz
```

### Prepare Checkerboard

建议先用棋盘格标定。仓库已准备一份 A4 打印文件：

```text
docs/calibration/printables/checkerboard_squares_9x7_inner_8x6_square_25mm_A4.pdf
```

这份棋盘格实际为 `9 x 7` 个方格、`8 x 6` 个内部角点，单格边长 `25 mm`。因此 ROS 标定参数应为：

- `size`: `8x6`，这是内部角点数量，不是格子数量。
- `square`: `0.025`，单位是米。

打印时请选择“实际大小 / 100% / 不缩放”，不要使用“适合页面”。

另有一份 ArUco 打印文件，后续可用于手眼标定或位姿检查：

```text
docs/calibration/printables/aruco_original_id23_marker_100mm_A4.pdf
```

该标记来自 `https://chev.me/arucogen/`，使用 `DICT_ARUCO_ORIGINAL` 字典，ID 为 `23`，黑色标记外边长为 `100.0 mm`。打印后建议用尺子复核外边长。

### Run Camera Node

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

### Run Calibration

当前仓库棋盘格对应命令：

```bash
source /opt/ros/noetic/setup.bash
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 \
  --square 0.025 \
  image:=/carm_a3/camera/image_raw \
  camera:=/carm_a3/camera
```

如果看到：

```text
Waiting for service /carm_a3/camera/set_camera_info ...
Service not found
```

说明当前运行的相机节点还没有提供 `set_camera_info` 服务。请先拉取本仓库更新、重新编译 `carm_ws`，再重新启动 `carm_a3_vision`。

采集时让棋盘格覆盖画面不同区域和姿态：

- 中心、四角、边缘都要覆盖。
- 前后距离要变化。
- 有一定俯仰、偏航、滚转角度变化。
- 避免运动模糊和强反光。
- 保持棋盘完整出现在画面中。

标定完成后点击 `CALIBRATE`，结果稳定后点击 `SAVE`。保存得到的 YAML 后，再放入仓库：

```text
workspace/ubuntu/carm_ws/src/carm_a3_vision/config/camera_info.yaml
```

如果 YAML 中的 `camera_name` 不是 `carm_a3_camera`，建议改成 `carm_a3_camera`，与 `config/camera.yaml` 保持一致。

### Before Hand-eye Calibration

进入眼在手上手眼标定前，至少需要满足：

- `/carm_a3/camera/image_raw` 稳定。
- `/carm_a3/camera/camera_info` 已有真实内参。
- `base_link -> flange` TF 可用。
- 明确相机实际安装在末端还是固定在外部。
- 如果相机在末端，需要求解 `flange -> carm_a3_camera_optical_frame`。
- 如果相机在外部，需要求解 `base_link -> carm_a3_camera_optical_frame`。

## Eye-in-hand ArUco Sampling

当前相机安装在机械臂末端，先按眼在手上流程采样。`carm_a3_calibration` 提供只记录、不运动的 ArUco 采样节点：

- 识别字典：`DICT_ARUCO_ORIGINAL`
- 标记 ID：`23`
- 黑色标记外边长：`0.1 m`
- 机器人位姿：读取 TF `base_link -> flange`
- 相机观测：估计 `carm_a3_camera_optical_frame -> aruco_marker`
- 默认保存目录：`workspace/ubuntu/logs/handeye_samples`

安装依赖：

```bash
sudo apt update
sudo apt install python3-opencv python3-yaml ros-noetic-tf2-ros
```

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2，启动机械臂只读 TF：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_driver readonly_state.launch
```

终端 3，启动相机：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_vision camera.launch
```

终端 4，启动采样器：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_calibration aruco_handeye_sampler.launch
```

保持 ArUco 标记可见后，另开终端保存一组样本：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rosservice call /carm_a3/handeye/save_sample
```

建议采集至少 `15` 组，条件允许采 `20-30` 组。每组之间应改变末端位置和姿态，尤其要有明显的腕部旋转变化；不要只平移不旋转。采样时避免图像模糊、反光、ArUco 太远或只出现在画面极边缘。

原始样本默认在：

```text
workspace/ubuntu/logs/handeye_samples
```

该目录默认不进入 Git。确认采样质量后，再整理出可提交的标定结果。

采样完成后求解：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
rosrun carm_a3_calibration solve_handeye.py \
  --samples-dir /home/noetic/maxhub-a3/workspace/ubuntu/logs/handeye_samples
```

输出文件：

```text
workspace/ubuntu/logs/handeye_samples/handeye_result.yaml
```

脚本默认使用 `PARK` 作为推荐方法。先重点看 `sample_count`、`motion_summary`、`recommended_transform` 和 `method_consistency_to_recommended`。推荐结果表达的是：

```text
flange -> carm_a3_camera_optical_frame
```

如果 PARK 和 HORAUD 结果接近，而 TSAI、ANDREFF 或 DANIILIDIS 有偏差，优先把 PARK/HORAUD 当作当前草案；后续还需要用实物观察或重投影误差验证。

当前 30 组样本的 PARK 草案结果已整理到：

```text
workspace/ubuntu/carm_ws/src/carm_a3_calibration/config/handeye_flange_camera.yaml
```

结果质量摘要：

- PARK 与 HORAUD 平移差约 `1.1 mm`。
- PARK 与 HORAUD 旋转差约 `0.5 deg`。
- 最大末端相对旋转约 `178.8 deg`。

发布草案静态 TF：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_calibration publish_handeye_tf.launch
```

检查：

```bash
rosrun tf tf_echo flange carm_a3_camera_optical_frame
rosrun tf tf_echo base_link carm_a3_camera_optical_frame
```

你当前验证到的静态外参为：

```text
flange -> carm_a3_camera_optical_frame
translation: [0.061, -0.003, 0.045] m
RPY: [-26.1, 0.9, 89.0] deg
```

下一步做固定 ArUco 的一致性验证。保持 ArUco 纸固定不动，启动采样器发布实时 `camera -> marker`：

```bash
roslaunch carm_a3_calibration aruco_handeye_sampler.launch
```

另开终端观察：

```bash
rosrun tf tf_echo base_link aruco_marker
```

缓慢改变机械臂/相机姿态时，如果 `base_link -> aruco_marker` 的平移基本稳定，说明当前 `flange -> camera` 外参方向和数量级大体可信。如果漂移很明显，优先重新检查 ArUco 尺寸、相机画面方向、样本姿态覆盖和 TF 方向约定。

## Bringup Launch

`carm_a3_bringup` 用于组合启动当前已验证的只读链路。它不会执行机械臂运动。

启动默认链路：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch
```

该命令会启动：

- `carm_a3_driver` 只读状态节点和 `base_link -> flange` TF。
- `carm_a3_vision` 原装 USB 相机节点和 `640x480` 内参。
- `flange -> carm_a3_camera_optical_frame` 手眼静态 TF。

手眼验证链路：

```bash
roslaunch carm_a3_bringup handeye_validation.launch
```

该命令会在默认链路基础上启动 ArUco 实时检测，并发布 `carm_a3_camera_optical_frame -> aruco_marker`。

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

如果启动时看到 `libtcp_com.so: cannot open shared object file`，说明当前终端没有加载厂家 SDK 运行时库路径。当前 `readonly_state.launch` 已内置 vendor SDK 的 `LD_LIBRARY_PATH`；拉取更新并重新编译后可直接启动。临时处理仍可手动执行上面的 `source .../setup.bash`。

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

视觉链路的下一阶段建议是启动机械臂只读 TF 和相机节点，准备眼在手上的手眼标定采样。

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
- `camera_info.yaml` 已保存 `640x480` 内参；重新编译并启动当前版本后，`K/R/P/D` 应发布真实标定值。
- `image_view` 能正常显示图像；退出时出现 OpenCV GTK 窗口销毁异常：

```text
Can't destroy non-registered window
```

该异常发生在 `image_view` 退出阶段，不影响当前对相机话题、帧率和图像显示的判断。后续可优先使用 `rqt_image_view` 或继续用 `rostopic hz` 做链路检查。

### 2026-07-19 Camera Intrinsics Loaded

Ubuntu 20.04 + ROS Noetic 下已确认 `carm_a3_vision` 可以加载 `640x480` 相机内参：

- `/carm_a3/camera/camera_info` 尺寸为 `640x480`。
- `frame_id` 为 `carm_a3_camera_optical_frame`。
- `distortion_model` 为 `plumb_bob`。
- `D/K/R/P` 均已发布真实标定值，不再是全零。

当前内参只适用于 `640x480` 默认模式。切换到其他分辨率，尤其是后续 MJPG 高分辨率模式后，需要重新标定。

### 2026-07-19 Hand-eye Draft TF Validation Passed

Ubuntu 20.04 + ROS Noetic 下已完成第一版眼在手上外参 TF 验证：

- 已发布 `flange -> carm_a3_camera_optical_frame` 静态 TF。
- 已通过 ArUco 实时检测发布 `carm_a3_camera_optical_frame -> aruco_marker`。
- 固定 ArUco 纸不动，改变机械臂/相机姿态后，`base_link -> aruco_marker` 基本稳定。

实测 `base_link -> aruco_marker` 平移约在以下范围内波动：

```text
x: 0.231-0.233 m
y: -0.035--0.030 m
z: -0.001-0.003 m
```

该结果说明当前 PARK 方法求得的 `flange -> carm_a3_camera_optical_frame` 外参方向和数量级可信，可作为当前默认手眼外参继续后续视觉实验。后续如更换相机安装、相机分辨率、标定板尺寸或采样方式，需要重新标定和验证。

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
