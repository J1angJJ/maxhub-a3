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

## Official Network Environment

机械臂控制器当前网络地址为 `192.168.31.60`。上位机/示教器 Web 前端运行在控制器本机，ROS 节点不是等待机械臂主动推送到虚拟机端口，而是作为客户端主动连接控制器服务。

已做低侵入探测，未发送运动命令或写配置请求：

| Port | Role | Notes |
| --- | --- | --- |
| `22/tcp` | SSH | `OpenSSH_8.2p1 Ubuntu` |
| `80/tcp` | Web teach pendant | Apache `2.4.41 (Ubuntu)`，首页标题为 `Carm Teach Pendant` |
| `8090/tcp` | CArm SDK / WebSocket controller | 官方 C++ SDK 和前端默认连接 `ws://192.168.31.60:8090` |
| `1999/tcp` | Backend API | 前端使用的 HTTP API 服务 |
| `7070/tcp` | Perception default | 前端默认配置里存在，但当前端口未开放或不可达 |

前端默认连接配置：

```text
carm:       192.168.31.60:8090
extraArm:   192.168.31.60:8090
backend:    192.168.31.60:1999
perception: 192.168.31.60:7070
```

只读 API 验证：

```bash
curl http://192.168.31.60:1999/api/device/network
curl 'http://192.168.31.60:1999/api/config/load?path=config/config.json&module=controller&default=0'
```

实测控制器基础配置摘要：

- `scenario/model`: `A3_DM_C`
- `dof`: `6`
- `control_period`: `0.002 s`
- `task_period`: `0.02 s`
- `broadcast_rate`: `0.02 s`
- `arm_control_version`: `1.0.260203`

`/arm_gui/` 目录索引中可见模型资源目录：`A3_AC_A/`、`A3_DM_A/`、`A3_DM_B/`、`A3_DM_C/`、`A3_DM_S/`、`D3_AC_B/`、`sca5/`、`sca_dual/`。

开发原则：自研 ROS 链路优先使用官方 C++ SDK 连接 `8090`；`1999` 后端接口仅作为只读辅助信息来源。不要直接调用 Web 前端的写配置接口，例如 `/api/config/write`、`/api/device/network` 的 POST，除非明确需要并已确认风险。

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

## Motion Control Paths

`carm_a3_motion` 是当前自研运动控制入口，独立于 `carm_a3_driver`。当前有三类并行路径，互不覆盖：

- C++ SDK 路径：`safe_motion.launch`，用于下一阶段主测。真实运动前必须显式启用官方风格初始化：延迟、`set_ready()`、注册 joint / pose / error / completion 回调。
- WebSocket fallback：`py_ws_motion.launch`，使用厂家纯 Python/WebSocket SDK，已验证控制器接受 `TASK_MOVJ`，实机有细微动作。
- 自研 WebSocket 路径：`raw_ws_motion.launch`，仓库内直接实现 WebSocket JSON 通讯，不依赖厂家 Python `Carm` 类。
- C++ 官方 topic 对照：`official_topic_motion.launch`，保留为尽量贴近厂家 ROS1 demo 的诊断/对照节点。

默认安全原则不变：

- `dry_run: true`
- `allow_motion: false`
- `allow_ready: false`
- `allow_servo_enable: false`
- `auto_ready_on_connect: false`
- `register_callbacks_on_connect: false`
- `set_speed_before_motion: false`
- `use_duration: false`
- `wait_for_motion: false`
- `verify_after_motion: true`
- `verify_joint_tolerance_rad: 0.003`
- 小步 jog 上限：`0.03 rad`
- 一次完整关节目标移动的单关节差值上限：`0.15 rad`

### Compile

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

### C++ SDK Motion Path

早期未完整初始化时曾复现实测：

- `carm_a3_driver` 的 C++ SDK 状态读取正常。
- `safe_motion_node` 在真实 `move_joint(target, -1, false)` 内部 `exit code -11`。
- `official_topic_motion_node` 直接复刻官方 ROS1 demo 的 `move_joint(msg->position, -1, false)`，在没有 `set_ready()` 和回调注册时同样在该调用后段错误。
- 同一控制器状态下，Python/WebSocket `TASK_MOVJ` 能被接受并完成任务，实机有细微动作。

后续复测确认：`official_topic_motion_node` 启用 `auto_ready:=true`、`register_callbacks:=true`、`pre_ready_delay_s:=1.0` 后不再崩溃，机械臂有极小动作。因此当前判断是：C++ SDK 异步 `move_joint(..., -1, false)` 依赖厂家 ROS1 demo 的完整初始化顺序。

C++ service 节点已经补齐同样的初始化选项。真实极小步测试：

```bash
roslaunch carm_a3_motion safe_motion.launch \
  allow_motion:=true \
  dry_run:=false \
  auto_ready_on_connect:=true \
  register_callbacks_on_connect:=true \
  pre_ready_delay_s:=1.0
```

另开终端：

```bash
rosservice call /carm_a3/motion/status
rosservice call /carm_a3/motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

`jog_joint` 和 `move_joint` 会在 SDK 返回后读取关节状态做闭环确认。正常返回应包含：

```text
move_ret=1, verified=true, max_error=..., target=[...], actual=[...]
```

如果 `move_ret=1` 但 `verified=false`，说明控制器接受了命令，但在 `verify_timeout_s` 内没有到达目标容差，应先停止继续加大动作，检查当前模式、速度、碰撞/限位、目标是否被裁剪。

已增加上层规划会用到的只读服务：

```bash
rosservice call /carm_a3/motion/get_joint_snapshot
rosservice call /carm_a3/motion/get_cartesian_snapshot
rosservice call /carm_a3/motion/solve_fk "{tool_index: 0, positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
rosservice call /carm_a3/motion/solve_ik "{tool_index: 0, pose: [0.25, 0.0, 0.30, 0.0, 0.0, 0.0, 1.0], seed_positions: []}"
```

`solve_ik` 的 pose 顺序为 `x,y,z,qx,qy,qz,qw`，`seed_positions` 为空时默认使用当前关节作为参考解。IK/FK 服务本身不运动；确认返回的 `positions` 后，再交给 `/carm_a3/motion/move_joint` 执行。

也可以用 CLI：

```bash
rosrun carm_a3_motion motion_cli.py snapshot
rosrun carm_a3_motion motion_cli.py cart
rosrun carm_a3_motion motion_cli.py fk "0,0,0,0,0,0"
rosrun carm_a3_motion motion_cli.py ik-current
rosrun carm_a3_motion motion_cli.py ik-probe
rosrun carm_a3_motion motion_cli.py ik-offset 0.01 0 0
rosrun carm_a3_motion motion_cli.py ik "0.25,0,0.30,0,0,0,1"
```

已验证 `fk "0,0,0,0,0,0"` 可返回有效位姿，约为：

```text
[-2.9e-07, 0.0, 0.236, 0.707, 0.0, 0.707, 0.0]
```

`ik "0.25,0,0.30,0,0,0,1"` 返回 `inverse_kine ret=-1`，更像是目标位置/姿态离当前构型和工具约束太远，不代表 IK 接口不可用。优先用 `motion_cli.py cart` 获取当前实际 pose，或用已知 FK 输出做 IK round-trip，再逐步尝试小偏移。

如果 `ik-current` 也失败，运行：

```bash
rosrun carm_a3_motion motion_cli.py ik-probe
```

当前最新实测：增强版 `ik-probe --include-mm` 已确认 IK 可用。当前 cart pose、plan pose、FK(current joints) 都能解回接近当前关节的解。项目约定保持为米制位置和 `x,y,z,qx,qy,qz,qw`；`wxyz` 载荷顺序失败，毫米位置输入失败。

只解算当前位姿小偏移，不运动：

```bash
rosrun carm_a3_motion motion_cli.py ik-offset 0.01 0 0
```

该命令读取当前 cart pose，保持当前四元数不变，只对 `x/y/z` 增加米制偏移，然后调用 IK。确认返回 joints 合理后，再决定是否交给 `/carm_a3/motion/move_joint`。

IK 诊断记录见：

```text
docs/vendor/cpp_sdk_inverse_kine_probe_notes.md
```

本地环境排查项：

- 确认每个终端都重新 `source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash`。
- 用干净终端只加载 `/opt/ros/noetic/setup.bash`、vendored SDK setup、`carm_ws/devel/setup.bash`，避免其它 `LD_LIBRARY_PATH` 污染。
- 确认没有多个运动客户端同时发命令。
- 若要进一步定位，可用 `ldd devel/lib/carm_a3_motion/official_topic_motion_node` 查看是否链接到仓库内 vendored SDK/Poco 库。

提交官方 issue 时可说明：

- C++ 状态 API 和 WebSocket 通讯均正常。
- 如果跳过 `set_ready()` 和回调注册，C++ `move_joint(target, -1, false)` 可能在 `libarm_control_sdk.so` 内段错误。
- 按厂家 ROS1 demo 顺序执行 `set_ready()` 并注册回调后，C++ 异步 `move_joint()` 可正常小幅运动。
- Python/WebSocket 等价 `TASK_MOVJ` 成功。
- gdb 已确认旧崩溃栈落在 `libarm_control_sdk.so` 内的 `carm::CArmKernelImpl::move_joint(...)`，上一层为 `carm::CArmSingleCol::move_joint(...)`。

可反馈售后的简述：Ubuntu 20.04 / ROS Noetic / A3_DM_C，C++ SDK `CArmSingleCol::move_joint(target, -1, false)` 在未执行完整 ready/callback 初始化时段错误；`get_status()`、`get_joint_pos()` 正常；启用 `set_ready()` 和 SDK 回调注册后 C++ 运动正常；同一状态下 WebSocket `webRecieveTasks` `TASK_MOVJ` 也成功。

官方 topic 对照节点也保留完整初始化复测命令：

```bash
roslaunch carm_a3_motion official_topic_motion.launch \
  allow_move_joint:=true \
  auto_ready:=true \
  register_callbacks:=true \
  pre_ready_delay_s:=1.0
```

售后报告草稿见：

```text
docs/vendor/cpp_sdk_move_joint_gdb_report.md
docs/vendor/cpp_sdk_move_joint_gdb_full.txt
```

### Build

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

### Recommended Python WebSocket Path

先查状态：

```bash
roslaunch carm_a3_motion py_ws_motion.launch
rosservice call /carm_a3/py_motion/status
```

dry-run：

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=true
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

真动极小步：

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=false
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

实测 `0.005 rad` 能收到 `Task_Recieve` 和 task finished callback，机械臂可见细微动作。

### Raw WebSocket Path

这个节点完全由仓库代码直接发 WebSocket JSON，不使用厂家 Python `Carm` 类。用于把正式链路逐步从 vendored Python SDK 收敛到自研实现。

```bash
roslaunch carm_a3_motion raw_ws_motion.launch
rosservice call /carm_a3/raw_motion/status
```

```bash
roslaunch carm_a3_motion raw_ws_motion.launch allow_motion:=true dry_run:=true
rosservice call /carm_a3/raw_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

```bash
roslaunch carm_a3_motion raw_ws_motion.launch allow_motion:=true dry_run:=false
rosservice call /carm_a3/raw_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

### Legacy C++ Service Safe Launch

以下命令只启动 C++ service 节点的默认安全门控，不会自动执行 `set_ready()`，也不会打开真实运动：

```bash
roslaunch carm_a3_motion safe_motion.launch
rosservice call /carm_a3/motion/status
```

默认情况下，`jog_joint` 应返回被 `allow_motion=false` 拦截；这说明 C++ 诊断节点的安全门控生效。

### Official Topic Compatibility Test

如果需要对照厂家 ROS1 demo 行为，可以启动尽量贴近官方写法的 topic 节点。它不读当前关节、不拼 jog target、不走 service；收到 `sensor_msgs/JointState` 后直接调用：

```cpp
move_joint(msg->position, -1, false)
```

终端 1：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_motion official_topic_motion.launch \
  allow_move_joint:=true \
  auto_ready:=true \
  register_callbacks:=true \
  pre_ready_delay_s:=1.0
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rosservice call /carm_a3/official_motion/status
rostopic pub -1 /carm_a3/official_motion/move_joint sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
position: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []"
```

当前已验证：带完整初始化参数时，该节点不会崩溃，实机有极小动作。旧的无完整初始化崩溃栈已保留在 `docs/vendor/cpp_sdk_move_joint_gdb_report.md`，用于给厂家反馈“未初始化状态下 SDK 不应段错误”的健壮性问题。

### Python WebSocket Motion Details

厂家纯 Python/WebSocket 轻量接口作为 C++ 路径之外的并行 fallback。该节点直接加载 vendored SDK 里的 `carm.py`，发送 `TASK_MOVJ`，不导入 `carm_py` C++ 扩展。

如系统 Python 缺依赖，先安装：

```bash
pip3 install --user websocket-client
```

编译：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

先查状态：

```bash
roslaunch carm_a3_motion py_ws_motion.launch
rosservice call /carm_a3/py_motion/status
```

dry-run：

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=true
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

真动极小步：

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=false
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

如果 `use_sdk_clip:=true` 路径出错，再试绕过厂家 Python SDK 的关节裁剪函数，直接发原始 JSON：

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=false use_sdk_clip:=false
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
