# ROS 2 Humble RL Start Notes

本文档记录暂停 ROS 1 主线后，基于 Ubuntu 22.04 + ROS 2 Humble 开始强化学习方向开发的初始判断。

目标不是直接迁移全部 ROS 1 功能，而是把现有 CArm A3 实机能力收敛为一个适合机器人学习和 Sim-to-Real 的 ROS 2 安全接口。

## Reference Repo

官方仓库已作为本地参考资料 clone 到：

```text
reference/carm_demo
```

该目录只作参考，已通过 `.gitignore` 排除，不进入本仓库版本控制。

当前参考分支：

```text
release
```

当前参考提交：

```text
b22ea30 同步1.0.260327发布版本
```

## Official ROS 2 Support

官方 `carm_demo` 提供 ROS 2 包：

```text
reference/carm_demo/carm_ros2/src/carm_api
```

源码结构很薄：

```text
carm_api/
├── CMakeLists.txt
├── package.xml
└── src/carm_ros.cpp
```

官方推荐环境是：

```text
Ubuntu 20.04.6 + ROS Noetic + ROS 2 Foxy + Python 3.8
```

官方 ROS 2 包是 C++ `ament_cmake` / `rclcpp` 节点，依赖：

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `arm_control_sdk`

它暴露的是 topic 风格接口，和官方 ROS 1 demo 基本对齐：

- `connect`
- `ready`
- `emergency_stop`
- `move_joint`
- `move_pose`
- `move_line_joint`
- `move_line_pose`
- `move_tracking_joint`
- `move_tracking_pose`
- `set_speed_level`
- `set_servo_enable`
- `set_collision_config`
- `set_gripper`
- `real_joint_state`
- `flange_cart_state`
- `arm_state`
- `task_completion`
- `carm_error`

## Humble Compatibility Assessment

从源码看，官方 ROS 2 demo 用的是常规 `rclcpp`、标准消息和 `ament_cmake`，没有明显 Foxy 专属 API。因此在 ROS 2 Humble 上大概率可以编译，主要风险不在 ROS 2 API，而在：

- 官方 SDK 的 Linux amd64 动态库是否能在 Ubuntu 22.04/glibc 2.35 下正常加载。
- `arm_control_sdkConfig.cmake` 暴露的 include/lib 路径是否适配 Humble colcon 构建。
- SDK 内部依赖的 Poco/jsoncpp 是否和 22.04 系统库发生符号冲突。
- 官方 demo 直接 topic 调运动接口，不带安全门控，不适合作为 RL 实机执行层。

建议先做只读编译/加载验证：

```bash
source /opt/ros/humble/setup.bash
source /path/to/arm_control_sdk/setup.bash
cd /workspace/ros2
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables carm_api
ldd install/carm_api/lib/carm_api/carm_ros_node
```

如果 `ldd` 出现 `not found`，优先检查 `arm_control_sdk/setup.bash` 和 `LD_LIBRARY_PATH`，不要先改业务代码。

## Do Not Use Official Topic Node As RL Executor

官方 `carm_ros.cpp` 中的运动 topic 回调会直接调用 SDK，例如：

```text
move_joint(...)
move_pose(...)
move_line_joint(...)
move_line_pose(...)
track_joint(...)
track_pose(...)
set_servo_enable(...)
set_gripper(...)
```

这适合做 SDK 使用示例，不适合直接接强化学习策略。原因：

- topic 没有请求/响应语义，策略无法可靠知道执行是否成功。
- 默认缺少 `dry_run`、`allow_motion`、`allow_gripper`、`allow_ready` 等安全 gate。
- 缺少动作限幅、工作空间限制、IK/FK 服务封装和执行后 readback verification。
- RL 高频输出如果直接连 `track_joint` / `track_pose`，需要额外 watchdog 和频率策略。

ROS 1 主线里已经验证过的安全设计，应迁移到 ROS 2：

- 默认不使能、不复位、不运动。
- 所有真实动作必须显式参数打开。
- 策略输出先限幅，再走 IK/FK 和工作空间检查。
- 实机执行使用 service/action，并返回明确结果。
- 执行后读取关节和末端状态做闭环验证。

## Recommended ROS 2 Workspace Layout

建议在本仓库另起 ROS 2 路径：

```text
workspace/ros2/
├── src/
│   ├── carm_a3_ros2_driver/
│   ├── carm_a3_description/
│   ├── carm_a3_bringup/
│   ├── carm_a3_rl_env/
│   └── carm_a3_rl_policy/
└── docker/
    └── humble-maxhub-a3/
```

第一阶段只需要两个核心包：

- `carm_a3_ros2_driver`: 安全版 SDK adapter，先做只读状态、FK/IK、dry-run motion service/action。
- `carm_a3_rl_env`: Gymnasium 环境和 ROS 2 bridge，策略输出小步目标，不直接调用官方裸 topic。

## Research Positioning

目标导师黄畅昕老师方向包括具身智能、机器人学习、强化学习、VLA、Sim-to-Real 和任务迁移。当前项目更适合以低硬件成本的机器人学习切入：

```text
基于 ROS 2 的低成本协作机械臂强化学习与 Sim-to-Real 策略迁移平台
```

第一版任务建议：

```text
wrist-camera visual pre-grasp positioning
```

也就是策略只学习把末端移动到目标方块上方的安全接近/观测位，不直接学习下探抓取。这样能复用已有相机、手眼、颜色块检测和 IK/FK 链路，同时显著降低实机风险。

## Milestones

1. 搭建 Ubuntu 22.04 + ROS 2 Humble Docker 环境。
2. 在 Humble 中编译官方 `carm_api`，仅验证 SDK 链接和 topic wrapper 可构建。
3. 新建 `carm_a3_ros2_driver`，先实现只读状态和 FK/IK service。
4. 迁移 ROS 1 的安全门控思想，加入 dry-run motion service/action。
5. 新建轻量 Gymnasium 仿真环境，训练 reaching/pre-grasp policy。
6. 通过 ROS 2 driver 做 dry-run replay。
7. 在人工确认下做小步实机验证，并记录成功率、步数、IK 失败率和安全拦截次数。

## Resume Narrative

可作为简历经历的表述：

```text
构建基于 ROS 2 Humble 的 CArm A3 协作机械臂强化学习实验平台，设计 Gymnasium 任务环境与安全执行适配层，通过视觉检测、IK/FK、动作限幅和执行验证实现 pre-grasp 策略的 Sim-to-Real 小样本验证。
```
