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
