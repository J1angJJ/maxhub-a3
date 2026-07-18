# Ubuntu Workspace

本目录用于保存 Ubuntu 20.04 虚拟机端的开发记录、环境快照、只读连接测试和后续 ROS / SDK 适配代码。

## Quick Start

在 Ubuntu 虚拟机中执行：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
```

ROS Noetic 工作区：

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
roslaunch carm_a3_driver readonly_state.launch
```

## Files

- `environment.yml`: conda 环境快照。
- `requirements.txt`: pip 依赖快照。
- `config/robot.yaml`: 当前机械臂的非敏感配置。
- `scripts/check_network.py`: 只做网络连通性检查。
- `scripts/inspect_carm_sdk.py`: 只导入并检查 `carm` SDK 模块。
- `carm_ws/src/carm_a3_driver`: ROS Noetic 驱动包，当前包含只读状态节点。
- `carm_ws/src/carm_a3_vision`: ROS Noetic 视觉包，当前包含原装 USB 相机 V4L2 采集节点。

## Safety

本目录中的第一阶段脚本必须保持只读：可以检查网络、导入 SDK、读取模块信息、读取设备状态；不得默认下发任何运动、回零、使能或夹爪动作。
