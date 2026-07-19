# C++ SDK `move_joint` 段错误报告

本文记录本地 MAXHUB / CArm A3 环境中，厂家 C++ SDK 运动接口可稳定复现的段错误问题。

## 环境信息

- 操作系统：Ubuntu 20.04
- ROS：Noetic
- 机械臂 / 控制器 IP：`192.168.31.60`
- 控制器端口：`8090`
- SDK / WebSocket 上报型号：`A3_DM_C`
- ROS 工作区：`/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws`
- SDK 路径：`/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## 库加载情况

执行：

```bash
ldd devel/lib/carm_a3_motion/official_topic_motion_node | grep -E "arm_control|tcp_com|Poco|jsoncpp"
```

结果显示相关库均来自仓库内 vendored SDK 目录：

```text
libarm_control_sdk.so => .../vendor/arm_control_sdk/lib/libarm_control_sdk.so
libjsoncpp.so.1 => .../vendor/arm_control_sdk/lib/libjsoncpp.so.1
libtcp_com.so => .../vendor/arm_control_sdk/lib/libtcp_com.so
libcarm_poco_net.so => .../vendor/arm_control_sdk/lib/libcarm_poco_net.so
libmlog.so => .../vendor/arm_control_sdk/lib/libmlog.so
libPocoNet.so.71 => .../vendor/arm_control_sdk/poco/lib/libPocoNet.so.71
libPocoUtil.so.71 => .../vendor/arm_control_sdk/poco/lib/libPocoUtil.so.71
libPocoFoundation.so.71 => .../vendor/arm_control_sdk/poco/lib/libPocoFoundation.so.71
libPocoXML.so.71 => .../vendor/arm_control_sdk/poco/lib/libPocoXML.so.71
libPocoJSON.so.71 => .../vendor/arm_control_sdk/poco/lib/libPocoJSON.so.71
```

因此目前不太像是系统库、Poco、jsoncpp 或旧版 SDK 混用导致的问题。

## 复现步骤

先启动 `roscore`，然后在另一个终端中设置 SDK 库路径，并用 `gdb` 启动官方 topic 对照节点：

```bash
cd /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export LD_LIBRARY_PATH=/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/vendor/arm_control_sdk/lib:/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/vendor/arm_control_sdk/poco/lib:$LD_LIBRARY_PATH

gdb --args devel/lib/carm_a3_motion/official_topic_motion_node \
  __name:=carm_a3_official_topic_motion \
  _allow_move_joint:=true \
  _robot_host:=192.168.31.60 \
  _robot_port:=8090
```

进入 `gdb` 后执行：

```gdb
set pagination off
run
```

发布一个很小的关节目标：

```bash
rostopic pub -1 /carm_a3/official_motion/move_joint sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
position: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []"
```

节点输出：

```text
official_topic_motion calling move_joint(position, -1, false)
```

随后进程触发 `SIGSEGV` 段错误。

崩溃后可用以下 gdb 命令保存完整日志：

```gdb
set logging file /tmp/carm_cpp_move_joint_gdb.txt
set logging on
bt
thread apply all bt
info sharedlibrary
set logging off
```

## GDB 崩溃栈

崩溃点位于厂家 C++ SDK 内部：

```text
Thread 1 "official_topic_" received signal SIGSEGV, Segmentation fault.
0x00007ffff7d1dbab in carm::CArmKernelImpl::move_joint(
    std::vector<double, std::allocator<double> > const&,
    std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >&,
    double,
    int,
    bool
) ()
from /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/vendor/arm_control_sdk/lib/libarm_control_sdk.so

#0  carm::CArmKernelImpl::move_joint(...) ()
    from .../vendor/arm_control_sdk/lib/libarm_control_sdk.so
#1  carm::CArmSingleCol::move_joint(std::vector<double, std::allocator<double> > const&, double, bool) ()
    from .../vendor/arm_control_sdk/lib/libarm_control_sdk.so
#2  OfficialTopicMotionNode::handleMoveJoint(...)
```

完整 gdb 日志包含 `bt`、`thread apply all bt` 和 `info sharedlibrary`，已归档到：

```text
docs/vendor/cpp_sdk_move_joint_gdb_full.txt
```

完整线程栈显示：

- 主线程在 `carm::CArmKernelImpl::move_joint(...)` 内部崩溃。
- SDK 通讯线程仍在 `libtcp_com.so` 中运行，包括：
  - `carm::CommunicationClient::sendRequestFunc()`
  - `carm::CommunicationClient::sendHeartbeatFunc()`
  - `carm::CommunicationClient::broadcastDataFunc()`
  - `carm::CommunicationClient::threadCycle()`
- WebSocket 接收线程阻塞在 vendored Poco / WebSocket 接收函数中，没有崩溃：
  - `Poco::Net::WebSocket::receiveFrame(...)`
  - `carm::WebSocketClient::recieveMsg[abi:cxx11]()`
- ROS 回调线程处于正常等待状态。

这说明进程已经连上控制器，SDK 通讯线程也在运行，但直接调用 C++ 真实运动接口时，在主 ROS 回调线程中同步崩溃。

## 共享库信息

完整日志中的 `info sharedlibrary` 进一步确认当前进程使用的是 vendored SDK 库：

```text
libarm_control_sdk.so   .../vendor/arm_control_sdk/lib/libarm_control_sdk.so
libjsoncpp.so.1         .../vendor/arm_control_sdk/lib/libjsoncpp.so.1
libtcp_com.so           .../vendor/arm_control_sdk/lib/libtcp_com.so
libcarm_poco_net.so     .../vendor/arm_control_sdk/lib/libcarm_poco_net.so
libmlog.so              .../vendor/arm_control_sdk/lib/libmlog.so
libPocoNet.so.71        .../vendor/arm_control_sdk/poco/lib/libPocoNet.so.71
libPocoUtil.so.71       .../vendor/arm_control_sdk/poco/lib/libPocoUtil.so.71
libPocoFoundation.so.71 .../vendor/arm_control_sdk/poco/lib/libPocoFoundation.so.71
libPocoXML.so.71        .../vendor/arm_control_sdk/poco/lib/libPocoXML.so.71
libPocoJSON.so.71       .../vendor/arm_control_sdk/poco/lib/libPocoJSON.so.71
```

ROS 相关库来自 `/opt/ros/noetic`，系统库来自 Ubuntu 系统路径，符合预期。

## 对照实验

同一控制器状态下，WebSocket 路径可以正常运动：

- C++ SDK 状态接口正常：`connect`、`get_status`、`get_joint_pos` 可用。
- WebSocket / Python `TASK_MOVJ` 返回 `Task_Recieve`。
- 能收到 task finished callback。
- 实机可以看到小幅动作。

WebSocket 返回示例：

```text
{'command': 'webRecieveTasks', 'recv': 'Task_Recieve', 'task_key': '...'}
Task finished callback received: ...
```

## 当前判断

问题目前看起来集中在 C++ SDK 的真实运动调用路径：

```cpp
CArmSingleCol::move_joint(target, -1, false)
```

该问题不太像 ROS service 封装导致，因为一个尽量贴近官方 ROS1 demo 调用方式的 stripped topic 节点也能复现同样崩溃。

## 后续待复测：完整官方初始化流程

当前 gdb 复现使用了 `_allow_move_joint:=true`，但没有启用 `_auto_ready:=true`，也没有注册任务完成和错误回调。厂家原 ROS1 节点的启动顺序更完整：

```text
创建 CArmSingleCol
等待约 1 秒
set_ready()
注册 joint / pose / error / completion 回调
接收 move_joint(..., -1, false)
```

这可能影响 SDK 内部任务状态初始化。SDK 即便未就绪也不应段错误，而应返回错误码；但为了排除本地初始化流程差异，需要复测完整官方流程。

复测命令：

```bash
gdb --args devel/lib/carm_a3_motion/official_topic_motion_node \
  __name:=carm_a3_official_topic_motion \
  _allow_move_joint:=true \
  _auto_ready:=true \
  _register_callbacks:=true \
  _pre_ready_delay_s:=1.0 \
  _robot_host:=192.168.31.60 \
  _robot_port:=8090
```

如果完整初始化后 C++ `move_joint()` 不再崩溃，则说明 SDK 的运动接口依赖 `set_ready()` 或回调注册带来的内部任务状态；仍建议厂家增强未初始化状态下的错误处理，避免段错误。

如果完整初始化后仍然崩溃，则更能确认问题位于当前 C++ SDK / 控制器固件组合的 `move_joint()` 路径。

## 希望厂家协助确认的问题

请协助确认当前 C++ SDK 版本是否兼容 `A3_DM_C` 控制器固件，以及 `CArmSingleCol::move_joint(target, -1, false)` 是否存在已知崩溃路径。相同控制器状态下，等价 WebSocket `TASK_MOVJ` 命令可以成功执行。
