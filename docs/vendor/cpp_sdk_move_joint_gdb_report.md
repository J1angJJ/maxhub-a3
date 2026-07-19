# C++ SDK `move_joint` Segmentation Fault Report

This report records a reproducible crash of the vendor C++ SDK motion API on the local MAXHUB / CArm A3 setup.

## Environment

- OS: Ubuntu 20.04
- ROS: Noetic
- Robot/controller IP: `192.168.31.60`
- Controller port: `8090`
- Robot model reported by SDK/WebSocket: `A3_DM_C`
- Workspace: `/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws`
- SDK path: `/home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## Library Resolution

`ldd devel/lib/carm_a3_motion/official_topic_motion_node | grep -E "arm_control|tcp_com|Poco|jsoncpp"` shows all relevant libraries are loaded from the vendored SDK path:

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

This makes an obvious system-library or stale-SDK mix-up unlikely.

## Reproduction

Start `roscore`, then start the official-topic compatibility node under `gdb` with the SDK library path set:

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

In `gdb`:

```gdb
set pagination off
run
```

Publish a small joint target:

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

The node prints:

```text
official_topic_motion calling move_joint(position, -1, false)
```

Then it crashes with `SIGSEGV`.

For a complete support log after the crash:

```gdb
set logging file /tmp/carm_cpp_move_joint_gdb.txt
set logging on
bt
thread apply all bt
set logging off
```

## GDB Backtrace

The crash is inside the vendor C++ SDK:

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

A full gdb log with `bt`, `thread apply all bt`, and `info sharedlibrary` is archived at:

```text
docs/vendor/cpp_sdk_move_joint_gdb_full.txt
```

The full thread dump shows:

- Thread 1 crashes in `carm::CArmKernelImpl::move_joint(...)`.
- SDK communication worker threads are alive in `libtcp_com.so`, including:
  - `carm::CommunicationClient::sendRequestFunc()`
  - `carm::CommunicationClient::sendHeartbeatFunc()`
  - `carm::CommunicationClient::broadcastDataFunc()`
  - `carm::CommunicationClient::threadCycle()`
- The WebSocket receive thread is blocked in vendored Poco/WebSocket receive functions, not crashed:
  - `Poco::Net::WebSocket::receiveFrame(...)`
  - `carm::WebSocketClient::recieveMsg[abi:cxx11]()`
- ROS callback threads are waiting normally in `roscpp`.

This supports the conclusion that the process is connected and the SDK communication machinery is running, while the direct real-motion call path crashes synchronously in the main ROS callback thread.

## Shared Libraries From Full Log

`info sharedlibrary` confirms the motion node is using vendored SDK libraries:

```text
libarm_control_sdk.so  .../vendor/arm_control_sdk/lib/libarm_control_sdk.so
libjsoncpp.so.1        .../vendor/arm_control_sdk/lib/libjsoncpp.so.1
libtcp_com.so          .../vendor/arm_control_sdk/lib/libtcp_com.so
libcarm_poco_net.so    .../vendor/arm_control_sdk/lib/libcarm_poco_net.so
libmlog.so             .../vendor/arm_control_sdk/lib/libmlog.so
libPocoNet.so.71       .../vendor/arm_control_sdk/poco/lib/libPocoNet.so.71
libPocoUtil.so.71      .../vendor/arm_control_sdk/poco/lib/libPocoUtil.so.71
libPocoFoundation.so.71 .../vendor/arm_control_sdk/poco/lib/libPocoFoundation.so.71
libPocoXML.so.71       .../vendor/arm_control_sdk/poco/lib/libPocoXML.so.71
libPocoJSON.so.71      .../vendor/arm_control_sdk/poco/lib/libPocoJSON.so.71
```

ROS and system dependencies come from `/opt/ros/noetic` and Ubuntu system paths, as expected.

## Control Experiment

The same controller state works through WebSocket:

- C++ SDK status APIs work: `connect`, `get_status`, `get_joint_pos`.
- WebSocket/Python `TASK_MOVJ` returns `Task_Recieve`.
- Task finished callback is received.
- The real robot shows a small visible motion.

Example WebSocket response:

```text
{'command': 'webRecieveTasks', 'recv': 'Task_Recieve', 'task_key': '...'}
Task finished callback received: ...
```

## Current Conclusion

The failure appears isolated to the C++ SDK real-motion path:

```cpp
CArmSingleCol::move_joint(target, -1, false)
```

The issue is unlikely to be caused by the ROS service wrapper, because a stripped topic node matching the official ROS1 demo call shape reproduces the same crash.

## Suggested Vendor Question

Please confirm whether the provided C++ SDK build is compatible with the current `A3_DM_C` controller firmware and whether `CArmSingleCol::move_joint(target, -1, false)` has a known crash path. The equivalent WebSocket `TASK_MOVJ` command succeeds on the same controller.
