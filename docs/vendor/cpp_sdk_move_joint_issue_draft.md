# C++ SDK async `move_joint` may segfault before full ready/callback initialization

## Environment

- OS: Ubuntu 20.04
- ROS: Noetic
- Controller/model reported by SDK/WebSocket: `A3_DM_C`
- Controller: `192.168.31.60:8090`
- SDK location in test workspace: `workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## Summary

`CArmSingleCol::move_joint(target, -1, false)` can trigger a `SIGSEGV` inside `libarm_control_sdk.so` if it is called after connection but before the full initialization sequence used by the official ROS1 demo.

After matching the official ROS1 demo sequence, the same async `move_joint` call works:

1. Create `CArmSingleCol`.
2. Wait about 1 second.
3. Call `set_ready()`.
4. Register joint, pose, error, and task-completion callbacks.
5. Call `move_joint(target, -1, false)`.

With that sequence, the node no longer crashes and the arm makes a tiny visible move.

## What Works

- `connect()`
- `get_status()`
- `get_joint_pos()`
- Read-only ROS state publishing
- Python/WebSocket `TASK_MOVJ`
- C++ async `move_joint(target, -1, false)` after `set_ready()` and callback registration

## Crash Reproduction

Start a ROS node that connects to the controller and directly calls:

```cpp
CArmSingleCol::move_joint(target, -1, false);
```

without first calling `set_ready()` and without registering SDK callbacks.

Observed result:

```text
Thread 1 received signal SIGSEGV, Segmentation fault.
0x00007ffff7d1dbab in carm::CArmKernelImpl::move_joint(...)
from .../vendor/arm_control_sdk/lib/libarm_control_sdk.so

#0 carm::CArmKernelImpl::move_joint(...)
#1 carm::CArmSingleCol::move_joint(std::vector<double> const&, double, bool)
#2 ROS topic/service callback
```

The process links against the vendored SDK libraries:

```text
libarm_control_sdk.so
libtcp_com.so
libcarm_poco_net.so
libjsoncpp.so.1
libPocoNet.so.71
libPocoUtil.so.71
libPocoFoundation.so.71
libPocoXML.so.71
libPocoJSON.so.71
```

## Expected Behavior

If the SDK session is not ready for motion, `move_joint()` should return an error code or a clear diagnostic instead of crashing the process.

## Related Local Notes

Full local report and GDB output are archived in:

```text
docs/vendor/cpp_sdk_move_joint_gdb_report.md
docs/vendor/cpp_sdk_move_joint_gdb_full.txt
```
