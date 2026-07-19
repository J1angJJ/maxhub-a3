# carm_a3_motion

Safety-gated motion services for CArm / MAXHUB A3 on ROS Noetic.

This package is intentionally separate from `carm_a3_driver`. The driver can stay useful for read-only state publishing, while this package owns commands that can change the physical robot state.

Current recommended path:

- Use `py_ws_motion.launch` for day-to-day motion tests. It uses the vendor pure Python WebSocket SDK and has been verified to send `TASK_MOVJ`.
- Use `raw_ws_motion.launch` when you want an in-repo WebSocket implementation that does not depend on the vendor Python class.
- Keep `safe_motion.launch` and `official_topic_motion.launch` as C++ SDK diagnostics only. On the current test setup, both crash inside the C++ SDK `move_joint()` real-motion call.

Default launch settings are conservative:

- `dry_run: true`
- `allow_motion: false`
- `allow_ready: false`
- `allow_servo_enable: false`
- per-joint jog limit: `0.03 rad`
- full joint move per-command delta limit: `0.15 rad`
- `set_speed_before_motion: false`
- `use_duration: false`
- `wait_for_motion: false`

## Build

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

## Safe Start

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_motion safe_motion.launch
```

Check status:

```bash
rosservice call /carm_a3/motion/status
```

## First Dry-run Jog

Default launch should block jog commands at `allow_motion=false`. To exercise the command path without physical motion, keep `dry_run=true` and explicitly open the motion gate:

```bash
roslaunch carm_a3_motion safe_motion.launch allow_motion:=true dry_run:=true
```

```bash
rosservice call /carm_a3/motion/jog_joint "{joint_index: 1, delta_rad: 0.01, duration_s: 2.0}"
```

The response should say that the command was accepted as a dry run.

## Real Jog Test

Only run this after the arm is physically clear, the original teach pendant is accessible, and the emergency stop button is reachable.

```bash
roslaunch carm_a3_motion safe_motion.launch allow_motion:=true dry_run:=false
```

Then send a tiny jog:

```bash
rosservice call /carm_a3/motion/jog_joint "{joint_index: 1, delta_rad: 0.01, duration_s: 2.0}"
```

Emergency stop service:

```bash
rosservice call /carm_a3/motion/emergency_stop
```

This node does not automatically call `set_ready()` or servo enable before motion. If the controller is not already in a safe position-control state, motion services return an error.

It also does not call `set_speed_level()` by default. Use the current controller/teach-pendant speed for the first real jog. If later testing proves the SDK speed call is stable on this controller firmware, `set_speed_before_motion` can be enabled explicitly.

The first real-motion path intentionally matches the official ROS1 demo style: `move_joint(target, -1, false)`. Duration and synchronous waiting are disabled by default because early testing showed SDK crashes in some blocking/control helper paths.

## Official Topic Compatibility Test

If `safe_motion_node` crashes inside `move_joint()`, use this node to test the official ROS1 demo calling style with as little local wrapping as possible. It subscribes to a topic and calls:

```cpp
move_joint(msg->position, -1, false)
```

Start with the motion gate closed:

```bash
roslaunch carm_a3_motion official_topic_motion.launch
rosservice call /carm_a3/official_motion/status
```

Then open only the topic motion gate:

```bash
roslaunch carm_a3_motion official_topic_motion.launch allow_move_joint:=true
```

Publish the current near-zero pose plus a tiny joint-1 jog:

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

If this node also crashes immediately after `official_topic_motion calling move_joint(position, -1, false)`, the failure is very likely inside the C++ SDK real-motion call path on this controller/firmware, not in the ROS service wrapper.

## C++ SDK Motion Crash Notes

Observed behavior:

- C++ read-only SDK calls work: connect, status, joint position, `/joint_states`, and TF publication.
- `safe_motion_node` crashes with `exit code -11` inside `move_joint(target, -1, false)`.
- `official_topic_motion_node`, which mirrors the official ROS1 demo call `move_joint(msg->position, -1, false)`, crashes at the same point.
- The same robot state reports `connected=true`, `servo=true`, `state=0`, and `fsm_state=1/POSITION`.
- Python WebSocket `TASK_MOVJ` is accepted and task completion is reported, with visible small motion.

Possible causes to rule out locally:

- Runtime library mismatch or stale SDK files. Re-source `workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash`, clean rebuild, and verify the node uses the vendored `.so` files in `workspace/ubuntu/carm_ws/vendor/arm_control_sdk/lib` and `poco/lib`.
- ABI or dependency conflict from another `LD_LIBRARY_PATH` entry. Test in a fresh terminal with only `/opt/ros/noetic/setup.bash`, the vendored SDK setup, and this workspace setup sourced.
- Multiple clients sending conflicting real-motion commands. Stop the C++ motion nodes before starting the WebSocket motion node, and keep only one motion client active.

What looks more like an upstream SDK issue:

- The crash happens inside the official C++ call shape, after all local safety checks and after current joint state was read successfully.
- It reproduces in the stripped official-topic compatibility node, not just in the service wrapper.
- The controller accepts the equivalent WebSocket/Python `TASK_MOVJ` command.

This is good evidence to send to vendor support: C++ SDK state APIs are usable, C++ SDK real-motion `move_joint()` segfaults, while WebSocket `TASK_MOVJ` works on the same controller state.

## Python WebSocket Motion Test

If both C++ motion nodes crash inside `move_joint()`, use the pure Python WebSocket path. This node loads the vendor `carm.py` file directly and sends the same `TASK_MOVJ` command that the web/ Python SDK uses, avoiding the C++ `move_joint()` shared-library path.

Install dependency if needed:

```bash
pip3 install --user websocket-client
```

Build after pulling this package:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

Start with the motion gate closed:

```bash
roslaunch carm_a3_motion py_ws_motion.launch
rosservice call /carm_a3/py_motion/status
```

Dry-run the WebSocket target construction:

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=true
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

Real tiny jog:

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=false
rosservice call /carm_a3/py_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

If `use_sdk_clip:=true` fails before sending, try the raw JSON path:

```bash
roslaunch carm_a3_motion py_ws_motion.launch allow_motion:=true dry_run:=false use_sdk_clip:=false
```

## Raw WebSocket Motion Test

This is the in-repo WebSocket implementation. It does not import the vendor `Carm` class, but sends the same JSON command to `ws://192.168.31.60:8090`.

Start:

```bash
roslaunch carm_a3_motion raw_ws_motion.launch
rosservice call /carm_a3/raw_motion/status
```

Dry-run:

```bash
roslaunch carm_a3_motion raw_ws_motion.launch allow_motion:=true dry_run:=true
rosservice call /carm_a3/raw_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```

Real tiny jog:

```bash
roslaunch carm_a3_motion raw_ws_motion.launch allow_motion:=true dry_run:=false
rosservice call /carm_a3/raw_motion/jog_joint "{joint_index: 1, delta_rad: 0.005, duration_s: 2.0}"
```
