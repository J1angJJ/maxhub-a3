# carm_a3_motion

Safety-gated motion services for CArm / MAXHUB A3 on ROS Noetic.

This package is intentionally separate from `carm_a3_driver`. The driver can stay useful for read-only state publishing, while this package owns commands that can change the physical robot state.

Current recommended path:

- Use `safe_motion.launch` for C++ SDK motion tests, but enable the official-style initialization path before real motion.
- Use `py_ws_motion.launch` as the known-good WebSocket fallback. It uses the vendor pure Python WebSocket SDK and has been verified to send `TASK_MOVJ`.
- Use `raw_ws_motion.launch` when you want an in-repo WebSocket implementation that does not depend on the vendor Python class.
- Keep `official_topic_motion.launch` as the stripped C++ SDK compatibility test for reproducing vendor demo behavior.

Default launch settings are conservative:

- `dry_run: true`
- `allow_motion: false`
- `allow_ready: false`
- `allow_servo_enable: false`
- `auto_ready_on_connect: false`
- `register_callbacks_on_connect: false`
- per-joint jog limit: `0.03 rad`
- full joint move per-command delta limit: `0.15 rad`
- `set_speed_before_motion: false`
- `use_duration: false`
- `wait_for_motion: false`
- `verify_after_motion: true`
- `verify_joint_tolerance_rad: 0.003`

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
roslaunch carm_a3_motion safe_motion.launch \
  allow_motion:=true \
  dry_run:=false \
  auto_ready_on_connect:=true \
  register_callbacks_on_connect:=true \
  pre_ready_delay_s:=1.0
```

Then send a tiny jog:

```bash
rosservice call /carm_a3/motion/jog_joint "{joint_index: 1, delta_rad: 0.01, duration_s: 2.0}"
```

The service reads back joint state after the SDK command. A successful real-motion response should contain `move_ret=1`, `verified=true`, `max_error=...`, `target=[...]`, and `actual=[...]`.

## Joint Snapshot, FK, and IK

The C++ service node also exposes read-only helpers for upper-level planners:

```bash
rosservice call /carm_a3/motion/get_joint_snapshot
rosservice call /carm_a3/motion/get_cartesian_snapshot
```

Forward kinematics, using joint values in radians:

```bash
rosservice call /carm_a3/motion/solve_fk "{tool_index: 0, positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Inverse kinematics, using pose order `x,y,z,qx,qy,qz,qw`. Leave `seed_positions` empty to use the current robot joints as the reference solution:

```bash
rosservice call /carm_a3/motion/solve_ik "{tool_index: 0, pose: [0.25, 0.0, 0.30, 0.0, 0.0, 0.0, 1.0], seed_positions: []}"
```

These helpers do not move the arm. To execute an IK result, inspect the returned `positions` first, then pass it to `/carm_a3/motion/move_joint`.

The same services can be called with the helper CLI:

```bash
rosrun carm_a3_motion motion_cli.py snapshot
rosrun carm_a3_motion motion_cli.py cart
rosrun carm_a3_motion motion_cli.py fk "0,0,0,0,0,0"
rosrun carm_a3_motion motion_cli.py ik-current
rosrun carm_a3_motion motion_cli.py ik-probe
rosrun carm_a3_motion motion_cli.py ik "0.25,0,0.30,0,0,0,1"
rosrun carm_a3_motion motion_cli.py jog 1 0.005 --duration-s 2.0
```

Initial FK/IK notes:

- `fk "0,0,0,0,0,0"` has been verified to return a valid pose near `[0, 0, 0.236, 0.707, 0, 0.707, 0]`.
- `ik "0.25,0,0.30,0,0,0,1"` can fail with `inverse_kine ret=-1`. This does not prove the IK interface is broken; that pose and orientation are a large jump from the current configuration.
- `ik-current` can also fail on the current near-zero/singular configuration. In that case run `ik-probe`; it tests current cart pose, plan pose, FK(current joints), quaternion sign variants, and tool indices `0,1,2,3`.
- If every `ik-probe` candidate fails, treat the vendor IK path as suspect for the current configuration and continue joint-space control first. Revisit Cartesian IK from a less singular pose.

Emergency stop service:

```bash
rosservice call /carm_a3/motion/emergency_stop
```

This command intentionally follows the vendor ROS1 initialization shape before accepting real motion: wait briefly, call `set_ready()`, and register joint, pose, error, and task-completion callbacks. The default launch still keeps this disabled so that a plain launch never changes controller state.

Post-motion verification is enabled by default. It polls `get_joint_pos()` until the target is reached within `verify_joint_tolerance_rad` or `verify_timeout_s` expires. The SDK command can still return `move_ret=1`; the ROS service returns failure if the readback does not converge.

It also does not call `set_speed_level()` by default. Use the current controller/teach-pendant speed for the first real jog. If later testing proves the SDK speed call is stable on this controller firmware, `set_speed_before_motion` can be enabled explicitly.

The first real-motion path intentionally matches the official ROS1 demo style: `move_joint(target, -1, false)`. Duration and synchronous waiting are disabled by default.

## Official Topic Compatibility Test

Use this node to test the official ROS1 demo calling style with as little local wrapping as possible. It subscribes to a topic and calls:

```cpp
move_joint(msg->position, -1, false)
```

Start with the motion gate closed:

```bash
roslaunch carm_a3_motion official_topic_motion.launch
rosservice call /carm_a3/official_motion/status
```

Then open the topic motion gate with the official-style initialization enabled:

```bash
roslaunch carm_a3_motion official_topic_motion.launch \
  allow_move_joint:=true \
  auto_ready:=true \
  register_callbacks:=true \
  pre_ready_delay_s:=1.0
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

Earlier testing showed that this same node could crash immediately after `official_topic_motion calling move_joint(position, -1, false)` when launched without `set_ready()` and callback registration. With the full initialization flags above, the node no longer crashed and the arm made a tiny visible move.

## C++ SDK Motion Crash Notes

Observed behavior before full initialization was added:

- C++ read-only SDK calls work: connect, status, joint position, `/joint_states`, and TF publication.
- `safe_motion_node` crashed with `exit code -11` inside `move_joint(target, -1, false)`.
- `official_topic_motion_node`, which mirrors the official ROS1 demo call `move_joint(msg->position, -1, false)`, crashed at the same point when started without `set_ready()` and SDK callbacks.
- The same robot state reports `connected=true`, `servo=true`, `state=0`, and `fsm_state=1/POSITION`.
- Python WebSocket `TASK_MOVJ` is accepted and task completion is reported, with visible small motion.

Possible causes to rule out locally:

- Runtime library mismatch or stale SDK files. Re-source `workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash`, clean rebuild, and verify the node uses the vendored `.so` files in `workspace/ubuntu/carm_ws/vendor/arm_control_sdk/lib` and `poco/lib`.
- ABI or dependency conflict from another `LD_LIBRARY_PATH` entry. Test in a fresh terminal with only `/opt/ros/noetic/setup.bash`, the vendored SDK setup, and this workspace setup sourced.
- Multiple clients sending conflicting real-motion commands. Stop the C++ motion nodes before starting the WebSocket motion node, and keep only one motion client active.

Follow-up result:

- After adding the vendor-style sequence, `official_topic_motion_node` was retested with `auto_ready:=true`, `register_callbacks:=true`, and `pre_ready_delay_s:=1.0`.
- The C++ node did not crash.
- The arm made a tiny visible move.

Current interpretation:

- The C++ SDK motion path appears to require the vendor initialization sequence before asynchronous `move_joint(..., -1, false)`.
- Missing that sequence should still return an error instead of causing `SIGSEGV`, so the old crash report remains useful for an upstream robustness issue.
- The controller also accepts the equivalent WebSocket/Python `TASK_MOVJ` command.

This is good evidence to send to vendor support: C++ SDK state APIs are usable; C++ async `move_joint()` can segfault if called before the full ready/callback setup; with the official initialization sequence it works; WebSocket `TASK_MOVJ` also works on the same controller state.

The first crash reproduction did not run the complete vendor ROS1 initialization flow. To test that path:

```bash
roslaunch carm_a3_motion official_topic_motion.launch \
  allow_move_joint:=true \
  auto_ready:=true \
  register_callbacks:=true \
  pre_ready_delay_s:=1.0
```

This calls `set_ready()` after a short delay and registers joint, pose, error, and completion callbacks before accepting `move_joint`.

Vendor-facing report draft:

```text
docs/vendor/cpp_sdk_move_joint_gdb_report.md
docs/vendor/cpp_sdk_move_joint_gdb_full.txt
```

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
