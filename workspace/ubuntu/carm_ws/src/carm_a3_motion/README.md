# carm_a3_motion

Safety-gated motion services for CArm / MAXHUB A3 on ROS Noetic.

This package is the unified robot-facing SDK node. It owns the single recommended SDK connection, publishes read-only robot state, and exposes safety-gated command services. The older `carm_a3_driver` package is kept only as a compatibility fallback.

Current recommended path:

- Use `safe_motion.launch` for C++ SDK motion tests, but enable the official-style initialization path before real motion.
- Use `py_ws_motion.launch` as the known-good WebSocket fallback. It uses the vendor pure Python WebSocket SDK and has been verified to send `TASK_MOVJ`.
- Use `raw_ws_motion.launch` when you want an in-repo WebSocket implementation that does not depend on the vendor Python class.
- Keep `official_topic_motion.launch` as the stripped C++ SDK compatibility test for reproducing vendor demo behavior.

Default launch settings are conservative:

- `publish_state: true`
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

Read published state and extended SDK state:

```bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /maxhub_a3/flange_pose
rostopic echo -n 1 /maxhub_a3/diagnostics
rosrun carm_a3_motion motion_cli.py extended
rosrun carm_a3_motion motion_cli.py tool-info 0
```

The node publishes `/joint_states`, `/maxhub_a3/flange_pose`, and `/maxhub_a3/diagnostics` from the same SDK session used by motion services. Keep `publish_flange_tf:=false` when `carm_a3_description` and `robot_state_publisher` are active, so the URDF remains the source for the full `base_link -> flange` chain.

When `publish_gripper_joints:=true`, `/joint_states` also appends `joint7` and `joint8` from the SDK gripper gap. The default scale is `0.5`, mapping the reported two-finger gap to each prismatic finger joint for TF visualization.

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

Batch FK/IK are also wrapped directly from the official C++ SDK:

```bash
rosservice call /carm_a3/motion/solve_fk_array "{tool_index: 0, positions: [0,0,0,0,0,0, 0.01,0,0,0,0,0]}"
```

```bash
rosservice call /carm_a3/motion/solve_ik_array "{tool_index: 0, poses: [0.0,0.0,0.236,0.707,0.0,0.707,0.0], seed_positions: []}"
```

Array service layout:

- `solve_fk_array.positions`: flattened joint points, 6 values per point.
- `solve_fk_array.poses`: flattened poses, 7 values per point.
- `solve_ik_array.poses`: flattened poses, 7 values per point.
- `solve_ik_array.seed_positions`: empty means reuse current joints for every point; otherwise flattened seeds, 6 values per point.
- `solve_ik_array.positions`: flattened joint solutions, 6 values per point.
- `point_success[i]` reports whether point `i` solved; failed points keep the fixed layout with `NaN` values.

The same services can be called with the helper CLI:

```bash
rosrun carm_a3_motion motion_cli.py snapshot
rosrun carm_a3_motion motion_cli.py cart
rosrun carm_a3_motion motion_cli.py fk "0,0,0,0,0,0"
rosrun carm_a3_motion motion_cli.py fk-array "0,0,0,0,0,0,0.01,0,0,0,0,0"
rosrun carm_a3_motion motion_cli.py ik-current
rosrun carm_a3_motion motion_cli.py ik-probe
rosrun carm_a3_motion motion_cli.py ik-offset 0.01 0 0
rosrun carm_a3_motion motion_cli.py ik "0.25,0,0.30,0,0,0,1"
rosrun carm_a3_motion motion_cli.py ik-array "0,0,0.236,0.707,0,0.707,0"
rosrun carm_a3_motion motion_cli.py jog 1 0.005 --duration-s 2.0
```

Initial FK/IK notes:

- `fk "0,0,0,0,0,0"` has been verified to return a valid pose near `[0, 0, 0.236, 0.707, 0, 0.707, 0]`.
- `ik "0.25,0,0.30,0,0,0,1"` can fail with `inverse_kine ret=-1`. This does not prove the IK interface is broken; that pose and orientation are a large jump from the current configuration.
- Enhanced `ik-probe --include-mm` verified that IK works for current cart pose, plan pose, and FK(current joints).
- Keep the project IK convention as meters with pose order `x,y,z,qx,qy,qz,qw`.
- `wxyz` payload order fails, and millimeter-scaled positions fail.
- Official C++ SDK, pybind SDK, and pure Python WebSocket SDK all expose FK/IK. The official ROS1 demo does not expose FK/IK as ROS topics, so this package's ROS services are the project-level wrapper.
- See `docs/vendor/official_kinematics_reference.md` for the official interface notes and the PyPI pose-order inconsistency.
- `/solve_ik_array` and `/solve_fk_array` call the official C++ SDK batch APIs directly, so use them for candidate scans instead of repeated single-point service calls.
- `ik-offset dx dy dz` solves a small Cartesian offset from the current pose without moving the arm.
- `ik-offset --execute` calls `/carm_a3/motion/move_joint` after checking the IK solution's max joint delta. The service still applies its own motion safety limits and post-motion verification.

```bash
rosrun carm_a3_motion motion_cli.py ik-probe --include-mm
rosrun carm_a3_motion motion_cli.py ik-offset 0.01 0 0
rosrun carm_a3_motion motion_cli.py ik-offset 0 0 0.01
rosrun carm_a3_motion motion_cli.py ik-offset 0 0 0.01 --execute --max-joint-delta 0.05
rosrun carm_a3_motion motion_cli.py ik-offset-scan z
```

Current near-zero pose offset notes:

- `+0.01 m` in `x` failed IK.
- `+0.01 m` in `y` failed IK.
- `+0.01 m` in `z` succeeded with max joint delta about `0.029 rad`.
- `+0.01 m` in `z` has been executed successfully with `--execute --max-joint-delta 0.05`; `move_joint` returned `verified=true` with max readback error about `0.0027 rad`.
- After moving up, `-0.01 m` in `z` failed IK while holding orientation fixed. `ik-offset-scan z` showed `-0.001/-0.002/-0.005 m` are solvable; use `-0.005 m` or smaller for return steps.

## Additional SDK Wrappers

The package wraps the official C++ SDK functions that are useful for the current control stack:

- read-only: version/config, plan joint state, external tau/force, gripper state, current tool index, tool coordinate
- settings: speed level, control mode, collision config, tool index
- end effector: gripper position/force command
- motion: `move_joint`, `move_pose`, `move_line_joint`, `move_line_pose`, `move_flow_pose`
- kinematics: single and batch FK/IK

Intentionally not exposed yet:

- `track_joint()` / `track_pose()`: continuous command streaming needs a separate watchdog and rate policy.
- `move_joint_traj()`: exposed as `/carm_a3/motion/move_joint_trajectory`; the ROS node validates waypoint dimensions and adjacent joint deltas before calling the SDK trajectory API.
- `move_pose_traj()`: not exposed yet; it still needs pose waypoint validation, timing validation, and a stop policy.
- `trajectory_teach()` / `trajectory_recorder()` / `check_teach()`: useful later, but not required for the current perception-guided motion chain.

Settings are blocked by default:

```bash
roslaunch carm_a3_motion safe_motion.launch allow_settings:=true dry_run:=true
rosrun carm_a3_motion motion_cli.py set-speed 1.0 --response-level 20
rosrun carm_a3_motion motion_cli.py set-collision true --sensitivity-level 0
rosrun carm_a3_motion motion_cli.py set-tool 0
rosrun carm_a3_motion motion_cli.py set-control-mode 1
```

Gripper commands require both `allow_motion:=true` and `allow_gripper:=true`:

```bash
roslaunch carm_a3_motion safe_motion.launch allow_motion:=true allow_gripper:=true dry_run:=true
rosrun carm_a3_motion motion_cli.py set-gripper 0.04 --tau 10
```

Pose and line motion commands use the same `allow_motion` and `dry_run` gates as joint motion. They also reject targets whose translation jump from the current flange pose exceeds `max_pose_position_delta_m`.

```bash
rosrun carm_a3_motion motion_cli.py move-pose "0,0,0.246,0.707,0,0.707,0"
rosrun carm_a3_motion motion_cli.py move-line-joint "0.01,0,0,0,0,0"
rosrun carm_a3_motion motion_cli.py move-line-pose "0,0,0.246,0.707,0,0.707,0"
rosrun carm_a3_motion motion_cli.py move-flow-pose "0,0,0.246,0.707,0,0.707,0"
```

For real pose motion, prefer the existing IK path first:

```bash
rosrun carm_a3_motion motion_cli.py ik-offset 0 0 0.005
rosrun carm_a3_motion motion_cli.py ik-offset 0 0 0.005 --execute --max-joint-delta 0.03
```

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
