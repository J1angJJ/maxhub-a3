# carm_a3_motion

Safety-gated motion services for CArm / MAXHUB A3 on ROS Noetic.

This package is intentionally separate from `carm_a3_driver`. The driver can stay useful for read-only state publishing, while this package owns commands that can change the physical robot state.

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
