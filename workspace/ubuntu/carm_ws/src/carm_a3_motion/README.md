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
