# carm_a3_driver

ROS Noetic legacy read-only driver package for CArm / MAXHUB A3.

This package is kept as a compatibility fallback. The recommended robot-facing SDK node is now `carm_a3_motion/safe_motion_node`, which publishes read-only state and exposes safety-gated command services through one SDK connection.

The legacy node here is read-only. It may connect to the robot, read state, publish `/joint_states`, publish `/maxhub_a3/flange_pose`, and broadcast the minimal `base_link -> flange` TF. It must not enable, reset, move, stop, or control the gripper.

Future higher-level work should stay in separate packages when it grows beyond driver responsibilities:

- `carm_a3_calibration`: hand-eye calibration and camera extrinsics.
- `carm_a3_vision`: YOLO or other perception pipelines.
- `carm_a3_tasks`: simple pick-and-place coordination.
- `carm_a3_learning`: reinforcement learning experiments.

## Build

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

## Run

Prefer the unified node:

```bash
roslaunch carm_a3_motion safe_motion.launch
```

Run this legacy fallback only when comparing behavior or rolling back:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_driver readonly_state.launch
```

## Inspect Topics

```bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic echo -n 1 /maxhub_a3/diagnostics
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /maxhub_a3/flange_pose
rosrun tf tf_echo base_link flange
```
