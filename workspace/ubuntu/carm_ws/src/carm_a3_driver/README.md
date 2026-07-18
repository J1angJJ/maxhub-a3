# carm_a3_driver

ROS Noetic driver package for CArm / MAXHUB A3.

This package should contain the robot-facing ROS interface: SDK connection, state publishers, diagnostics, and later carefully gated motion or gripper command nodes.

The first node is read-only. It may connect to the robot, read state, publish `/joint_states`, publish `/maxhub_a3/flange_pose`, and broadcast the minimal `base_link -> flange` TF. It must not enable, reset, move, stop, or control the gripper.

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
