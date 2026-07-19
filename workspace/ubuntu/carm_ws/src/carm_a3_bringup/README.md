# carm_a3_bringup

Launch composition package for CArm / MAXHUB A3 ROS workflows.

This package does not implement robot control. It starts existing state, vision, motion, and calibration nodes together.

## Read-only Vision Hand-eye

Start the current default chain:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch
```

This starts:

- `carm_a3_motion` unified SDK node with motion gates closed; it publishes `/joint_states`, `/maxhub_a3/flange_pose`, and `/maxhub_a3/diagnostics`.
- `carm_a3_description` robot model through `robot_state_publisher`.
- `carm_a3_vision` USB camera node with calibrated `640x480` camera info.
- `flange -> carm_a3_camera_optical_frame` static hand-eye TF.

The unified SDK node does not publish the minimal direct `base_link -> flange` TF in this bringup. The full robot TF chain comes from URDF + `robot_state_publisher`.

The motion node starts by default, but physical motion is disabled by default. To explicitly dry-run the safety-gated motion services:

```bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch \
  motion_allow_motion:=true \
  motion_auto_ready_on_connect:=true \
  motion_register_callbacks_on_connect:=true \
  motion_pre_ready_delay_s:=1.0 \
  motion_dry_run:=true
```

For real motion, prefer launching `carm_a3_motion` directly during first tests so the command surface is obvious. If using this bringup for real motion, keep the same official-style initialization arguments and set `motion_dry_run:=false` only after dry-run service calls look correct.

The legacy `carm_a3_driver` read-only node is still available with `launch_motion:=false launch_driver:=true` if rollback is needed.

## Hand-eye Validation

Start the default chain plus the ArUco detector:

```bash
roslaunch carm_a3_bringup handeye_validation.launch
```

Then inspect a fixed marker:

```bash
rosrun tf tf_echo base_link aruco_marker
```
