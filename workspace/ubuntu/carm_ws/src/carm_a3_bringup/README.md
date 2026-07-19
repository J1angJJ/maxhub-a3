# carm_a3_bringup

Launch composition package for CArm / MAXHUB A3 ROS workflows.

This package does not implement robot control. It only starts existing read-only, vision, and calibration nodes together.

## Read-only Vision Hand-eye

Start the current default chain:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch
```

This starts:

- `carm_a3_driver` read-only state publisher.
- `carm_a3_description` robot model through `robot_state_publisher`.
- `carm_a3_vision` USB camera node with calibrated `640x480` camera info.
- `flange -> carm_a3_camera_optical_frame` static hand-eye TF.

The driver still publishes `/joint_states`, but its minimal direct `base_link -> flange` TF is disabled in this bringup. The full robot TF chain comes from URDF + `robot_state_publisher`.

Motion is optional and disabled by default. To add the safety-gated motion services without physical motion:

```bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch \
  launch_motion:=true \
  motion_allow_motion:=true \
  motion_dry_run:=true
```

For real motion, prefer launching `carm_a3_motion` directly during first tests so the command surface is obvious.

## Hand-eye Validation

Start the default chain plus the ArUco detector:

```bash
roslaunch carm_a3_bringup handeye_validation.launch
```

Then inspect a fixed marker:

```bash
rosrun tf tf_echo base_link aruco_marker
```
