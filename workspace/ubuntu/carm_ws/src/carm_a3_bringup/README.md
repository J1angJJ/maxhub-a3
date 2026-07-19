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
- `carm_a3_vision` USB camera node with calibrated `640x480` camera info.
- `flange -> carm_a3_camera_optical_frame` static hand-eye TF.

## Hand-eye Validation

Start the default chain plus the ArUco detector:

```bash
roslaunch carm_a3_bringup handeye_validation.launch
```

Then inspect a fixed marker:

```bash
rosrun tf tf_echo base_link aruco_marker
```
