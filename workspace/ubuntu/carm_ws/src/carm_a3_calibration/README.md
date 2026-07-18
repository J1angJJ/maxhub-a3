# carm_a3_calibration

ROS Noetic calibration helpers for CArm / MAXHUB A3.

The first tool is an eye-in-hand ArUco sample recorder. It detects the printed `DICT_ARUCO_ORIGINAL` marker ID `23`, estimates `camera_T_marker`, reads `base_link -> flange` from TF, and saves one YAML file per sample.

It also publishes a live `carm_a3_camera_optical_frame -> aruco_marker` TF for validation. It does not move or command the robot.

## Install

```bash
sudo apt update
sudo apt install python3-opencv python3-yaml ros-noetic-tf2-ros
```

## Run

Start `roscore`, `carm_a3_driver`, and `carm_a3_vision` first. Then:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_calibration aruco_handeye_sampler.launch
```

## Save Samples

Keep the ArUco marker visible and call:

```bash
rosservice call /carm_a3/handeye/save_sample
```

Samples are saved by default under:

```text
workspace/ubuntu/logs/handeye_samples
```

The `logs/` directory is ignored by Git. Keep raw samples private until they are reviewed and summarized.

## Solve

After collecting samples, run:

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
rosrun carm_a3_calibration solve_handeye.py \
  --samples-dir /home/noetic/maxhub-a3/workspace/ubuntu/logs/handeye_samples
```

The script writes:

```text
workspace/ubuntu/logs/handeye_samples/handeye_result.yaml
```

The recommended result is `flange -> carm_a3_camera_optical_frame`.

The default primary method is `PARK`. Compare `all_methods` and `method_consistency_to_recommended`; PARK and HORAUD should usually be close if the samples are healthy. Large method disagreement means the transform should be treated as a draft and verified physically.

The current draft result is stored in:

```text
config/handeye_flange_camera.yaml
```

Publish it as a static TF for physical validation:

```bash
roslaunch carm_a3_calibration publish_handeye_tf.launch
```

Then inspect:

```bash
rosrun tf tf_echo flange carm_a3_camera_optical_frame
rosrun tf tf_echo base_link carm_a3_camera_optical_frame
```

For live marker validation, also run the ArUco sampler with the marker visible:

```bash
roslaunch carm_a3_calibration aruco_handeye_sampler.launch
rosrun tf tf_echo base_link aruco_marker
```

Keep the ArUco paper fixed in the workspace and move the robot/camera through several poses. A good draft hand-eye transform should keep `base_link -> aruco_marker` roughly stable.

## Sampling Notes

Collect at least 15 samples. Use 20-30 if possible.

Move the robot by hand or through safe manual controls only. Use varied poses:

- Different X/Y/Z positions.
- Different wrist rotations.
- Marker visible near the image center and near edges.
- Avoid nearly identical poses.
- Avoid blur and strong reflections.
