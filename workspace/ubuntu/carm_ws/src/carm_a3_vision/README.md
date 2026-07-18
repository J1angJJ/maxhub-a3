# carm_a3_vision

ROS Noetic vision package for the original CArm / MAXHUB A3 USB camera.

The first node reads `/dev/video0` through V4L2, requests YUYV format, converts frames to `rgb8`, applies software orientation correction, and publishes ROS image topics. It intentionally avoids OpenCV, `cv_bridge`, `image_transport`, `usb_cam`, and `v4l2_camera` package dependencies so it can run in a minimal ROS environment.

## Build

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
cd workspace/ubuntu/carm_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
cd /home/noetic/maxhub-a3
source /opt/ros/noetic/setup.bash
source workspace/ubuntu/carm_ws/devel/setup.bash
roslaunch carm_a3_vision camera.launch
```

## Inspect

```bash
source /opt/ros/noetic/setup.bash
source /home/noetic/maxhub-a3/workspace/ubuntu/carm_ws/devel/setup.bash
rostopic hz /carm_a3/camera/image_raw
rostopic echo -n 1 /carm_a3/camera/diagnostics
rosrun image_view image_view image:=/carm_a3/camera/image_raw
```

If `image_view` is not installed, use `rostopic hz` first and install visualization tools later.

## Orientation

The camera currently appears upside down in `guvcview`. The default config sets `rotate_180: true`. If the image becomes correct after physical adjustment, set it to `false` in `config/camera.yaml`.
