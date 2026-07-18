# carm_a3_vision

ROS Noetic vision package for the original CArm / MAXHUB A3 USB camera.

The first node reads `/dev/video0` through V4L2, requests YUYV format, converts frames to `rgb8`, applies software orientation correction, loads camera calibration through `camera_info_manager`, and publishes ROS image topics. It intentionally avoids OpenCV, `cv_bridge`, `image_transport`, `usb_cam`, and `v4l2_camera` package dependencies so it can run in a minimal ROS environment.

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
rostopic echo -n 1 /carm_a3/camera/camera_info
rosservice list | grep /carm_a3/camera/set_camera_info
rosrun image_view image_view image:=/carm_a3/camera/image_raw
```

If `image_view` is not installed, use `rostopic hz` first and install visualization tools later.

## Calibration

The node advertises `/carm_a3/camera/set_camera_info` so ROS `camera_calibration` can run:

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 \
  --square 0.025 \
  image:=/carm_a3/camera/image_raw \
  camera:=/carm_a3/camera
```

The default launch file loads:

```text
config/camera_info.yaml
```

The current file is calibrated for `640x480`. Uploaded calibration data from `/carm_a3/camera/set_camera_info` is accepted through `camera_info_manager`; use the calibrator's `SAVE` output as the persistent calibration file, then place the YAML under `config/camera_info.yaml`.

## Camera Modes

The original USB camera reports these modes through `v4l2-ctl -d /dev/video0 --list-formats-ext`:

| Format | Resolution | FPS |
| --- | --- | --- |
| MJPG | 320x240 | 30 |
| MJPG | 640x480 | 30 |
| MJPG | 800x600 | 30 |
| MJPG | 1024x768 | 30 |
| MJPG | 1280x720 | 30 |
| MJPG | 1280x1024 | 30 |
| MJPG | 1920x1080 | 30 |
| YUYV | 320x240 | 30 |
| YUYV | 640x480 | 30 |
| YUYV | 800x600 | 21 |
| YUYV | 1024x768 | 6 |
| YUYV | 1280x720 | 9 |
| YUYV | 1280x1024 | 6 |
| YUYV | 1920x1080 | 6 |

This package currently supports YUYV input. Keep `640x480 YUYV 30fps` as the default calibrated mode. Higher-resolution 30fps modes require MJPG decoding or a mature ROS camera driver package.

Camera intrinsics are resolution-specific. The current `config/camera_info.yaml` is for `640x480`; calibrate again after changing resolution.

## Orientation

The camera currently appears upside down in `guvcview`. The default config sets `rotate_180: true`. If the image becomes correct after physical adjustment, set it to `false` in `config/camera.yaml`.
