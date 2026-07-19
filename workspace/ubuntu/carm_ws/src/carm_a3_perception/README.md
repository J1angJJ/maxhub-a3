# carm_a3_perception

Traditional perception package for the CArm / MAXHUB A3 camera.

The first node detects fixed-size red and green blocks with HSV segmentation. It does not command the robot.

## Run

Start the camera first:

```bash
roslaunch carm_a3_vision camera.launch
```

Start color block segmentation:

```bash
roslaunch carm_a3_perception color_blocks.launch
```

Inspect outputs:

```bash
rostopic echo -n 1 /carm_a3/perception/color_blocks
rostopic echo -n 1 /carm_a3/perception/color_blocks/diagnostics
rosrun image_view image_view image:=/carm_a3/perception/color_blocks/debug_image
```

The detection topic is `std_msgs/String` containing JSON. Each detection includes:

- `color`: `red` or `green`
- `center_px`: pixel center `[u, v]`
- `corners_px`: four corners from OpenCV `minAreaRect`
- `area_px`: contour area
- `rect_size_px`: min-area rectangle dimensions
- `angle_deg`: rectangle angle
- `confidence`: simple geometry score

## Notes

This is a bootstrap detector. It is useful for lighting checks, dataset collection, and early grasp-planning experiments. It does not estimate full 3D pose yet; use it first to confirm repeatable 2D detection before adding PnP, table-plane projection, or YOLO.
