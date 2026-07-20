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
- `corners_px`: four robust rectangle corners
- `corners8_px`: eight rectangle control points, ordered as corner, edge midpoint, corner, edge midpoint...
- `area_px`: contour area
- `rect_size_px`: robust rectangle dimensions
- `min_area_rect_size_px`: raw OpenCV `minAreaRect` dimensions for debugging
- `angle_deg`: rectangle angle
- `color_fill_ratio`: contour area divided by robust rectangle area
- `confidence`: simple color and geometry score

## Notes

This is a bootstrap detector. It uses HSV masks, median filtering, morphology, and a percentile-based robust rectangle fit to reduce the effect of small shadows, highlights, and chipped mask edges. It does not estimate full 3D pose yet; use it first to confirm repeatable 2D detection before adding PnP, table-plane projection, or YOLO.
