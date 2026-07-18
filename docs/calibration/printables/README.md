# Calibration Printables

本目录保存可直接打印的相机与手眼标定图案。

打印时请选择“实际大小 / 100% / 不缩放”，不要使用“适合页面”。打印后建议用尺子复核关键尺寸。

## Files

- `checkerboard_squares_9x7_inner_8x6_square_25mm_A4.pdf`
  - A4 横向。
  - 图案为 `9 x 7` 个方格。
  - 内部角点为 `8 x 6`，ROS `camera_calibration` 参数使用 `--size 8x6`。
  - 单格边长为 `25 mm`，ROS 参数使用 `--square 0.025`。

- `aruco_original_id23_marker_100mm_A4.pdf`
  - A4 纵向。
  - 来源：`https://chev.me/arucogen/`。
  - 字典：`DICT_ARUCO_ORIGINAL`。
  - ID：`23`。
  - 黑色标记外边长：`100.0 mm`。
