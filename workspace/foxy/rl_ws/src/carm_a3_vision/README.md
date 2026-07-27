# carm_a3_vision

这是原 ROS1 `carm_a3_vision` USB 相机节点的 ROS2 Foxy 迁移版。

节点通过 V4L2 读取机械臂配套 USB 相机，请求 YUYV 格式，把图像转换为 `rgb8`，按当前安装姿态做软件方向修正，并通过 `camera_info_manager` 加载标定参数。默认发布 `/carm_a3/camera/image_raw`、`/carm_a3/camera/camera_info` 和 `/carm_a3/camera/diagnostics`。

默认设备使用稳定的 by-id 路径：

```text
/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0
```

## 编译

```bash
cd /workspace/rl_ws
colcon build --symlink-install --packages-select carm_a3_vision
source install/setup.bash
```

## 启动

宿主机进入带相机映射的容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose -f compose.yaml -f compose.camera.yaml run --rm foxy-maxhub-a3 bash
```

容器内启动节点：

```bash
cd /workspace/rl_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch carm_a3_vision camera.launch.py
```

## 检查

```bash
ros2 topic hz /carm_a3/camera/image_raw
ros2 topic echo --once /carm_a3/camera/diagnostics
ros2 topic echo --once /carm_a3/camera/camera_info
ros2 service list | grep /carm_a3/camera/set_camera_info
ros2 run image_tools showimage --ros-args -r image:=/carm_a3/camera/image_raw
```

不打开图形界面时，可以采样少量帧并保存为 PPM：

```bash
ros2 run carm_a3_vision capture_image_sample \
  --count 30 \
  --save-every 30 \
  --output-dir /workspace/rl_ws/artifacts/camera_samples
```

这个工具只订阅已经发布的 ROS 图像话题，不直接访问 `/dev/video*`。相机节点里的默认配置已经保持 ROS1 的倒置修正：`rotate_180: true`。

## 标定

节点提供 `/carm_a3/camera/set_camera_info` 服务，后续可以用 ROS 相机标定工具写入内参。默认 launch 文件加载：

```text
config/camera_info.yaml
```

当前标定文件对应 `640x480`。如果修改分辨率，需要重新标定。

## 相机模式

ROS1 记录中这台 USB 相机支持 YUYV `640x480@30fps`，这里也保持这个默认标定模式。更高分辨率的 30fps 通常需要 MJPG 解码，后续如果要上视觉观测训练，可以再评估是否切换成熟 ROS 相机驱动。
