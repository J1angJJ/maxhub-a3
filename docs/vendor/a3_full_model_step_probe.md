# A3 整机 STEP 夹爪 TCP 草案

来源文件未放入仓库：

```text
C:\Users\JJ406\Downloads\外发模型汇总\外发模型汇总\A3整机模型_带夹爪相机hf899.stp
```

文件信息：

- 格式：STEP / ISO-10303-21
- 导出：CVTE / Creo Parametric
- 单位：mm
- 顶层产品：`ASM0003_ASM`
- 装配中包含 `00_J0` 到 `00_J6`，以及 `ASM0007_ASM`、`_HF8990001_ASM_1_ASM`、`JIAZHUA_1_1_2_3_4_1`

## 候选装配关系

整机 STEP 中可读到：

```text
ASM0003_ASM -> 00_J6_____SW0001
origin = [0.000, -13.119, 184.000] mm
axis = [0.000, 0.000, -1.000]
refdir = [-1.000, 0.000, 0.000]

ASM0003_ASM -> ASM0007_ASM
origin = [-64.891, -7.180, 219.486] mm
axis = [0.003, 0.000, 1.000]
refdir = [1.000, 0.000, -0.003]
```

暂按 `00_J6` 作为 ROS `flange` 附近参考，得到候选：

```text
flange -> gripper_base
xyz = [0.064891, 0.005939, -0.035486] m
rpy = [3.141592654, -0.002999991, 3.141592654] rad
```

`gripper_tcp` 使用夹爪本体前端的初始估计。首次草案曾使用 `+0.149 m`，实机抓取表现为停在物块上方约 `5-10 cm`，说明该本地轴向不符合当前 ROS 抓取姿态；现改为 `-0.149 m`：

```text
gripper_base -> gripper_tcp
xyz = [0, 0, -0.149] m
rpy = [0, 0, 0]
```

组合后约为：

```text
flange -> gripper_tcp
xyz = [0.065338, 0.005939, 0.113513] m
```

## 验证注意

这些数值来自 CAD 装配坐标推断，不等价于实机 TCP 标定。下一步应在 Ubuntu/ROS 中验证：

```bash
rosrun tf tf_echo flange gripper_base
rosrun tf tf_echo flange gripper_tcp
rostopic echo -n 1 /joint_states
```

如果 `00_J6` 与 ROS `flange` 的坐标定义不一致，可能出现 180 度翻转或轴向偏移。正式下探抓取前，必须用实物尺寸或低速轻触桌面确认 `gripper_tcp` 的高度和方向。

## 2026-07-20 夹指 STL 复核

二进制 STL 包围盒显示，左右夹指在各自 link 下的尺寸约为：

```text
gripper_left/right local size ~= [0.057, 0.105, 0.022] m
```

按 URDF 中 `joint7` / `joint8` 的固定原点和 RPY 变换到 `gripper_base` 后，左右夹指实体在 `gripper_base` 下约为：

```text
x = -0.0285 .. 0.0285 m
y = -0.0217 .. 0.0217 m
z =  0.0580 .. 0.1630 m
```

这些 STL 结果说明夹指局部模型可用于检查夹爪尺寸，但不能直接覆盖当前 ROS 抓取姿态里的 TCP 方向。实机和 2026-07-20 的 plan 复核表明，`gripper_base -> gripper_tcp = [0, 0, +0.1105]` 会让 `flange` 目标落到桌面以下，因此不作为默认 TCP。当前继续使用实机接近测试中可工作的方向：

```text
gripper_base -> gripper_tcp = [0, 0, -0.149] m
```

任务层默认从 TF 读取 `flange -> gripper_tcp`，抓取高度由物块高度自动计算为：

```text
tcp_grasp_z(top_safe) = table_z + block_height + tcp_top_clearance
tcp_grasp_z(center)   = table_z + block_height / 2 + tcp_center_clearance
```

这样后续高度误差应优先通过验证/修正 `gripper_tcp` 模型定义处理，而不是在抓取任务里添加固定 Z 偏移。
