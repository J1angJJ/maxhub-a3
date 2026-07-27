# 已追踪文档复查记录

日期：2026-07-27

本文档记录本次对 git 已追踪文档的复查结果。检查目标是确认文档入口、链接、隐私边界、阶段口径和可恢复性。

## 检查范围

按 git 已追踪文件筛选：

```text
README*
*.md
*.txt
requirements.txt
docs/calibration/printables/*.pdf
```

主要文档：

- 根入口：`README.md`
- 设备/Ubuntu/Noetic：`README_getting_started.md`、`README_ubuntu.md`、`README_ros_noetic.md`
- 收束 review：`docs/project_closure_review_2026-07-27.md`
- 早期 Humble 判断：`docs/ros2_humble_rl_start.md`
- 厂商 SDK 调研记录：`docs/vendor/*.md`、`docs/vendor/*.txt`
- Foxy/RL：`workspace/foxy/*.md`
- Noetic/Foxy 各 ROS 包 README
- Docker README 和 requirements
- 标定打印 PDF：`docs/calibration/printables/*.pdf`

## 执行检查

```text
git status --short --branch
git ls-files | rg '(^|/)(README[^/]*|.*\.(md|txt|rst|pdf))$'
git ls-files | rg '(^private/|^reference/|\.(csv|npz|npy|zip|bag|db3|mp4|avi|mov|pdf)$)'
Markdown 本地链接检查
git diff --check
```

额外确认：

- `private/` 未被追踪。
- `reference/` 未被追踪。
- 训练模型、CSV、bag、视频等运行产物未被追踪。
- Markdown 本地相对链接均可解析。
- 文档和源码无 trailing whitespace 等 `git diff --check` 问题。

## 发现与处理

### Humble 文档口径

`docs/ros2_humble_rl_start.md` 是早期 Ubuntu 22.04 + ROS 2 Humble 评估记录，但后续实际路线改为 Ubuntu 20.04 + ROS 2 Foxy。

处理：

- 已在文档开头加入阶段说明，明确它是历史评估记录。
- 当前 RL 冻结和主线迁移判断以 `workspace/foxy/PROJECT_SUMMARY_RL.md` 与 `docs/project_closure_review_2026-07-27.md` 为准。

### Foxy README 旧“下一步”口径

`workspace/foxy/README.md` 后半保留了早期“仿真方向下一步”文字，容易和当前冻结状态冲突。

处理：

- 已改为“仿真方向历史参考”。
- 已补充说明 Foxy/RL 已冻结，不建议继续围绕 MAXHUB A3 盲目扩展训练功能。
- 迁移边界已更新为：Foxy 已迁移基础接口、bringup、机器人描述和相机节点；手眼、抓取任务、安全运动层未整体搬运。

### Catkin 顶层 CMakeLists

`workspace/ubuntu/carm_ws/src/CMakeLists.txt` 是 catkin 工作区常见的符号链接，指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`。在宿主机未安装 Noetic 或不在容器内时会显示为 broken symlink。

判断：

- 这是 catkin 工作区预期行为，不是文档问题。
- 本轮已在 Noetic 容器内执行 `catkin_make` 并通过。

### 标定 PDF

`docs/calibration/printables/` 下两个 PDF 被追踪：

- `aruco_original_id23_marker_100mm_A4.pdf`
- `checkerboard_squares_9x7_inner_8x6_square_25mm_A4.pdf`

判断：

- 这两个是标定打印资产，不属于私有资料或运行产物，可以保留。

## 当前文档口径

当前建议以后按以下入口阅读：

1. `README.md`：总入口。
2. `docs/project_closure_review_2026-07-27.md`：全仓库收束 review。
3. `workspace/foxy/PROJECT_SUMMARY_RL.md`：Foxy/RL 阶段总结和简历表述。
4. `workspace/foxy/EXPERIMENTS.md`：Foxy/RL 实验流水。
5. `README_ros_noetic.md`：Noetic 实机和视觉工程恢复入口。

## 结论

已追踪文档当前可以留档。

注意事项：

- `README_getting_started.md` 和 `README_ros_noetic.md` 很长，保留大量历史调试信息；它们不是最终教程，而是设备工程记录。
- `docs/ros2_humble_rl_start.md` 是历史方案评估，不是当前路线。
- `workspace/foxy/README.md` 仍保留大量实验命令，最终结论应以文档开头冻结说明和 `PROJECT_SUMMARY_RL.md` 为准。
