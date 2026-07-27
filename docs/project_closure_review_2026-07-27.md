# MAXHUB A3 工程收束 Review

日期：2026-07-27

本文档用于给整个 `maxhub-a3` 仓库做阶段性 review。结论先行：本仓库已经形成两条可保留成果线，但不建议继续把 MAXHUB A3 作为主强化学习研究平台。

## 总体结论

MAXHUB A3 项目当前可以阶段性封存。

保留价值：

- 完成从 Windows/虚拟机开发路径到本机 Linux Docker 开发路径的迁移。
- Noetic 线保留了实机接入、SDK 状态读取、视觉、手眼标定和抓取任务草案。
- Foxy 线保留了 ROS2/Gazebo/Gymnasium/SB3 强化学习原型闭环。
- 文档、Docker、设备路径、图形化总结和复现实验入口已经基本可交接。

停止原因：

- 官方 RL demo、标准仿真任务和 benchmark 支持较薄。
- 继续推进会把大量时间花在驱动、仿真误差、安全执行层和视觉链路补丁上。
- 当前经历已经能证明工程迁移和早期 RL 实验能力；下一阶段应把精力转到官方 RL demo 更完备的平台。

## 图形化概览

Foxy/RL 阶段总结中的图形化材料：

- [工程收束地图](../workspace/foxy/assets/project_map.svg)
- [ROS2/RL 原型链路](../workspace/foxy/assets/rl_pipeline.svg)
- [Gazebo trace 诊断图](../workspace/foxy/assets/gazebo_trace_seed3035_action008_20000.png)
- [Gazebo rollout 动图](../workspace/foxy/assets/gazebo_reaching_rollout_seed3035.gif)
- [平台去留决策图](../workspace/foxy/assets/platform_decision.svg)

主要总结文档：

- [Foxy/RL 阶段总结](../workspace/foxy/PROJECT_SUMMARY_RL.md)
- [Foxy 实验记录](../workspace/foxy/EXPERIMENTS.md)
- [Noetic 启动说明](../README_ros_noetic.md)

## 仓库结构 Review

```text
maxhub-a3/
├── README.md
├── README_getting_started.md
├── README_ubuntu.md
├── README_ros_noetic.md
├── docs/
│   ├── migration_status_2026-07-26.md
│   ├── project_closure_review_2026-07-27.md
│   └── vendor/
└── workspace/
    ├── ubuntu/
    │   ├── docker/noetic-maxhub-a3/
    │   └── carm_ws/src/
    └── foxy/
        ├── docker/foxy-maxhub-a3/
        ├── assets/
        ├── rl_ws/src/
        ├── EXPERIMENTS.md
        └── PROJECT_SUMMARY_RL.md
```

### 根目录

状态：可保留。

根 README 已指向 Noetic、Foxy/RL、迁移记录和本 review。后续如果重启项目，应优先从根 README 进入，避免直接从旧命令片段开始。

### `workspace/ubuntu`

状态：Noetic 实机工程线保留。

主要包：

- `carm_a3_motion`：统一 SDK 会话、安全门控运动服务、只读状态。
- `carm_a3_driver`：旧只读状态节点兼容保留。
- `carm_a3_description`：URDF、mesh、TF/RViz。
- `carm_a3_vision`：V4L2 USB 相机节点，包含 `rotate_180: true` 倒置修正。
- `carm_a3_calibration`：手眼标定采样和求解。
- `carm_a3_perception`：色块分割。
- `carm_a3_tasks`：抓取任务草案。

Review 判断：

- 这条线更像“设备工程和实机 bring-up”，不是后续主 RL 平台。
- 如果未来恢复实机开发，应从 `README_ros_noetic.md` 的安全 bring-up 顺序开始。
- 不建议把新的 RL 算法代码塞回 Noetic 线。

### `workspace/foxy`

状态：Foxy/RL 原型线冻结。

主要包：

- `carm_api`：官方 ROS2 demo 薄迁移，默认不自动 ready。
- `carm_a3_description`：Foxy 版机器人模型。
- `carm_gazebo`：Gazebo Classic 11 + `ros2_control`。
- `carm_rl_env`：轻量 Gymnasium reaching baseline。
- `carm_rl_gazebo`：Gazebo reaching 训练、评估、trace、plot。
- `carm_a3_vision`：ROS2 相机节点迁移。

代表性结果：

```text
Toy reaching:
success_rate=0.9900
mean_distance=0.0254

Gazebo reaching:
model=ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip
success_hold_steps=3
success_rate=0.7700
mean_distance=0.0312
```

Review 判断：

- 该线已经证明“能跑通 ROS2/Gazebo/RL 原型”。
- 不建议继续做长训或持续 hard target 微调。
- 后续如恢复，应先补相机实测和安全执行层，而不是直接加新算法。

### `private/` 与 `reference/`

状态：本地资料，不入库。

- `private/` 保存飞书导出、产品手册、软件开发手册等资料。
- `reference/` 保存官方 demo clone。
- 二者已在 `.gitignore` 中排除。

Review 判断：

- 当前做法正确。公开提交中不要加入完整设备 SN、私发 PDF、账号、凭据、局域网敏感拓扑。

## 安全边界

当前明确没有完成：

- Foxy 侧实机安全执行层。
- Gazebo 策略到实机的限幅、速度、工作空间和急停状态检查。
- sim-to-real 校验。
- 视觉观测闭环训练。

禁止默认操作：

- 不要直接把 Gazebo PPO 策略下发到真实机械臂。
- 不要在没有人工确认和急停可触达的情况下调用运动服务。
- 不要把 Web 前端写配置接口当作普通 API 使用。

## 本轮验证

本次收束 review 执行了以下检查：

```text
git diff --check
git ls-files private reference
git ls-files | rg '\.(csv|npz|npy|zip|bag|db3|mp4|avi|mov)$'
docker compose config  # workspace/foxy/docker/foxy-maxhub-a3
docker compose config  # workspace/ubuntu/docker/noetic-maxhub-a3
colcon build --symlink-install --packages-select carm_a3_description carm_a3_vision carm_gazebo carm_rl_env carm_rl_gazebo carm_rl_bringup
catkin_make  # workspace/ubuntu/carm_ws
```

结果：

- 文档和源码无 trailing whitespace 等 `git diff --check` 问题。
- `private/`、`reference/` 未被纳入 git。
- 训练模型、CSV、bag、视频等运行产物未被纳入 git。
- Foxy compose 配置可展开。
- Noetic compose 配置可展开。
- Foxy 关键包编译通过；Python 包仅有 setuptools 旧式安装警告。
- Noetic catkin 工作区编译通过。

未执行：

- 未启动实机运动节点。
- 未启动 Gazebo 在线评估。
- 未做 USB 相机真实采集。
- 未做网络写配置或运动服务调用。

## 推荐后续方向

主 RL 研究建议迁移到标准平台：

- Franka Panda + MuJoCo/Gymnasium
- Franka Panda + Isaac Lab
- 其他官方 RL demo 完整、社区资料充足的平台

A3 后续定位：

- 工程迁移案例。
- ROS/设备接入案例。
- 低成本机械臂 bring-up 经验材料。
- 必要时作为视觉/控制接口练习平台，而不是算法主战场。

## 恢复本项目时的最小清单

如果未来需要恢复 A3：

1. 确认机械臂网页和 SDK 网络连接。
2. 从 Noetic `safe_motion.launch` 做只读状态检查。
3. 确认 USB 相机 by-id 路径和 `rotate_180` 设置。
4. 在 Foxy 容器中编译 `rl_ws`。
5. 只在 Gazebo 里复现实验，不下发到实机。
6. 实机运动前先实现安全执行层。

## 当前收束判断

可以 push 留档。

建议提交后不再继续扩展 MAXHUB A3 的 RL 功能，把后续精力切到新平台调研、环境搭建和标准任务复现。
