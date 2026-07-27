# MAXHUB A3 ROS2/RL 阶段总结

本文档用于冻结 MAXHUB A3 在 ROS2 Foxy 与强化学习方向的阶段性成果，方便后续简历、汇报和迁移到新平台时快速回看。

## 项目定位

低成本机械臂 ROS2/RL 迁移与验证项目。

本阶段不是追求完整实机强化学习部署，而是验证一个官方 RL 支持较薄的机械臂平台能否完成从 ROS1/Windows 虚拟机开发，迁移到 Linux Docker + ROS2 Foxy + Gazebo + Gymnasium/SB3 的可复现实验链路。

## 已完成内容

- 搭建项目专属 Foxy Docker 镜像 `foxy-maxhub-a3:latest`，从共享 `ubuntu-env:foxy-user` 派生。
- 建立 ROS2 colcon 工作区 `/workspace/rl_ws`。
- 迁移官方 ROS2 demo 基础接口为 `carm_api`，默认不自动 ready，不主动运动。
- 迁移 A3 机器人描述包 `carm_a3_description`，支持 RViz2 展示和 `robot_state_publisher`。
- 建立 Gazebo Classic 11 仿真包 `carm_gazebo`，接入 `ros2_control` 与 `joint_trajectory_controller`。
- 建立轻量 Gymnasium reaching 环境 `carm_rl_env`，用于快速 PPO baseline。
- 建立 Gazebo reaching 环境 `carm_rl_gazebo`，支持训练、评估、失败 seed trace 和轨迹可视化。
- 迁移 ROS1 USB 相机节点为 `carm_a3_vision`，保留 V4L2 YUYV 采集、`rgb8` 发布、`camera_info_manager` 标定和 `rotate_180: true` 倒置修正。

## 关键技术栈

```text
OS/ROS: Ubuntu 20.04, ROS2 Foxy
Container: Docker Compose, host network, optional camera/GPU overlays
Simulation: Gazebo Classic 11, ros2_control
RL API: Gymnasium 0.29.1
RL Library: Stable-Baselines3 2.3.2
Algorithm: PPO
Modeling: URDF, robot_state_publisher, joint_trajectory_controller
Vision: V4L2, sensor_msgs/Image, camera_info_manager
```

## 代表性结果

Toy kinematics reaching：

```text
model=/workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip
episodes=100
success_threshold=0.0300
success_rate=0.9900
mean_distance=0.0254
best_distance=0.0052
worst_distance=0.1622
```

Gazebo reaching 当前主线：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip
episodes=100
success_hold_steps=3
success_threshold=0.0300
success_rate=0.7700
mean_distance=0.0312
mean_best_distance=0.0250
worst_distance=0.1549
```

当前结果说明：toy 环境已经能稳定学会 reaching；Gazebo 环境中 PPO 策略已经形成可用闭环，但仍受仿真控制误差、目标分布和到达后保持能力影响。

## 主要经验

- 从零随机策略直接在 Gazebo 里训练成本较高，先用 toy kinematics 环境得到策略，再迁移到 Gazebo 微调更快。
- Gazebo reaching 的主要困难不是“从远处接近目标”，而是“接近目标后不跑开”。`mean_best_distance` 经常明显优于最终距离。
- `progress_reward_scale`、近目标动作惩罚和 `success_hold_steps=3` 对稳定性有帮助。
- 单个 hard target replay 能提高某些失败目标成功率，但容易把失败转移到其他空间区域；多困难目标 replay 能缓和极端失败，但会牺牲整体成功率。
- 对这个平台继续深挖会逐渐变成设备支持、驱动和仿真误差治理，而不是纯算法收益。

## 当前边界

- 未完成实机 RL 闭环。
- 未实现 Foxy 侧实机安全执行层。
- 未把 Gazebo 策略下发到真实机械臂。
- 相机节点已迁移并编译通过，但真实图像采集仍需在带 camera overlay 的容器里实测。
- 未进行视觉观测训练、模仿学习数据集采集或 sim-to-real 系统验证。

## 推荐停止点

本项目建议在当前状态阶段性停止，保留为工程迁移和早期 RL 验证案例。后续如果重启，应优先补：

- 相机实测和少量样本采集。
- 实机安全执行层，包括动作限幅、速度限制、工作空间限制、急停状态检查和人工确认。
- 更标准的 expert/BC/PPO pipeline，而不是继续手动微调 hard target。

下一阶段建议切换到官方 RL demo 更完整的平台，例如 Franka Panda + MuJoCo/Gymnasium 或 Isaac Lab，把重点从设备适配转到算法实验。

## 简历表述

中文：

```text
基于 ROS2 Foxy 与 Gazebo 为 MAXHUB A3 机械臂搭建强化学习实验环境，完成 Docker 化开发环境、URDF/Gazebo 控制接入、ROS1 相机节点迁移、Gymnasium 环境封装，并使用 Stable-Baselines3 PPO 完成 reaching 任务训练与评估。通过失败 seed 回放、reward shaping 和连续保持成功判定改进策略稳定性，在 Gazebo 仿真中达到 77% 成功率。
```

English:

```text
Built a ROS2 Foxy and Gazebo-based reinforcement learning pipeline for a MAXHUB A3 robotic arm, including a reproducible Docker environment, URDF/Gazebo control integration, ROS1-to-ROS2 camera node migration, Gymnasium environment wrapping, and Stable-Baselines3 PPO training. Improved reaching policy stability with failure-seed tracing, reward shaping, and hold-based success criteria, achieving a 77% success rate in Gazebo simulation.
```

更保守版本：

```text
搭建并验证 MAXHUB A3 机械臂的 ROS2/Gazebo 强化学习原型环境，完成低维 reaching 任务 PPO baseline、Gazebo 闭环评估、轨迹诊断工具和相机节点迁移，为后续迁移到标准机器人学习平台积累了 ROS2、仿真控制和 RL 工程经验。
```

## 复现入口

进入容器：

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose run --rm foxy-maxhub-a3 bash
```

编译工作区：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

启动 Gazebo：

```bash
ros2 launch carm_gazebo spawn_a3_control.launch.py
```

评估当前主线模型：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip \
  --episodes 100 \
  --max-steps 45 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --success-hold-steps 3 \
  --progress-reward-scale 0.5 \
  --distance-regression-penalty-scale 3.0 \
  --near-target-action-penalty-scale 0.08 \
  --near-target-action-penalty-radius 0.08 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_nearstop_hold3_5000.csv \
  --device cpu
```

生成单 episode 轨迹图：

```bash
ros2 run carm_rl_gazebo trace_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip \
  --seed 3035 \
  --max-steps 45 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --success-hold-steps 3 \
  --progress-reward-scale 0.5 \
  --distance-regression-penalty-scale 3.0 \
  --near-target-action-penalty-scale 0.08 \
  --near-target-action-penalty-radius 0.08 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/trace_seed3035_hold3.csv \
  --device cpu

ros2 run carm_rl_gazebo plot_gazebo_trace \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/trace_seed3035_hold3.csv \
  --output /workspace/rl_ws/artifacts/gazebo_reaching/trace_seed3035_hold3.png
```
