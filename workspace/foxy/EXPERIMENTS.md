# MAXHUB A3 Foxy 实验记录

本文档记录 Foxy/RL 方向的实验进度、模型路径、关键命令和下一步计划。原则是轻量记录，避免训练结果和代码版本脱节。

## 2026-07-27 阶段冻结摘要

本阶段目标是验证 MAXHUB A3 能否迁入 Linux Docker + ROS2 Foxy，并形成一个可运行的强化学习 reaching 实验闭环。当前目标已经达到，可以作为阶段性成果冻结。

### 已完成

- 项目镜像：`foxy-maxhub-a3:latest`
- ROS2 工作区：`/workspace/rl_ws`
- 机器人描述：`carm_a3_description`
- Gazebo Classic 11 控制：`carm_gazebo`，使用 `joint_trajectory_controller` 控制 6 轴主臂
- Toy Gymnasium reaching：`carm_rl_env`
- Gazebo Gymnasium reaching：`carm_rl_gazebo`
- 训练库：Stable-Baselines3 `2.3.2`
- 算法：PPO，A2C 入口保留但非主线
- 诊断工具：evaluation CSV、single-seed trace CSV、trace PNG 可视化
- 相机迁移：ROS1 `carm_a3_vision` V4L2 节点已迁移到 Foxy，并保留 `rotate_180: true`

### 主要结果

Toy kinematics reaching：

```text
model=/workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip
episodes=100
success_threshold=0.0300
success_rate=0.9900
mean_distance=0.0254
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

### 冻结判断

该项目已经足够证明：

- 能把 ROS1/虚拟机开发路径迁到本机 Docker + ROS2 Foxy。
- 能把官方 SDK/参考 demo、URDF、Gazebo、ROS2 control 和 SB3/Gymnasium 接成可训练闭环。
- 能针对失败 seed 做 trace、reward shaping、hard target replay 和稳定成功口径评估。

但不建议继续把它作为长期主 RL 平台：

- 官方 RL demo 和标准 benchmark 支持薄，需要持续自建环境、调接口和补诊断。
- Gazebo 与实机控制链路之间仍缺安全执行层、延迟/限幅建模和 sim-to-real 校验。
- 相机已迁移但尚未进入视觉观测训练；继续推进会迅速转向设备工程而不是算法实验。

### 停止点

- 不继续盲目增加 hard replay 或长训。
- 不把当前 Gazebo 策略直接下发到实机。
- 后续如恢复本项目，优先做“相机实测”和“实机安全执行层”，而不是继续调 PPO。
- 下一阶段 RL 主线建议迁到官方 RL demo 更完备的平台，例如 Franka Panda + MuJoCo/Gymnasium/Isaac Lab。

## 2026-07-27 Reaching Toy Baseline

### 环境

- ROS 2：Foxy
- 容器镜像：`foxy-maxhub-a3:latest`
- 环境包：`carm_rl_env`
- 环境类：`carm_rl_env.CArmA3ReachingEnv`
- 训练库：Stable-Baselines3 `2.3.2`
- 算法：PPO
- 设备：CPU
- 并行环境：`--num-envs 4`
- 最大 episode 长度：`100`
- 成功阈值：`0.03 m`

### 训练命令

初始训练：

```bash
ros2 run carm_rl_env train_reaching \
  --algo ppo \
  --timesteps 50000 \
  --num-envs 4 \
  --n-steps 256 \
  --batch-size 128 \
  --device cpu \
  --eval-episodes 20
```

续训：

```bash
ros2 run carm_rl_env train_reaching \
  --algo ppo \
  --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_50000_steps.zip \
  --timesteps 200000 \
  --num-envs 4 \
  --n-steps 256 \
  --batch-size 128 \
  --device cpu \
  --eval-episodes 50
```

### 模型

```text
/workspace/rl_ws/artifacts/reaching/ppo_reaching_50000_steps.zip
/workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip
```

`artifacts/` 不入库，模型文件需要本机保留或另行备份。

### 训练结果

50k 初始训练评估：

```text
eval_episodes=20
eval_mean_distance=0.0889
eval_mean_reward=-12.5160
```

200k 续训评估：

```text
eval_episodes=50
eval_mean_distance=0.0242
eval_mean_reward=-2.8603
```

### 固定评估

评估命令：

```bash
ros2 run carm_rl_env evaluate_reaching \
  --model /workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip \
  --episodes 100 \
  --csv /workspace/rl_ws/artifacts/reaching/eval_200000_more.csv
```

结果：

```text
episodes=100
success_threshold=0.0300
success_rate=0.9900
mean_distance=0.0254
best_distance=0.0052
worst_distance=0.1622
mean_episode_length=15.43
mean_reward=-3.3578
```

### 结论

第一版 Gymnasium/SB3 reaching baseline 已成立。平均距离低于 3cm 阈值，100 episode 成功率达到 99%。当前环境仍是轻量 toy baseline，尚未包含动力学、碰撞、执行延迟或 Gazebo 仿真。

### 下一步

1. 增强评估记录：保存失败 episode 的 seed、target、final TCP 和是否 truncated。
2. 加难度评估：尝试 `--success-threshold 0.02` 的 2cm 标准。
3. 改进 reward：加入动作平滑惩罚、关节限位惩罚。
4. 接 Gazebo Classic：把当前 toy baseline 作为对照组，逐步迁移到仿真动力学环境。

## 2026-07-27 Gazebo Classic Reaching

### 环境

- ROS 2：Foxy
- 容器镜像：`foxy-maxhub-a3:latest`
- Gazebo：Gazebo Classic 11
- Gazebo 启动入口：`ros2 launch carm_gazebo spawn_a3_control.launch.py`
- 环境包：`carm_rl_gazebo`
- 环境类：`carm_rl_gazebo.CArmA3GazeboReachingEnv`
- 训练库：Stable-Baselines3 `2.3.2`
- 算法：PPO
- 并行环境：单 Gazebo world，暂不并行
- 常用控制参数：`action_scale=0.08`、`command_duration=0.10`、`command_timeout=0.12`
- 常用 reset：`--reset-world-on-reset`、`reset_noise=0.05`

### 主要模型与评估

当前 Gazebo 最佳基线之一：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_low_z_hard3035_progress_4096.zip
episodes=100
success_rate=0.6500
mean_distance=0.0376
worst_distance=0.1469
```

基于 seed 3035 低位困难目标 20k 续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_hard3035_progress_20000.zip
episodes=100
success_rate=0.6600
mean_distance=0.0372
worst_distance=0.1488
worst_failure=seed=3035 target=(0.1542,0.2146,0.1086) tcp=(0.1236,0.1035,0.2027)
```

距离回退惩罚续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_regress3_hard3035_10000.zip
episodes=100
success_rate=0.6500
mean_distance=0.0455
mean_best_distance=0.0339
worst_distance=0.1706
```

结论：单纯惩罚距离回退没有带来泛化收益，反而拉高了平均终点距离；但 `mean_best_distance` 显示大量 episode 曾经靠近过目标，保持/刹车是主要问题。

近目标动作惩罚续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3035_10000.zip
episodes=100
success_rate=0.6900
mean_distance=0.0392
mean_best_distance=0.0321
worst_distance=0.1462
worst_failure=seed=3036 target=(0.5025,0.0751,0.5261) tcp=(0.4499,0.1142,0.3954)
```

结论：近目标动作惩罚是当前有效方向，成功率提升到 69%，失败数从 34 降到 31。剩余失败目标已经分散到低 z、高 z、正 y、负 y，不宜继续只围绕 seed 3035 加权。

针对 seed 3036 高 z 目标 5k 短续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip
episodes=100
success_rate=0.7600
mean_distance=0.0425
mean_best_distance=0.0367
worst_distance=0.3425
worst_failure=seed=3060 target=(0.4676,-0.0989,0.1139) tcp=(0.1261,-0.1238,0.1247)
```

结论：单点 hard3036 短训显著提高最终成功率到 76%，但把最坏失败转移到低 z、较大 x 的 seed 3060/3061，并且 worst distance 明显变差。下一轮不应继续单点 hard target，而应混合多个困难目标 replay。

多困难目标 replay 10k 续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_multihard_10000.zip
episodes=100
success_rate=0.6900
mean_distance=0.0458
mean_best_distance=0.0342
worst_distance=0.2186
worst_failure=seed=3062 target=(0.2446,0.1443,0.1689) tcp=(0.0551,0.1412,0.0600)
```

结论：多困难目标 replay 把 hard3036 单点续训造成的极端 worst distance 从 0.3425 拉回到 0.2186，但成功率回落到 69%，不能作为新主线。值得注意的是，低 z 失败目标的 best distance 有改善，说明多目标采样有帮助，但 10k steps 和 0.45 replay ratio 可能过强，导致整体策略被重新拉偏。

更轻多困难目标 replay 5k 续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_multihard_light_5000.zip
episodes=100
success_rate=0.6900
mean_distance=0.0460
mean_best_distance=0.0333
worst_distance=0.3128
worst_failure=seed=3008 target=(0.3592,-0.0587,0.4807) tcp=(0.1814,-0.1237,0.2317)
```

结论：降低 replay ratio 和训练步数仍未保住 hard3036 的 76% 成功率，且 worst distance 仍偏大。当前不继续 hard replay 盲训。

### 诊断记录

seed 3035 低位目标：

```text
target=(0.1542,0.2146,0.1086)
```

30 步 trace 曾到 `final_distance=0.1015`，且尾段仍在接近。45 步 trace 反而退到 `final_distance=0.1811`，说明不是单纯 episode 长度不够，而是靠近后策略继续动作导致偏离。

不同 action scale 的 45/60 步诊断：

```text
action_scale=0.08, max_steps=45, final_distance=0.1811
action_scale=0.06, max_steps=45, final_distance=0.0808
action_scale=0.04, max_steps=45, final_distance=0.1599
action_scale=0.06, max_steps=60, final_distance=0.1112
```

结论：`0.06` 更稳但仍未达 3cm；继续延长到 60 步会回退。后续优先改 reward/采样，而不是只加步数。

近目标动态动作缩放诊断：

```text
light model, seed=3008, no damping: final_distance=0.3128, best_distance=0.0382
light model, seed=3008, radius=0.08, min=0.25: final_distance=0.0207, terminated=True
light model, seed=3008, radius=0.04, min=0.50: final_distance=0.0246, terminated=True
```

100 集评估：

```text
light model, radius=0.08, min=0.25:
success_rate=0.5600
mean_distance=0.0483
mean_best_distance=0.0362
worst_distance=0.2119

light model, radius=0.04, min=0.50:
success_rate=0.7400
mean_distance=0.0404
mean_best_distance=0.0320
worst_distance=0.2898

hard3036 model, radius=0.04, min=0.50:
success_rate=0.6800
mean_distance=0.0432
mean_best_distance=0.0334
worst_distance=0.1802
```

结论：动态动作缩放能有效救回“曾经靠近后跑开”的单点，且可以压低 worst distance；但全局会误伤一部分本来能成功的轨迹。`radius=0.08/min=0.25` 太重，`radius=0.04/min=0.50` 更可用但仍不适合作为默认主线。

连续保持成功判定：

```text
base model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip
success_hold_steps=3
success_rate=0.7000
mean_distance=0.0355
mean_best_distance=0.0269
worst_distance=0.1695

base model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip
success_hold_steps=5
success_rate=0.5900
mean_distance=0.0416
mean_best_distance=0.0284
worst_distance=0.3077
```

结论：`success_hold_steps=3` 是较好的稳定成功口径；`5` 过严，会明显拉低成功率并放大 worst distance。

连续保持 3 步短续训：

```text
model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip
train_from=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip
timesteps=5000
success_hold_steps=3
hard_target_ratio=0.0

eval with success_hold_steps=3:
success_rate=0.7700
mean_distance=0.0312
mean_best_distance=0.0250
worst_distance=0.1549

eval with success_hold_steps=1:
success_rate=0.7600
mean_distance=0.0406
mean_best_distance=0.0331
worst_distance=0.2125
```

结论：hold3 短续训是当前最佳主线。它在稳定成功口径下从 70% 提升到 77%，同时普通成功口径仍保持 76%，没有牺牲原有指标。

### 当前状态

已新增多困难目标 replay 参数 `--hard-target-positions`，用于把多个失败中心混合采样，格式为 `x,y,z;x,y,z`。

已新增近目标动态动作缩放参数 `--near-target-action-scale-radius` 和 `--near-target-action-scale-min`。该机制默认关闭，当前仅作为评估/诊断开关。

已新增连续保持成功参数 `--success-hold-steps`。默认值为 `1`，兼容旧实验；当前推荐训练/稳定评估口径为 `3`。

### 下一步

1. 暂定 `ppo_gazebo_reaching_action008_nearstop_hold3_5000.zip` 为当前主线模型。
2. 暂停继续 hard replay 盲训。
3. 下一步可以进入相机/视觉观测准备，或者围绕 hold3 主线做更保守的小步评估。
