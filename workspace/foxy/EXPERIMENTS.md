# MAXHUB A3 Foxy 实验记录

本文档记录 Foxy/RL 方向的实验进度、模型路径、关键命令和下一步计划。原则是轻量记录，避免训练结果和代码版本脱节。

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

### 当前进行中

已新增多困难目标 replay 参数 `--hard-target-positions`，用于把多个失败中心混合采样，格式为 `x,y,z;x,y,z`。

下一轮建议回到 `action008_nearstop_hard3036_5000` 主线，尝试更轻的多困难目标 replay，降低 replay ratio 和训练步数：

```text
load_model=/workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip
run_name=action008_nearstop_multihard_light_5000
hard_target_positions=0.1542,0.2146,0.1086;0.5025,0.0751,0.5261;0.4676,-0.0989,0.1139;0.4279,-0.0001,0.1377;0.2446,0.1443,0.1689
hard_target_ratio=0.25
hard_target_noise=0.04
timesteps=5000
```

### 下一步

1. 运行并评估 `action008_nearstop_multihard_light_5000`。
2. 如果成功率维持 75% 左右且 worst distance 回落，继续扩大多困难目标集合。
3. 如果成功率再次回落到 70% 左右，暂停 hard replay，优先考虑近目标低速动作裁剪、success 后保持步数或基于 best distance 的终止/奖励设计。
