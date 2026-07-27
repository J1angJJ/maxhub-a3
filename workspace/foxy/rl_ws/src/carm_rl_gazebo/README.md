# carm_rl_gazebo

`carm_rl_gazebo` 是连接 Gazebo Classic 控制器的 Gymnasium 风格 reaching 环境。

## 运行

终端 1 启动 Gazebo 和控制器：

```bash
cd /workspace/rl_ws
source install/setup.bash
ros2 launch carm_gazebo spawn_a3_control.launch.py
```

终端 2 运行随机动作 smoke test：

```bash
cd /workspace/rl_ws
source install/setup.bash
ros2 run carm_rl_gazebo random_gazebo_rollout --steps 10
```

运行一次很短的 PPO smoke test：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --timesteps 64 \
  --n-steps 32 \
  --batch-size 32 \
  --command-timeout 0.05 \
  --joint-target-tolerance 0.08 \
  --eval-episodes 1 \
  --device cpu
```

只检查训练循环速度时可跳过评估：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --timesteps 64 \
  --n-steps 32 \
  --batch-size 32 \
  --eval-episodes 0 \
  --device cpu
```

从 toy reaching 策略迁移到 Gazebo 微调：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip \
  --run-name toy200k_reset_noise_lr1e4_clip005_reward_4096 \
  --timesteps 4096 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.0001 \
  --clip-range 0.05 \
  --ent-coef 0.0 \
  --vf-coef 0.5 \
  --action-scale 0.05 \
  --command-duration 0.08 \
  --command-settle-time 0.02 \
  --command-timeout 0.10 \
  --joint-target-tolerance 0.06 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --eval-episodes 0 \
  --device cpu
```

评估已训练模型：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_toy200k_reset_noise_lr1e4_clip005_reward_4096.zip \
  --episodes 10 \
  --action-scale 0.05 \
  --command-duration 0.08 \
  --command-settle-time 0.02 \
  --command-timeout 0.10 \
  --joint-target-tolerance 0.06 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval_toy200k_reset_noise_lr1e4_clip005_reward_4096.csv \
  --device cpu
```

Progress reward 微调，用每步距离缩短量约束策略别走偏：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_toy200k_reset_noise_lr1e4_clip005_reward_4096.zip \
  --run-name progress_lr1e4_clip005_4096 \
  --timesteps 4096 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.0001 \
  --clip-range 0.05 \
  --action-scale 0.05 \
  --command-duration 0.08 \
  --command-settle-time 0.02 \
  --command-timeout 0.10 \
  --joint-target-tolerance 0.06 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --eval-episodes 0 \
  --device cpu
```

低 z hard target replay 微调：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_progress_lr1e4_clip005_4096.zip \
  --run-name low_z_hard3035_progress_4096 \
  --timesteps 4096 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.0001 \
  --clip-range 0.05 \
  --hard-target-position 0.1542,0.2146,0.1086 \
  --hard-target-ratio 0.35 \
  --hard-target-noise 0.03 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --eval-episodes 0 \
  --device cpu
```

当前最佳 Gazebo reaching 评估基线：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_low_z_hard3035_progress_4096.zip \
  --episodes 100 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_best_low_z_hard3035_action008_full_space.csv \
  --device cpu
```

基于当前最佳配置的 20k steps 续训实验：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_low_z_hard3035_progress_4096.zip \
  --run-name action008_hard3035_progress_20000 \
  --timesteps 20000 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.0001 \
  --clip-range 0.05 \
  --hard-target-position 0.1542,0.2146,0.1086 \
  --hard-target-ratio 0.35 \
  --hard-target-noise 0.03 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --eval-episodes 0 \
  --device cpu
```

20k steps 续训后的 100 集评估：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_hard3035_progress_20000.zip \
  --episodes 100 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_hard3035_progress_20000.csv \
  --device cpu
```

更新 `carm_rl_gazebo` 源码后，在容器内重新编译并刷新环境：

```bash
cd /workspace/rl_ws
colcon build --symlink-install --packages-select carm_rl_gazebo
source install/setup.bash
```

低位目标近目标刹车续训实验：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_hard3035_progress_20000.zip \
  --run-name action008_nearstop_hard3035_10000 \
  --timesteps 10000 \
  --max-steps 45 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.0001 \
  --clip-range 0.05 \
  --hard-target-position 0.1542,0.2146,0.1086 \
  --hard-target-ratio 0.45 \
  --hard-target-noise 0.03 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --distance-regression-penalty-scale 3.0 \
  --near-target-action-penalty-scale 0.08 \
  --near-target-action-penalty-radius 0.08 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --eval-episodes 0 \
  --device cpu
```

近目标刹车续训后的 100 集评估：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3035_10000.zip \
  --episodes 100 \
  --max-steps 45 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --distance-regression-penalty-scale 3.0 \
  --near-target-action-penalty-scale 0.08 \
  --near-target-action-penalty-radius 0.08 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_nearstop_hard3035_10000.csv \
  --device cpu
```

追踪单个 Gazebo 失败 seed：

```bash
ros2 run carm_rl_gazebo trace_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_hard3035_progress_20000.zip \
  --seed 3035 \
  --max-steps 45 \
  --action-scale 0.08 \
  --command-duration 0.10 \
  --command-settle-time 0.02 \
  --command-timeout 0.12 \
  --joint-target-tolerance 0.08 \
  --progress-reward-scale 0.5 \
  --smoothness-penalty-scale 0.01 \
  --joint-limit-penalty-scale 0.05 \
  --success-bonus 1.0 \
  --reset-noise 0.05 \
  --reset-world-on-reset \
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/trace_seed3035_action008_20000_45steps.csv \
  --device cpu
```

## 当前边界

- 环境不负责自动启动或关闭 Gazebo，需要外部先启动 `spawn_a3_control.launch.py`。
- 使用 `--reset-world-on-reset` 时，`reset()` 会先调用 `/reset_world`，再通过 trajectory 回到带噪声的中位关节姿态。
- 每步动作发布后会监听 `/joint_states`，关节误差低于 `joint_target_tolerance` 时提前继续，否则等到 `command_timeout`。
- 动作空间、观测结构和 reward shaping 尽量贴近 `carm_rl_env` 的 toy reaching 环境。
- 第一阶段只控制 `joint1` 到 `joint6`，夹爪暂不纳入训练。
- 当前训练入口只使用单实例 `DummyVecEnv`，Gazebo 多实例需要独立端口、namespace 和模型名隔离后再打开。
- 默认训练参数偏向 smoke/快速迭代：`command_timeout=0.05`、`joint_target_tolerance=0.08`。需要更保守的控制追踪时可以调大 timeout、调小 tolerance。
