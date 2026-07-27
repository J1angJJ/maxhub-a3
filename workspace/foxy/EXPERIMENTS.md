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
