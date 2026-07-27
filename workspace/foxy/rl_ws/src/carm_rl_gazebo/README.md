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
  --eval-episodes 1 \
  --device cpu
```

## 当前边界

- 环境不负责自动启动或关闭 Gazebo，需要外部先启动 `spawn_a3_control.launch.py`。
- `reset()` 当前通过向中位关节姿态发布 trajectory 做软复位，还没有调用 Gazebo reset/pause/step service。
- 动作空间、观测结构和 reward 第一版尽量贴近 `carm_rl_env` 的 toy reaching 环境。
- 第一阶段只控制 `joint1` 到 `joint6`，夹爪暂不纳入训练。
- 当前训练入口只使用单实例 `DummyVecEnv`，Gazebo 多实例需要独立端口、namespace 和模型名隔离后再打开。
