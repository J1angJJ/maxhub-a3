# MAXHUB A3 Foxy 工作区

这个目录是 MAXHUB A3 的 ROS 2 Foxy 开发入口，当前聚焦强化学习方向。

## 目录结构

```text
workspace/foxy/
├── docker/
│   └── foxy-maxhub-a3/   # 项目镜像与 compose 配置
└── rl_ws/                # ROS2 colcon 工作区
    └── src/
        ├── carm_api/
        ├── carm_a3_description/
        ├── carm_gazebo/
        ├── carm_rl_gazebo/
        ├── carm_rl_env/
        └── carm_rl_bringup/
```

## 进入容器

```bash
cd /home/j1angjj/workspace/maxhub-a3/workspace/foxy/docker/foxy-maxhub-a3
docker compose run --rm foxy-maxhub-a3 bash
```

带相机：

```bash
docker compose -f compose.yaml -f compose.camera.yaml run --rm foxy-maxhub-a3 bash
```

带 NVIDIA GPU/DRI 图形设备：

```bash
docker compose -f compose.yaml -f compose.gpu.yaml run --rm foxy-maxhub-a3 bash
```

相机和 GPU 同时启用：

```bash
docker compose -f compose.yaml -f compose.camera.yaml -f compose.gpu.yaml run --rm foxy-maxhub-a3 bash
```

## 编译 rl_ws

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
```

## 最小 Gymnasium 环境

当前已提供 `carm_rl_env`，接口按 Gymnasium 编写：

```python
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(action)
```

运行随机动作 smoke test：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 run carm_rl_env random_rollout
```

快速启动一次 PPO 训练：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --timesteps 10000 --num-envs 1 --n-steps 256 --batch-size 64 --device auto
```

4 个并行环境的 PPO baseline：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --timesteps 50000 --num-envs 4 --n-steps 256 --batch-size 128 --device cpu --eval-episodes 20
```

需要真正多进程采样时加 `--vec-env subproc`：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --timesteps 50000 --num-envs 4 --vec-env subproc --n-steps 256 --batch-size 128 --device cpu --eval-episodes 20
```

从已有模型继续训练：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_50000_steps.zip --timesteps 200000 --num-envs 4 --n-steps 256 --batch-size 128 --device cpu --eval-episodes 50
```

2cm 精度版续训：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip --timesteps 100000 --num-envs 4 --n-steps 256 --batch-size 128 --device cpu --eval-episodes 50 --success-threshold 0.02 --distance-reward-scale 2.0 --action-penalty-scale 0.02 --smoothness-penalty-scale 0.01 --joint-limit-penalty-scale 0.05 --success-bonus 1.0
```

固定失败目标微调：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_100000_more_steps.zip --target-position 0.1593,0.2044,0.4493 --timesteps 20000 --num-envs 4 --n-steps 128 --batch-size 64 --device cpu --success-threshold 0.02 --distance-reward-scale 2.0 --action-penalty-scale 0.02 --smoothness-penalty-scale 0.01 --joint-limit-penalty-scale 0.05 --success-bonus 1.0
```

混合 hard target replay 训练：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --load-model /workspace/rl_ws/artifacts/reaching/ppo_reaching_100000_more_steps.zip --hard-target-position 0.1593,0.2044,0.4493 --hard-target-ratio 0.2 --hard-target-noise 0.03 --timesteps 100000 --num-envs 4 --n-steps 256 --batch-size 128 --device cpu --success-threshold 0.02 --distance-reward-scale 2.0 --action-penalty-scale 0.02 --smoothness-penalty-scale 0.01 --joint-limit-penalty-scale 0.05 --success-bonus 1.0
```

评估已训练模型：

```bash
ros2 run carm_rl_env evaluate_reaching --model /workspace/rl_ws/artifacts/reaching/ppo_reaching_200000_more_steps.zip --episodes 100 --csv /workspace/rl_ws/artifacts/reaching/eval_200000_more.csv
```

追踪单个失败 seed：

```bash
ros2 run carm_rl_env trace_reaching --model /workspace/rl_ws/artifacts/reaching/ppo_reaching_100000_more_steps.zip --seed 2080 --success-threshold 0.02 --csv /workspace/rl_ws/artifacts/reaching/trace_seed_2080.csv
```

使用 A2C：

```bash
ros2 run carm_rl_env train_reaching --algo a2c --timesteps 10000 --device auto
```

如果使用 `compose.gpu.yaml` 进入容器，可以显式指定 CUDA：

```bash
ros2 run carm_rl_env train_reaching --algo ppo --timesteps 10000 --num-envs 4 --n-steps 256 --batch-size 128 --device cuda
```

训练产物默认保存到 `/workspace/rl_ws/artifacts/reaching`，该目录不入库。`--num-envs` 会创建多个向量化环境；默认 `--vec-env dummy` 在同一进程顺序采样，`--vec-env subproc` 会开启多进程采样。PPO 每轮采样量约为 `n_steps * num_envs`。当前轻量 MLP 环境在 CPU 上通常更快；GPU 路径主要用于后续图像观测、更大网络或 Gazebo/Isaac 并行仿真。

实验记录维护在 [EXPERIMENTS.md](EXPERIMENTS.md)。

第一版 reaching 任务只做轻量基线，不接动力学仿真：

```text
action: 6 轴归一化关节增量
observation: 关节位置 + TCP 位置 + 目标位置 + TCP 到目标的误差
reward: -distance(tcp, target) - action_penalty
terminated: TCP 距离目标小于阈值
truncated: 达到最大步数
```

## 查看机器人模型

宿主机允许本地 Docker 容器访问 X11：

```bash
xhost +local:docker
```

进入容器后编译并打开 RViz2：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch carm_a3_description display.launch.py
```

只发布 `robot_description` 和 TF，不打开 RViz：

```bash
ros2 launch carm_a3_description robot_state_publisher.launch.py
```

## Gazebo Classic

项目镜像已包含 Gazebo Classic 11、`gazebo_ros_pkgs`、`gazebo_ros2_control` 和常用控制器包。检查环境：

```bash
gzserver --version
gzclient --version
ros2 pkg list | grep -E "gazebo|ros2_control|controller"
```

启动 headless 空世界并加载 A3 模型：

```bash
cd /workspace/rl_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch carm_gazebo spawn_a3.launch.py
```

启动带 ROS 2 control 的 Gazebo：

```bash
ros2 launch carm_gazebo spawn_a3_control.launch.py
```

发送一个 6 关节位置目标：

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, 0.3, -0.3, 0.1, 0.1, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"
```

当前 `carm_gazebo` 已能通过 `joint_trajectory_controller` 控制 `joint1` 到 `joint6`。夹爪关节暂未纳入控制器；reaching 第一阶段先只控制主臂。

## Gazebo Reaching

终端 1 启动 Gazebo 和控制器：

```bash
cd /workspace/rl_ws
source install/setup.bash
ros2 launch carm_gazebo spawn_a3_control.launch.py
```

终端 2 运行 Gazebo 随机动作 smoke test：

```bash
cd /workspace/rl_ws
source install/setup.bash
ros2 run carm_rl_gazebo random_gazebo_rollout --steps 10
```

终端 2 也可以跑一次很短的 Gazebo PPO smoke test：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching --timesteps 64 --n-steps 32 --batch-size 32 --command-timeout 0.05 --joint-target-tolerance 0.08 --eval-episodes 1 --device cpu
```

评估 Gazebo 模型：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_1024_steps.zip --episodes 10 --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval_1024.csv --device cpu
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

评估这轮 Gazebo 微调：

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
colcon build --symlink-install --packages-select carm_rl_env carm_rl_gazebo
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

当前最佳候选是 `ppo_gazebo_reaching_action008_nearstop_hard3035_10000.zip`。100 集评估结果为 `success_rate=0.6900`、`mean_distance=0.0392`、`mean_best_distance=0.0321`、`worst_distance=0.1462`。剩余最差失败为 seed `3036`，目标 `0.5025,0.0751,0.5261`。

针对 seed 3036 高 z 目标的 5k steps 短续训：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3035_10000.zip \
  --run-name action008_nearstop_hard3036_5000 \
  --timesteps 5000 \
  --max-steps 45 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.00008 \
  --clip-range 0.05 \
  --hard-target-position 0.5025,0.0751,0.5261 \
  --hard-target-ratio 0.35 \
  --hard-target-noise 0.04 \
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

seed 3036 续训后的 100 集评估：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip \
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
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_nearstop_hard3036_5000.csv \
  --device cpu
```

seed 3036 续训评估结果：`success_rate=0.7600`、`mean_distance=0.0425`、`mean_best_distance=0.0367`、`worst_distance=0.3425`。成功率提高明显，但最坏失败转移到 seed `3060` 的低 z / 大 x 目标。

多困难目标 replay 续训实验：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip \
  --run-name action008_nearstop_multihard_10000 \
  --timesteps 10000 \
  --max-steps 45 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.00008 \
  --clip-range 0.05 \
  --hard-target-positions '0.1542,0.2146,0.1086;0.5025,0.0751,0.5261;0.4676,-0.0989,0.1139;0.4279,-0.0001,0.1377' \
  --hard-target-ratio 0.45 \
  --hard-target-noise 0.04 \
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

多困难目标 replay 续训后的 100 集评估：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_multihard_10000.zip \
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
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_nearstop_multihard_10000.csv \
  --device cpu
```

多困难目标 replay 10k 评估结果：`success_rate=0.6900`、`mean_distance=0.0458`、`mean_best_distance=0.0342`、`worst_distance=0.2186`。它缓解了 hard3036 单点续训的极端 worst distance，但成功率回落，暂不作为新主线。

更轻的多困难目标 replay 续训实验：

```bash
ros2 run carm_rl_gazebo train_gazebo_reaching \
  --load-model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_hard3036_5000.zip \
  --run-name action008_nearstop_multihard_light_5000 \
  --timesteps 5000 \
  --max-steps 45 \
  --n-steps 64 \
  --batch-size 64 \
  --learning-rate 0.00005 \
  --clip-range 0.05 \
  --hard-target-positions '0.1542,0.2146,0.1086;0.5025,0.0751,0.5261;0.4676,-0.0989,0.1139;0.4279,-0.0001,0.1377;0.2446,0.1443,0.1689' \
  --hard-target-ratio 0.25 \
  --hard-target-noise 0.04 \
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

更轻多困难目标 replay 续训后的 100 集评估：

```bash
ros2 run carm_rl_gazebo evaluate_gazebo_reaching \
  --model /workspace/rl_ws/artifacts/gazebo_reaching/ppo_gazebo_reaching_action008_nearstop_multihard_light_5000.zip \
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
  --csv /workspace/rl_ws/artifacts/gazebo_reaching/eval100_action008_nearstop_multihard_light_5000.csv \
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

当前 `carm_rl_gazebo` 不负责自动起停 Gazebo，需要先启动 `carm_gazebo spawn_a3_control.launch.py`。使用 `--reset-world-on-reset` 时，环境会在每个 episode reset 前调用 `/reset_world`，随后通过 trajectory 回到带噪声的中位关节姿态。每步会根据 `/joint_states` 的关节目标误差提前结束等待。

## 仿真方向

当前先把真实机械臂 SDK 接口、URDF 描述和 Gazebo Classic 基础入口迁到 Foxy。下一步建议按下面顺序推进：

1. Gazebo Classic：优先验证 URDF、关节轴、碰撞体、TF 和基础控制接口，和 Foxy 生态最贴近。
2. MuJoCo：适合强化学习训练，需要从 URDF 整理 actuator、joint limit、collision 和 MJCF 资产。
3. Isaac Sim / Isaac Lab：适合更重的视觉与并行仿真，但依赖 NVIDIA 图形栈和更大的镜像，建议单独建实验容器。

Gazebo reaching 设计见 [GAZEBO_REACHING_PLAN.md](GAZEBO_REACHING_PLAN.md)。

## 已知建模事项

当前 URDF 已增加无质量虚拟根 `world -> base_link`，用于避免 KDL 把带惯性的 `base_link` 当作根节点。关节信息和初步 RL 动作空间记录在 `carm_a3_description/docs/joints.md`。

## 迁移边界

当前已迁移机械臂 SDK 的 ROS2 基础接口、bringup 参数和机器人描述包。ROS1 中的视觉、手眼、抓取任务和安全运动层暂不整体搬运；后续应按 RL 需要逐步迁移，并优先补安全执行层。
