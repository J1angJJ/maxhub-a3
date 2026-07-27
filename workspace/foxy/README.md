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

训练产物默认保存到 `/workspace/rl_ws/artifacts/reaching`，该目录不入库。`--num-envs` 会创建多个向量化环境；PPO 每轮采样量约为 `n_steps * num_envs`。当前轻量 MLP 环境在 CPU 上通常更快；GPU 路径主要用于后续图像观测、更大网络或 Gazebo/Isaac 并行仿真。

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

当前 `carm_gazebo` 只负责模型加载和仿真入口，URDF 尚未加入 `ros2_control` 传动配置，因此还不能通过 Gazebo 控制器驱动关节。

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
