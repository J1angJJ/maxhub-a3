# Gazebo Reaching 设计草案

本文档记录 MAXHUB A3 Foxy 方向从 toy Gymnasium reaching baseline 迁移到 Gazebo Classic 仿真的设计。实验记录暂不在本文维护。

## 当前状态

- `foxy-maxhub-a3:latest` 已支持 ROS 2 Foxy、Gymnasium、Stable-Baselines3、PyTorch、NVIDIA GPU overlay。
- `carm_a3_description` 已提供 URDF、mesh、RViz launch。
- `carm_rl_env` 已提供 toy reaching 环境、PPO/A2C 训练、续训、评估和 trace 工具。
- 当前项目镜像尚未安装 Gazebo Classic、`gazebo_ros_pkgs`、`ros2_control` 或控制器包。

## 目标

第一阶段目标不是完整真实动力学，而是建立可训练闭环：

```text
Gymnasium Env -> ROS 2 action/topic/service -> Gazebo robot -> joint_states/TCP -> reward
```

最小验收标准：

1. Gazebo headless 可以启动。
2. 机器人模型可以 spawn 到 Gazebo。
3. 可以发布 6 轴关节动作。
4. 可以读回 joint state 并计算 TCP。
5. Reaching reward 和 toy env 的观测/动作定义尽量一致。
6. 能用 SB3 PPO 跑短训练 smoke test。

## 依赖设计

Gazebo 依赖不放进共享 `ubuntu-env:foxy-user`，而放进项目镜像或单独 overlay。理由：

- Gazebo/控制器是 MAXHUB A3 项目相关依赖，不是所有 Foxy 项目都需要。
- 后续 Isaac/MuJoCo 也可能各自膨胀，项目层更容易拆分。

建议先加到项目镜像：

```text
workspace/foxy/docker/foxy-maxhub-a3/
├── apt/
│   └── gazebo.packages
└── Dockerfile
```

候选 apt 包：

```text
gazebo11
ros-foxy-gazebo-ros-pkgs
ros-foxy-robot-state-publisher
ros-foxy-joint-state-publisher
ros-foxy-ros2-control
ros-foxy-ros2-controllers
ros-foxy-gazebo-ros2-control
ros-foxy-joint-state-broadcaster
ros-foxy-joint-trajectory-controller
```

是否所有包在当前 apt 源里可用，需要实际 `apt-cache policy`/`apt install --dry-run` 确认。

## 包结构

建议新增两个包：

```text
rl_ws/src/
├── carm_gazebo/
│   ├── launch/
│   ├── worlds/
│   └── config/
└── carm_rl_gazebo/
    └── carm_rl_gazebo/
```

职责：

- `carm_gazebo`：只负责 Gazebo 启动、spawn entity、控制器配置、world 文件。
- `carm_rl_gazebo`：负责 Gymnasium Gazebo 环境，不把训练逻辑塞进 launch 包。

## URDF/SDF 策略

第一阶段沿用 `carm_a3_description/urdf/carm_a3.urdf`，但需要补充仿真控制相关标签。

推荐做法：

1. 保留原始 `carm_a3.urdf` 作为描述基线。
2. 新增 `urdf/carm_a3_gazebo.urdf.xacro` 或 `xacro/carm_a3_gazebo.urdf.xacro`。
3. Gazebo 专用文件 include 基线描述，并追加：
   - transmission
   - gazebo_ros2_control plugin
   - 简化 collision 选项

如果 xacro include 旧 URDF 不方便，先复制一份 Gazebo 专用 URDF 也可以，但需要在文件头注明来源和同步风险。

## 控制策略

有两条路：

### A. 快速路径：直接设置关节状态

通过 Gazebo service 或自定义节点直接设置模型关节状态。

优点：

- 最快进入 RL 闭环。
- 避开 ros2_control 配置细节。

缺点：

- 动力学不真实。
- 和真实机械臂控制链差距较大。

### B. 推荐路径：ros2_control + JointTrajectoryController

使用 `gazebo_ros2_control`，给 6 个 revolute joints 配置 position 或 trajectory controller。

优点：

- 更接近真实控制接口。
- 后续可以平滑迁移到真实 SDK/安全执行层。

缺点：

- 首次配置成本高。
- 需要处理 controller lifecycle 和 joint limits。

建议第一版采用 B，但保留 A 作为排障兜底。

## Gymnasium Gazebo 环境

新增环境建议叫：

```python
CArmA3GazeboReachingEnv
```

动作空间保持和 toy env 一致：

```text
action: shape=(6,), Box(-1, 1)
```

第一版动作含义：

```text
target_joint_position = current_joint_position + action * action_scale
```

观测保持一致：

```text
joint_positions(6) + tcp_position(3) + target_position(3) + delta(3)
```

reward 先复用 toy env 参数：

```text
reward =
  - distance_reward_scale * distance
  - action_penalty_scale * ||action||
  - smoothness_penalty_scale * ||action - previous_action||
  - joint_limit_penalty_scale * joint_limit_penalty
  + success_bonus
```

## 通讯接口

Gazebo 环境需要 ROS 2 节点桥接：

- 发布动作：
  - `/joint_trajectory_controller/joint_trajectory`
  - 或 controller 对应 action。
- 订阅状态：
  - `/joint_states`
  - `/tf`
- 控制仿真：
  - `/pause_physics`
  - `/unpause_physics`
  - `/reset_simulation`

第一版可使用同步阻塞 step：

```text
send command -> wait fixed dt / state update -> compute obs/reward
```

后续再优化成严格 step simulation。

## 训练策略

阶段 1：Gazebo smoke test

```text
1 env, random actions, 10 episodes
```

阶段 2：短 PPO

```text
1 env, 5k-10k timesteps
```

阶段 3：并行

Gazebo 并行不能像 toy env 一样简单 `num_envs=4`。需要多个 Gazebo namespace/port/world，复杂度高。先不做并行 Gazebo，保留 toy env 并行作为训练速度基线。

## 下一步实施顺序

1. 给项目镜像加 Gazebo apt 包，先 dry-run，再构建。
2. 新增 `carm_gazebo` 包，提供 empty world + spawn robot launch。
3. 让 `ros2 launch carm_gazebo spawn.launch.py` 能启动 headless Gazebo 并 spawn 模型。
4. 接 ros2_control controller，并让 joint trajectory 命令能动模型。
5. 新增 `carm_rl_gazebo`，实现 random rollout。
6. 接 SB3 短训练。

## 风险

- Foxy/Gazebo Classic 包版本较旧，`gazebo_ros2_control` 可能需要额外兼容处理。
- 当前 URDF collision 使用 STL mesh，Gazebo 可能较慢或碰撞不稳定；必要时用简化 collision。
- 真实动力学参数来自 SolidWorks 导出，未校准，训练结果暂时只作为方法验证。
- Gazebo 单实例训练会比 toy env 慢很多，先重视 correctness，再考虑速度。
