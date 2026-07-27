# CArm A3 关节说明

本文档记录 `carm_a3.urdf` 中的主要关节信息，作为后续仿真、控制和强化学习动作空间设计的基础。

## 坐标根

当前 URDF 增加了一个无质量的虚拟根 link：

```text
world -> base_link
```

`world_to_base_link` 是固定关节，不改变模型位置。这样可以避免 KDL 把带惯性的 `base_link` 当作根节点时报错，同时保留 `base_link` 作为真实机身坐标。

## 机械臂关节

| 关节 | 类型 | 父 link | 子 link | 轴 | 下限 | 上限 | 说明 |
| --- | --- | --- | --- | --- | ---: | ---: | --- |
| `joint1` | revolute | `base_link` | `link1` | `0 0 1` | -2.9671 | 2.9671 | 底座旋转 |
| `joint2` | revolute | `link1` | `link2` | `0 0 1` | 0 | 3.1416 | 肩部关节 |
| `joint3` | revolute | `link2` | `link3` | `0 0 1` | -3.1416 | 0 | 肘部关节 |
| `joint4` | revolute | `link3` | `link4` | `0 0 1` | -2.6704 | 2.6704 | 腕部关节 |
| `joint5` | revolute | `link4` | `link5` | `0 0 1` | -1.5708 | 1.5708 | 腕部俯仰/摆动 |
| `joint6` | revolute | `link5` | `link6` | `0 0 1` | -2.8275 | 2.8275 | 末端旋转 |

所有关节轴都是在各自 joint frame 下表达的，不是全局坐标轴。后续需要用 RViz 或真实 SDK 对照确认正方向。

## 末端与夹爪

| 关节 | 类型 | 父 link | 子 link | 轴 | 下限 | 上限 | 说明 |
| --- | --- | --- | --- | --- | ---: | ---: | --- |
| `link6_to_flange` | fixed | `link6` | `flange` | - | - | - | 法兰固定连接 |
| `joint6_to_gripper_base` | fixed | `flange` | `gripper_base` | - | - | - | 夹爪底座固定连接 |
| `gripper_base_to_tcp` | fixed | `gripper_base` | `gripper_tcp` | - | - | - | 当前 TCP，位于夹爪中心线 |
| `joint7` | prismatic | `gripper_base` | `gripper_right` | `0 0 1` | 0 | 0.037 | 右夹爪开合 |
| `joint8` | prismatic | `gripper_base` | `gripper_left` | `0 0 1` | 0 | 0.037 | 左夹爪开合，mimic `joint7` |

`joint8` 使用 `<mimic joint="joint7" multiplier="1" offset="0" />`，因此初期控制可只暴露 `joint7` 作为夹爪开合动作。

## 强化学习动作空间建议

第一阶段 reaching 任务只使用六轴机械臂：

```text
action = [joint1, joint2, joint3, joint4, joint5, joint6] 的位置增量或速度指令
observation = 当前关节位置/速度 + TCP 位姿 + 目标点
reward = - TCP 到目标点的距离
```

后续再逐步加入：

- 动作平滑惩罚：避免抖动。
- 关节限位惩罚：避免策略贴边。
- 姿态奖励：让 TCP 姿态接近任务要求。
- 夹爪动作：从 reaching 进入 pick/place 或抓取任务时再加入 `joint7`。
