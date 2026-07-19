# 官方运动学接口资料梳理

本文记录 MAXHUB / CVTE CArm A3 官方 demo、SDK 与公开 Python 包中和正逆运动学相关的信息。结论用于指导本仓库 `carm_a3_motion` 的 FK/IK 服务设计。

## 资料来源

- 官方 demo 仓库：`https://github.com/cvte-robotics/carm_demo`
- 官方 GitHub 组织：`https://github.com/cvte-robotics`
- 官方文档中心：`https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf`
- 官方 Python 包页面：`https://pypi.org/project/carm/`
- 本地私有参考副本：`private/carm_demo`
- 本仓库迁移 SDK：`workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## 总体结论

官方资料确认 CArm 单臂 SDK 提供正逆运动学接口：

- `inverse_kine()`
- `forward_kine()`
- `inverse_kine_array()`
- `forward_kine_array()`

但官方 ROS1 demo 没有把 FK/IK 暴露成 ROS topic 或 service；ROS1 demo 主要暴露连接、ready、急停、关节/笛卡尔运动、跟随、速度、碰撞、夹爪、状态和回调话题。因此，本仓库把 FK/IK 做成 `carm_a3_motion` 内的 ROS service 是合理扩展，不是重复官方 ROS1 demo 的现成功能。

## C++ SDK 接口

本地 SDK 头文件 `arm_control_sdk/carm_cobot.h` 中的单臂接口如下：

```cpp
int inverse_kine(int tool_index,
                 const std::array<double, 7>& quat_pose,
                 const std::vector<double>& ref_joint,
                 std::vector<double>& jnt_value);

int forward_kine(int tool_index,
                 const std::vector<double>& jnt_value,
                 std::array<double, 7>& quat_pose);
```

参数约定：

- `tool_index`：工具号。
- `quat_pose`：目标或输出位姿，顺序为 `x,y,z,qx,qy,qz,qw`。
- `ref_joint`：IK 参考关节角，用来在多解中选择最优解。
- `jnt_value`：关节角，单位为 rad。
- 返回值：`1` 表示成功，`<1` 表示失败。

pybind 版 `carm_py.cpp` 只是薄封装 C++ SDK，返回 `(ret, result)`：

```python
ret, joints = arm.inverse_kine(tool_index, quat_pose, ref_joint)
ret, pose = arm.forward_kine(tool_index, jnt_value)
```

## 纯 Python WebSocket SDK

官方 `pip install carm` / 本地 `arm_control_sdk/python/carm_py/carm.py` 里也提供 FK/IK。它不是本仓库当前 C++ 主链路，但可作为协议参考：

```python
robot.inverse_kine(cart_pose, ref_joints, tool=0)
robot.forward_kine(joint_pos, tool=0)
```

底层请求使用同一个 WebSocket 命令：

```json
{
  "command": "getKinematics",
  "task_id": "inverse",
  "data": {
    "tool": 0,
    "point_cnt": 1,
    "point1": [x, y, z, qx, qy, qz, qw],
    "refer1": [j1, j2, j3, j4, j5, j6]
  }
}
```

正运动学对应：

```json
{
  "command": "getKinematics",
  "task_id": "forward",
  "data": {
    "tool": 0,
    "point_cnt": 1,
    "joint1": [j1, j2, j3, j4, j5, j6]
  }
}
```

这个路径可以作为后续备用实现：如果 C++ SDK 的运动学接口再次暴露稳定性问题，可以在 `carm_a3_motion` 中并行增加 WebSocket FK/IK service，而不覆盖 C++ service。

## 官方资料中的不一致

PyPI 页面中有一处状态属性说明把 `cart_pose` 写成 `x,y,z,qw,qx,qy,qz`，但以下证据都支持项目当前使用的 `x,y,z,qx,qy,qz,qw`：

- C++ 头文件 `carm_cobot.h` 注释写明 `x, y, z, qx, qy, qz, qw`。
- pybind 包装直接透传 C++ `std::array<double, 7>`。
- 纯 Python `inverse_kine()` 注释写明 `x, y, z, qx, qy, qz, qw`。
- PyPI IK 示例使用 `[0.5, 0, 0.3, 0.707, 0, 0.707, 0]`，与 `qx,qy,qz,qw` 约定一致。
- 本仓库实测 `ik-probe --include-mm` 中 `wxyz` 顺序失败，`xyzw` 顺序成功。

因此仓库内继续统一采用：

```text
pose = [x, y, z, qx, qy, qz, qw]
position unit = meter
joint unit = rad
```

## 和当前工程的关系

`carm_a3_motion` 已经对 C++ SDK FK/IK 做了 ROS service 封装：

- `/carm_a3/motion/solve_fk`
- `/carm_a3/motion/solve_ik`
- `/carm_a3/motion/solve_fk_array`
- `/carm_a3/motion/solve_ik_array`
- `/carm_a3/motion/get_cartesian_snapshot`
- `/carm_a3/motion/get_joint_snapshot`

推荐上层控制链路：

1. 先调用 `/get_joint_snapshot` 与 `/get_cartesian_snapshot`。
2. 用当前关节角作为 IK seed。
3. 小步修改笛卡尔位姿，优先从 `1-5 mm` 步长开始。
4. 调 `/solve_ik` 只求解，不运动。
5. 检查最大关节变化量，再调 `/move_joint` 执行。
6. 依赖 `/move_joint` 的读回验证确认实际到位。

当前实测结果：

- `forward_kine()` 可用。
- `inverse_kine()` 对当前位姿回代可用。
- `z + 0.01 m` 小步 IK 可用，并已通过 `move_joint` 执行。
- 从抬升后的姿态直接 `z - 0.01 m` 可能失败；扫描显示 `-0.005 m` 或更小回退步长可解。

## 后续建议

- 当前继续使用 C++ SDK FK/IK 作为主路径。
- 候选位姿扫描优先使用批量 ROS service，它们直接调用 C++ SDK 的 `forward_kine_array()` / `inverse_kine_array()`。
- 保留 WebSocket motion 节点作为运动备用路径。
- 暂不直接使用底层 low-level IK/FK，除非后续进入 1 ms 级控制、雅可比、动力学或阻抗/力控实验。
- 如果后续要做抓取或视觉伺服，优先实现“小步 IK + move_joint + 读回验证”的笛卡尔增量控制，而不是一次性大目标 IK。

## SDK 封装边界

当前 ROS 工程已经封装和任务相关的官方 C++ SDK 功能：

- 只读状态、配置、规划状态、外力、夹爪状态、工具坐标。
- 安全门控设置：速度、控制模式、碰撞配置、工具号。
- 安全门控运动：关节点到点、位姿点到点、关节直线、位姿直线、flow pose、夹爪。
- 单点和批量 FK/IK。

暂不封装连续 track、轨迹执行和示教复现。原因不是 SDK 不支持，而是这些功能需要独立的速率、超时、停止和轨迹合法性策略；当前阶段先保持基础视觉-IK-关节运动链路稳定。
