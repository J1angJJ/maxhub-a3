# Ubuntu Development Notes

本文档记录 MAXHUB A3 项目在 Ubuntu 20.04 虚拟机端的开发约定。Windows 端主要用于资料整理、Git 同步和快照归档；实际 SDK、ROS、Python 调试优先在 Ubuntu 虚拟机中完成。

## Local Paths

Windows workspace:

```text
R:\maxhub-a3
```

Ubuntu workspace:

```bash
cd /home/noetic/maxhub-a3
pwd
```

Expected output:

```text
/home/noetic/maxhub-a3
```

Ubuntu-side development area inside this repository:

```bash
cd /home/noetic/maxhub-a3/workspace/ubuntu
pwd
```

后续虚拟机环境快照、环境记录、只读连接脚本和 ROS / SDK 适配内容优先放在 `workspace/ubuntu/` 下，避免和 Windows 端资料混在一起。

## Conda Environment

Ubuntu 虚拟机中已新建 conda 环境。该环境主要用于纯 Python SDK 或辅助脚本；ROS Noetic C++ 工作流不依赖 conda。

```bash
cd /home/noetic/maxhub-a3
conda activate maxhub-a3
python --version
python -m pip show carm
```

建议在该环境中安装和测试 Python SDK 相关依赖。每次新增关键依赖后，记录当前环境：

```bash
cd /home/noetic/maxhub-a3
conda activate maxhub-a3
conda env export --no-builds > workspace/ubuntu/environment.yml
python --version > workspace/ubuntu/python_version.txt
python -m pip freeze > workspace/ubuntu/requirements.txt
```

如果某些依赖必须通过 apt 安装，建议同时记录：

```bash
cd /home/noetic/maxhub-a3
apt list --installed > workspace/ubuntu/apt_installed_snapshot.txt
```

## First Checks

在运行 ROS 节点之前，先确认 Ubuntu 虚拟机网络能访问机械臂：

```bash
cd /home/noetic/maxhub-a3
ping -c 4 192.168.31.60
curl -I http://192.168.31.60
```

如果 ping 不通，优先检查虚拟机网络是否为桥接模式，以及 Ubuntu IP 是否处于 `192.168.31.0/24` 网段。

## Official SDK Sources

厂家提供的 SDK Demo 和文档入口：

- SDK Demo: https://github.com/cvte-robotics/carm_demo
- Other CVTE robotics demos: https://github.com/cvte-robotics
- CArm user documentation: https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf
- Product page: https://www.cvte.com/product/flexiblemanipulator

本仓库已将 ROS 编译所需的厂家 C++ SDK 移植到 ROS 工作区内：

```text
workspace/ubuntu/carm_ws/vendor/arm_control_sdk
```

因此虚拟机端只需要 `git pull`，不需要再单独 clone 官方仓库。完整官方 Demo 仍只作为来源参考；本仓库只保留编译所需 SDK、本机适配、实验记录、配置和经过筛选的最小验证脚本。

ROS 编译或运行前声明 SDK 环境：

```bash
cd /home/noetic/maxhub-a3
source workspace/ubuntu/carm_ws/vendor/arm_control_sdk/setup.bash
echo "$arm_control_sdk_DIR"
```

## Recommended Bring-up Order

1. 确认 Ubuntu 网络可达机械臂。
2. 声明 vendored C++ SDK 环境。
3. 编译 `workspace/ubuntu/carm_ws` ROS Noetic 工作区。
4. 先运行只读 ROS 状态发布节点，只读取状态、关节角度和末端位姿。
5. 保存设备固件版本、SDK 版本、Python 版本和环境快照。
6. 只读连接稳定后，再进入低速、小幅、单关节运动测试。

ROS Noetic 详细步骤见 [README_ros_noetic.md](README_ros_noetic.md)。

## Current Local Scripts

从 Ubuntu 仓库根目录运行：

```bash
cd /home/noetic/maxhub-a3
python workspace/ubuntu/scripts/check_network.py
```

`check_network.py` 只检查 TCP / HTTP 连通性，不调用机械臂 SDK。  

## Safety Rule

任何运动脚本默认不得自动执行运动命令。低速运动测试应显式要求命令行参数或交互确认，并在执行前确认：

- 急停按钮可立即触及。
- 机械臂底座和夹爪固定可靠。
- 线缆不会被运动部件拉扯。
- 工作范围内无人和无障碍物。
- 当前命令为低速、小幅、可预期动作。
