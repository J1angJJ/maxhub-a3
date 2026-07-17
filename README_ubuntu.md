# Ubuntu Development Notes

本文档记录 MAXHUB A3 项目在 Ubuntu 20.04 虚拟机端的开发约定。Windows 端主要用于资料整理、Git 同步和快照归档；实际 SDK、ROS、Python 调试优先在 Ubuntu 虚拟机中完成。

## Local Paths

Windows workspace:

```text
R:\maxhub-a3
```

Ubuntu workspace:

```text
/home/noetic/maxhub-a3
```

Ubuntu-side development area inside this repository:

```text
workspace/ubuntu/
```

后续虚拟机环境快照、环境记录、只读连接脚本和 ROS / SDK 适配内容优先放在 `workspace/ubuntu/` 下，避免和 Windows 端资料混在一起。

## Conda Environment

Ubuntu 虚拟机中已新建 conda 环境：

```bash
conda activate maxhub-a3
```

建议在该环境中安装和测试 Python SDK 相关依赖。每次新增关键依赖后，记录当前环境：

```bash
conda env export --no-builds > workspace/ubuntu/conda_env_maxhub_a3.yml
python --version > workspace/ubuntu/python_version.txt
pip freeze > workspace/ubuntu/pip_freeze.txt
```

如果某些依赖必须通过 apt 安装，建议同时记录：

```bash
apt list --installed > workspace/ubuntu/apt_installed_snapshot.txt
```

## First Checks

在运行 SDK Demo 之前，先确认 Ubuntu 虚拟机网络能访问机械臂：

```bash
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

建议先将官方 Demo clone 到仓库外部目录，例如：

```bash
mkdir -p ~/vendor
cd ~/vendor
git clone https://github.com/cvte-robotics/carm_demo.git
```

本仓库只保留本机适配、实验记录、配置和经过筛选的最小验证脚本，不直接混入完整官方 Demo。

## Recommended Bring-up Order

1. 确认 Ubuntu 网络可达机械臂。
2. 激活 `maxhub-a3` conda 环境。
3. 阅读官方 Demo 的安装说明和最小连接示例。
4. 先写只读连接脚本，只读取状态、关节角度和末端位姿。
5. 保存设备固件版本、SDK 版本、Python 版本和 conda 环境快照。
6. 只读连接稳定后，再进入低速、小幅、单关节运动测试。

## Safety Rule

任何运动脚本默认不得自动执行运动命令。低速运动测试应显式要求命令行参数或交互确认，并在执行前确认：

- 急停按钮可立即触及。
- 机械臂底座和夹爪固定可靠。
- 线缆不会被运动部件拉扯。
- 工作范围内无人和无障碍物。
- 当前命令为低速、小幅、可预期动作。
