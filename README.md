# MAXHUB A3 Development Workspace

本仓库用于维护 MAXHUB A3 六轴智能柔性机械臂的开发资料、实验记录、脚本、配置和后续 ROS / Python / C++ 代码。

当前阶段以设备接入、网络确认、SDK 只读连接和低速安全运动测试为主。正式运动控制前，请先完成安全检查，并确认急停、电源、夹具、线缆和工作空间状态。

## 文档入口

- [README_getting_started.md](README_getting_started.md)：开箱清单、安装记录、电源急停、网络配置、网页验收和开发前置准备。

## 厂家资料

- SDK Demo：[https://github.com/cvte-robotics/carm_demo](https://github.com/cvte-robotics/carm_demo)
- 小龙虾以及其他 Demo：[https://github.com/cvte-robotics](https://github.com/cvte-robotics)
- CArm 用户文档中心：[https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf](https://cvte-bot.feishu.cn/wiki/GyGbwKeWMiqEfDk80QXc9zeCnjf)
- 官网：[https://www.cvte.com/product/flexiblemanipulator](https://www.cvte.com/product/flexiblemanipulator)

SDK 的使用 Demo 优先从官方 GitHub 获取。本仓库主要保存本地适配、实验记录、配置和必要的验证脚本。

## 当前设备摘要

| 项目 | 记录 |
|---|---|
| 设备型号 | MAXHUB A3 |
| 产品类型 | 六轴智能柔性机械臂 |
| 设备 SN | `A3YTL2607***-****` |
| 供电 | `24V DC, 20A Max` |
| 当前机械臂 IP | `192.168.31.60` |
| 出厂默认 IP | `10.42.0.101` |
| 网页上位机 | `http://192.168.31.60` |
| 推荐开发系统 | Ubuntu 20.04.6 LTS |
| 推荐 Python | Python 3.8 |
| 推荐 ROS | ROS 1 Noetic / ROS 2 Foxy |

## 建议仓库结构

```text
.
├── README.md
├── README_getting_started.md
├── config/
│   └── robot.yaml
├── docs/
│   ├── images/
│   ├── network/
│   └── troubleshooting/
├── scripts/
│   ├── connection_test/
│   ├── state_read/
│   ├── motion_test/
│   └── gripper_test/
└── src/
```

## 隐私与同步约定

- `private/` 用于保存本地私有资料，已加入 `.gitignore`，不进入 Git。
- 公开文档中不要写入完整设备 SN、账号、局域网敏感拓扑、凭据或厂商私发资料。
- Windows 与 Ubuntu 虚拟机之间通过 Git 同步；仓库配置要求 UTF-8 与 LF 换行，Windows 原生命令脚本除外。

## 开发前安全提醒

- 首次 SDK 连接建议只读设备状态，不下发运动命令。
- 首次运动建议低速、小幅、单关节测试。
- 回零或自动运动前，人员退出机械臂工作范围。
- 急停按钮必须放在操作者可立即触及的位置。
