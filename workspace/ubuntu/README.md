# Ubuntu Workspace

本目录用于保存 Ubuntu 20.04 虚拟机端的开发记录、环境快照、只读连接测试和后续 ROS / SDK 适配代码。

## Quick Start

在 Ubuntu 虚拟机中执行：

```bash
cd /home/noetic/maxhub-a3
conda activate maxhub-a3
python workspace/ubuntu/scripts/check_network.py
python workspace/ubuntu/scripts/inspect_carm_sdk.py
```

## Files

- `environment.yml`: conda 环境快照。
- `requirements.txt`: pip 依赖快照。
- `config/robot.yaml`: 当前机械臂的非敏感配置。
- `scripts/check_network.py`: 只做网络连通性检查。
- `scripts/inspect_carm_sdk.py`: 只导入并检查 `carm` SDK 模块。

## Safety

本目录中的第一阶段脚本必须保持只读：可以检查网络、导入 SDK、读取模块信息、读取设备状态；不得默认下发任何运动、回零、使能或夹爪动作。
