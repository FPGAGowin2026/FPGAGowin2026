# 工作区脚本

仓库地址和路径统一读取 [repo 清单](../manifests/default.xml)。现有项目根目录 `/home/aa/FPGAGowin2026` 直接作为客户端顶层；用法见 [manifest README](../manifests/README.md)。

| 脚本 | 用途 |
| --- | --- |
| `sync_workspaces.sh [范围…]` | 默认同步清单中的五个工作区，可指定部分范围 |
| `sync_fpga_workspace.sh` | 同步 FPGA_ws，兼容原入口 |
| `sync_ros_packages.sh` | 同步当前 ugv_ws，兼容原入口 |
| `sync_llm_workspace.sh` | 同步 llm_ws，兼容原入口 |
| `sync_gowingame.sh` | 停用提示，不访问 Git 或网络 |
| `repository.py` | 解析 manifest 的路径和远端 |
| `setup_ros_dependencies.bash` | 激活 ugv_ws 已有本地 ROS 依赖覆盖层 |
| `push_git.sh` | 提交并推送指定仓库 |
| `test_scripts.py` | 在临时目录与本地 bare 远程验证脚本 |

## 同步

```bash
./Scripts/sync_workspaces.sh
./Scripts/sync_workspaces.sh FPGA_ws ugv_ws
./Scripts/sync_workspaces.sh uav_ws swarm_ws
```

首次克隆，已有独立仓库沿当前分支 upstream 执行 `git pull --ff-only`；非 Git 目录保留并报错。
全部参数先验证，遇错停止，不自动切分支、reset、stash 或处理冲突。
repo 客户端可直接执行 `repo sync`，两个同步入口不要同时运行。

## 发布范围

```bash
./Scripts/push_git.sh main "更新项目文档"
./Scripts/push_git.sh FPGA_ws "更新 RTL 与软核程序"
./Scripts/push_git.sh ugv_ws "更新 ROS 功能包"
./Scripts/push_git.sh llm_ws "更新语音实验"
./Scripts/push_git.sh uav_ws "更新无人机工作区"
./Scripts/push_git.sh swarm_ws "更新集群工作区"
```

脚本暂存所选仓库全部未忽略改动，包括已有暂存内容，再提交并推送到 origin 同名分支。
执行前检查对应 Git 状态。主仓库排除各独立工作区、repo 元数据和旧 ros_ws/gowingame；这些路径已有暂存内容时停止并保留暂存区。
`ros_ws` 和 `gowingame` 范围已停用，不自动映射。没有新改动时仍可推送已有提交。
不自动 pull、强制推送或推送标签；推送失败保留本地提交。

## 离线验证

```bash
python3 Scripts/test_scripts.py
```

仅使用临时仓库与本地 bare 远程，验证范围、提交推送、克隆和快进同步。不连接实际项目远端，也不验证 RTL、ROS 或硬件功能。
