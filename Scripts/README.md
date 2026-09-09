# 工作区脚本

在主项目根目录运行以下脚本。同步脚本首次克隆仓库，再次运行执行 `git pull --ff-only`；遇到已存在的非 Git 目录会停止并保留文件。

| 脚本 | 用途 |
| --- | --- |
| `sync_fpga_workspace.sh` | 将 `FPGAGowin2026/fpga_rtl` 同步到 `FPGA_ws/` |
| `sync_gowingame.sh` | 将 `FPGAGowin2026/gowingame` 同步到 `gowingame/` |
| `sync_ros_packages.sh` | 将 `FPGAGowin2026/ros_ws` 整体同步到 `ros_ws/`，保留旧脚本名 |
| `setup_ros_dependencies.bash` | 使用 `source` 激活本地依赖覆盖层 |
| `push_git.sh` | 自动暂存、提交指定仓库的改动，再推送到 `origin` 同名分支 |

## 推送 Git 提交

指定仓库和提交说明，自动执行 `git add -A`、`git commit`、`git push`：

```bash
./Scripts/push_git.sh ros_ws "更新 ROS 功能包"
./Scripts/push_git.sh main "更新项目脚本与文档"
./Scripts/push_git.sh FPGA_ws "更新 FPGA 逻辑"
./Scripts/push_git.sh gowingame "更新软核实验"
```

按需选择一个命令执行。脚本会提交所选仓库中全部未忽略的新增、修改和删除文件，包括此前已暂存的内容；执行前可用 `git status` 检查范围。独立子仓库需要分别执行，两个 ROS 通信包统一通过 `ros_ws` 提交。

`main` 参数代表主项目仓库，不限定分支名。没有新改动时跳过提交，继续推送当前分支。脚本不自动拉取或强制推送；如果推送失败，本地提交保留，处理远端分歧或连接问题后可重新运行。

推送脚本的离线检查：`python3 Scripts/test_push_git.py`。

## ROS 工作区初始化

```bash
./Scripts/sync_ros_packages.sh
./Scripts/install_ros_dependencies.sh
source /opt/ros/humble/setup.bash
source Scripts/setup_ros_dependencies.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

必须先同步工作区，再安装依赖。`fpga_gateway` 和 `robot_bringup` 已作为普通功能包纳入 `ros_ws/src/robot/`，不再单独克隆或更新。其他三个自研包也由同一仓库管理。

`ros_ws/src/external/`、`build/`、`install/` 和 `log/` 保留在本地，不上传。同步脚本不会自动迁移旧目录；旧目录含本地修改时，应先备份并完成仓库迁移。

## 离线检查

```bash
python3 Scripts/test_ros_scripts.py
```

检查使用临时目录和模拟 Git 命令，不访问远端或安装依赖。
