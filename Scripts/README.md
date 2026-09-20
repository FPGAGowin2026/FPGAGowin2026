# 工作区脚本

在主项目根目录使用。同步脚本首次克隆，已有独立仓库执行 `git pull --ff-only`；遇到已存在的非 Git 目录时停止，不覆盖文件。

| 脚本 | 用途 |
|---|---|
| `sync_fpga_workspace.sh` | 同步统一 FPGA/软核仓库 `FPGAGowin2026/fpga_rtl` 到 FPGA_ws |
| `sync_gowingame.sh` | 已停用，只提示使用统一入口并退出，不访问 Git 或网络 |
| `sync_ros_packages.sh` | 同步 ROS 工作区 |
| `sync_llm_workspace.sh` | 同步独立语音工作区 |
| `setup_ros_dependencies.bash` | 激活已有本地 ROS 依赖覆盖层 |
| `push_git.sh` | 暂存、提交、推送指定仓库 |

## 发布范围

```bash
./Scripts/push_git.sh main "更新项目文档"
./Scripts/push_git.sh FPGA_ws "更新 RTL 与软核程序"
./Scripts/push_git.sh ros_ws "更新 ROS 功能包"
./Scripts/push_git.sh llm_ws "更新语音实验"
```

每次命令执行指定仓库的 `git add -A`，包括此前已暂存及未暂存的新增、修改和删除，再提交并推送当前分支；使用前检查该仓库状态。`gowingame` 目标已拒绝，不会自动映射到另一个仓库。当前迁移没有执行任何提交或推送。

父仓库忽略 FPGA_ws、ros_ws、llm_ws 和已退役 gowingame 路径；各独立仓库分别发布。旧依赖安装脚本和测试脚本当前不存在，不将历史命令列为可运行入口。
