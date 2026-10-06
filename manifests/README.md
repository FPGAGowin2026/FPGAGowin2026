# repo 工作区管理

`/home/aa/FPGA_GOWIN` 是主 Git 仓库和 repo 客户端顶层。主仓库独立管理文档、脚本、清单及参考输入；repo 只管理三个已有子仓库：

| 本地路径／脚本范围 | GitHub 仓库 | 分组 |
| --- | --- | --- |
| `FPGA_ws` | `FPGAGowin2026/fpga_rtl` | `fpga` |
| `ugv_ws` | `FPGAGowin2026/ugv_ws` | `ugv` |
| `llm_ws` | `FPGAGowin2026/llm_ws` | `llm` |

这里的 UGV 功能包仍由整个 `ugv_ws` 仓库管理，不自动采用 BoomBoomFly 的功能包拆分。
repo 不管理本地 Gowin EDA、许可证、构建产物、ROS overlay 或硬件运行状态。

## 初始化现有顶层

使用支持 `--use-local-gitdirs` 的官方 repo 工具，并准备 GitHub 仓库访问权限：

```bash
cd /home/aa/FPGAGowin2026
repo init -u https://github.com/FPGAGowin2026/FPGAGowin2026.git -b main \
  -m manifests/default.xml --use-local-gitdirs
repo sync
```

首次初始化使用该选项复用现有 `.git/`；repo 元数据放入顶层 `.repo/`，由主仓库忽略。
不要对其他存储模式的现有客户端直接切换此选项。同步前检查 `repo status`，不使用强制同步覆盖本地工作。

## 日常使用

```bash
git pull --ff-only  # 顶层主仓库
repo status        # 三个子仓库工作状态
repo sync          # 按远端清单同步子仓库
```

清单跟随各仓库 main。repo sync 不更新顶层主仓库的文档和脚本，不编译、不下载位流、不启动 ROS 节点或电机，也不提交和推送代码。
新获取项目可能处于 detached HEAD，开发前用 `repo start feature-name fpga_rtl` 创建分支；已有分支继续保留。
使用 `Scripts/push_git.sh FPGA_ws`、`ugv_ws`、`llm_ws` 或 `main` 发布对应仓库，脚本推送到当前同名分支。

原同步脚本保留为统一 `sync_workspaces.sh` 的入口，仓库地址及路径从同一 manifest 读取。
不要同时运行 shell 同步脚本和 repo sync；shell 同步沿当前分支 upstream 快进，repo 同步遵循清单 revision。

## 新机器与选择性获取

先克隆主仓库到所需顶层，再执行上面的初始化命令。只获取 FPGA 时添加 `repo init` 参数 `-g fpga`；UGV 为 `-g ugv`；语音工作区为 `-g llm`。
清单只负责获取源码，不自动准备 ROS、Gowin、模型或其他工具依赖。

## 保存版本组合

目标验证通过后执行：

```bash
repo manifest -r -o manifests/validated.xml
git rev-parse HEAD
```

锁定清单记录三个子仓库提交，主仓库 SHA 另行保存。锁定文件本身不是仿真、时序、上板或车辆测试证据。
