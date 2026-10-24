> 2026-10-20 状态更新：BX72 已收敛为纯 RTL 单 UDP、视频独占 DDR3；全局地图与蓝牙移除，CSM 局部定位子图保留。本文其余旧版本描述和构建数字均为历史记录，本轮结果见 [实施记录](../FPGA_ws/boards/BX72_138K_robot/docs/udp_only_implementation_20261020.md)。

# BX72 DDR 建图与网页融合记录

日期：2026-10-18。主窗口负责规划、接口审查和文档；所有 Git 融合、代码实施及软件测试交给 `gpt-6-luna`，推理强度 `max`，独立上下文 `fork_turns="none"`。本轮不修改全局模型设置。

本文保留融合、回归和综合阶段的证据。后续用户已要求执行带 30 分钟上限的 PNR，超时后精简/重构并重试；最新执行情况见 [PNR 闭环验证记录](PNR闭环验证记录.md)，不从本文历史阶段的“未运行 PNR”推断当前进度。

2026-10-19 状态补充：当前代码为 OV5647 融合与精简提交 `3314eba`，网页迁入独立 website 仓库。本文保留历史输入的验证，不能作为当前源码的构建验收；当前状态见 [交接](handoff.md)。

## 目标与协作

蓝牙和 UDP 都运行同款雷达 SLAM、关键帧回环及地图重建；UDP 保留相机视频。源分支提供蓝牙实现，本轮融合后按用户追加要求扩展 UDP，共享 DDR 的新增验证与源分支历史记录分别维护。

用户随后明确允许增加子代理。本轮代码工作并行为三名 `gpt-6-luna` / `max`：

| 代理 | 独占职责 |
| --- | --- |
| `integrate_ddr_mapping` | RTL、工程文件、构建与静态校验实现；最后统一提交代码 |
| `udp_mapping_tests` | 共享 DDR 并发测试、双模式核心测试及仿真入口 |
| `udp_mapping_review` | 只读独立审查、构建脚本回归、最终双版本综合验证 |

主窗口继续只做规划、只读审查及文档。提交由一名实施代理串行执行，避免共用索引的竞争；既有网页改动与主窗口文档不进入代码提交。

实施要求：UDP 复用现有唯一 DDR3 控制器和物理引脚，为视频双缓冲与地图分配互不重叠的地址区间；仲裁正确配对写命令/数据并将读返回送回请求方，传播双方背压且避免长期饥饿。地图初始化、回环清图/重放、视频连续收发和复位恢复必须同时验证。地图上行、网页栅格及 ROS 发布仍需独立协议，本轮不将建图等同于完成这些接口。

## 输入与实施计划

| 项目 | 固定输入或约定 |
| --- | --- |
| 目标仓库 | `E:\project\FPGA_workspace\FPGAGowin2026\FPGA_ws` |
| 本地基线 | `main@f6613fd`，融合前与 `origin/main` 相同 |
| 最新分支 | [codex/bx72-ddr-mapping](https://github.com/FPGAGowin2026/fpga_ws/tree/codex/bx72-ddr-mapping)，本轮 fetch 得到 `911a158` |
| 提交关系 | 共同祖先 `096ee71`；本地 main 独有 4 个提交，建图分支独有 `911a158`。建图分支相对共同祖先修改 27 个文件，新增 3486 行、删除 38 行 |
| 本地在途工作 | 网页 HTML、展示 JS、测试和验收文档；保留已跟踪与未跟踪内容，不提交这些已有改动 |
| 融合方式 | 保留双方历史的本地 merge，没有文本冲突；本轮不推送远端 |

工作顺序为：备份在途网页内容 → 合并双方提交 → 扩展 UDP 建图及共享 DDR → 并发回归、独立审查和最终输入综合 → 收口文档。分支已分叉，因此使用普通 merge；建图提交与在途网页改动没有重叠文件。

实施代理已将 5 个已跟踪文件差异与全部 24 个未跟踪文件备份到 `FPGA_ws/build/integration_ddr_mapping_20261018/`，包含 `tracked-ui-changes.patch`、`worktree-copy/` 和 `manifest.sha256.txt`。备份设为只读，记录原 HEAD、index tree 与原始文件 SHA-256；该目录由 Git 忽略，不作为远端交付物。

非快进合并自动完成，没有文件冲突；纯融合提交为 `b19c1f69af4b6e7993f8f7920230113f09a1f6e2`，只包含建图提交的 27 个文件，原 UI 改动保持未暂存，未跟踪文件仍在原位。兼容修复和回归与此融合节点分开记录。

最终实现提交为 `dec08a5cf99f967efc4279ba0c3d003e529c102c`（父节点为上述融合提交），包含 19 个代码、工程、约束和测试文件。主窗口文档独立提交；原有 UI 的代码、文档及验收产物均不进入本轮提交。未推送远端。

独立审查已核对原 UI 的 29 个文件 SHA-256，29/29 与备份匹配。审查与综合在代码冻结后进行，测试与代码最后由实施代理统一明确暂存，避免带入文档和网页在途改动。

源分支的公共核心无条件实例化建图，但 UDP 工程缺少对应源和后端接线。本轮给核心增加 `ENABLE_SLAM_MAPPING` 参数，两版顶层均显式开启；默认关闭配置仍用于独立核心测试。UDP 工程补齐建图源、后端和生命周期接线，共享模式复用 `radar_slam_ddr3` 的 FIFO/CDC/后端，`udp_camera_map_arbiter` 将其与视频接入唯一 DDR3 IP。

地图后端为初始化、tile 读取和写回统一加 `MAP_BASE_ADDR_UI`；UDP 使用 `0x080000`，蓝牙为 0。仲裁只在联合命令/数据就绪窗口轮转，读等待冻结下一授权，返回只送读请求方。蓝牙 PLL/复位层级约束随共享模式调整，UDP 补地图 FIFO 的异步释放链例外；两版构建都选择 `radar_slam` 资源配置。地图传输侧通过独立 `map_loc_clk_i` 明确时钟契约，当前 UDP 顶层地图与视频流均接 33 MHz，摄像头与 DDR UI 仍是独立时钟域。

## 功能边界

- 蓝牙版新增 EKF 位姿投影、关键帧回环约束、map→odom 修正和 DDR3 栅格地图重建；保留原蓝牙控制及 v4 遥测。
- UDP 版新增同款建图/回环/地图重建，同时保留摄像头、DDR3 视频、UDP 控制与遥测；地图不参与独立 N10 避障，也没有路径点导航。视频字节地址 `[0,0x177000)`，地图 `[0x200000,0x2C0000)`，完整布局见 [设计（历史文档已整理）](../FPGA_ws/boards/BX72_138K_robot/docs/udp_only_implementation_20261020.md)。
- 地图为 1024×1024 格、64 mm/格、6 bit/格，unknown=32，容量 768 KiB；关键帧最多 32 帧，每帧 64 扇区。
- 顶层单格地图读取接口当前绑空闲；本轮没有地图下载、网页栅格可视化或 ROS 地图协议。网页雷达点、规划扇区和位姿轨迹各自保留原含义。
- 蓝牙 `led[1]` 改为 DDR 校准且初始地图清理完成，`led[0]` 为主 PLL 锁定，不能继续把 `led[1]` 当自动模式指示。

## 构建证据的归属

分支随附 [SLAM 构建记录](../FPGA_ws/boards/BX72_138K_robot/docs/history_evidence.md)：蓝牙 `bx72_bluetooth_slam_pipeline_v6`，综合 326.252 秒、PNR 1796.238 秒、总耗时 2123.14 秒，setup/hold 为 +0.004/+0.133 ns，Logic/Register/BSRAM/DSP 为 68367/34898/69/212，Latch 0。记录包含输入指纹和位流哈希。

这些是源分支已有构建记录，本轮未据此认定本地融合及后续修复已重新完成 PNR。原始网表、报告、位流不随 Git 提供；源码或构建配置改变后须重新构建并核对指纹。2026-10-17 的蓝牙 +0.065 ns 数据仅代表加入 SLAM 前的版本。

Micron 官方颗粒模型仿真在源分支记录为未运行。快速事务级 DDR 用户接口仿真不验证 PHY、校准及真实颗粒时序；硬件验收也未完成。

## 本轮验证

以下均为实施代理在本轮实际运行；日志路径相对于 `FPGA_ws`，测试输出由 Git 忽略，不随远端仓库交付。

网页部分已于 2026-10-19 提取到父工作区 [website](../website/README.md)；下表两条网页/服务测试命令是迁移前记录，现行启动与测试命令见 website 的 [README](../website/README.md)。

| 命令或检查 | 本轮结果 | 证据与范围 |
| --- | --- | --- |
| `python Scripts/verify_project.py --variant all` | PASS | UDP 58 个活动文件/127 个显式引脚；蓝牙 36 个活动文件/98 个显式引脚。无候选哈希可核对，不表示位流已就绪 |
| `python Scripts/run_sim.py --board BX72_138K_robot --suite mapping --timeout 300` | 5/5 PASS | `boards/BX72_138K_robot/build/sim/mapping_iverilog/summary.json`；前端、回环管理器、DDR 后端、仲裁器、4096 像素双帧共享 DDR |
| `python Scripts/run_sim.py --board BX72_138K_robot --suite single_comm` | 4/4 PASS | `boards/BX72_138K_robot/build/sim/single_comm_iverilog/summary.json`；命令仲裁、UDP CRC/CDC、无建图源码时的默认核心、开启建图后的核心展开/复位 |
| `node --test boards/BX72_138K_robot/tools/tests/dashboard_network.test.cjs boards/BX72_138K_robot/tools/tests/dashboard_presentation.test.cjs` | 24/24 PASS | 对保留的本地网页改版执行，退出码 0；使用软件夹具，不连接实车 |
| 在 `Scripts/dashboard` 执行 `node --test` | 17/17 PASS | 本机服务测试退出码 0；无硬件连接 |
| `python boards/BX72_138K_robot/sim/checks/test_bx72_build.py` | 11/11 PASS | 针对本轮构建脚本再运行；输入缓存、阶段与超时门禁 |
| `python boards/BX72_138K_robot/sim/checks/test_timing_checker.py` | PASS | 独立审查代理直接执行门禁断言脚本 |
| `python Scripts/run_sim.py --suite mapping_fullframe --timeout 300` | 1/1 PASS | `boards/BX72_138K_robot/build/sim/mapping_fullframe_iverilog/summary.json`；两帧各 384000 像素逐点一致，地图线程完成 |
| 原 `tb_udp_camera_ddr_model` | PASS | 原 64 像素视频入口补齐新依赖和端口空闲值后仍可运行 |
| 最终 UDP `build_bx72.py --variant udp --stage syn` | PASS | 全新快照，tool/synthesis 均为 true，日志无 WARN/ERROR；证据 `boards/BX72_138K_robot/build/udp_mapping_review_syn_r2_20261018/` |
| 最终蓝牙 `build_bx72.py --variant bluetooth --stage syn` | PASS | 全新快照，tool/synthesis 均为 true，日志无 WARN/ERROR；证据 `boards/BX72_138K_robot/build/bluetooth_mapping_review_syn_r2_20261018/` |
| 文档检查 | PASS | 主窗口检查新增/修改文档的本地 Markdown 目标存在；两仓库 `git diff --check` 通过 |

UDP 最终输入指纹为 `aff32f421d634e26abc1d5704bbf74e19b37ba1926c8a500f2df49a73aa9da60`，综合阶段 279.563 秒、入口总耗时 280.547 秒，74/74 输入哈希与最终源码匹配；蓝牙指纹为 `5dee7ae13eafd0256c64a6fdb9721c024d5f9174f7e431ff29b03387c0c4fc0c`，综合阶段 228.558 秒、入口总耗时 229.422 秒，52/52 输入哈希匹配。诊断过程中带未驱动/未连接警告的旧快照不计为最终通过；共享模式闲置输出及局部物理端口连接已修复，最终日志无这些警告。

综合阶段独立审查确认 UDP 只实例化一套 `DDR3_Memory_Interface_Top` 和顶层 DDR PLL，地图共享模式没有第二套 PHY，并确认相关 PLL/复位实例存在。但后续首轮 PNR 证明：实例存在不足以证明 SDC 对象可匹配，蓝牙 generate 名称被合成为带点的单实例，原斜杠层级写法有误；后续修正及实测见 [PNR 闭环记录](PNR闭环验证记录.md)。综合阶段 `timing_gate_passed=false` 表示未执行 PNR 门禁，不是已运行门禁失败；本文综合结果不能当作 PNR/位流结论。

仲裁回归覆盖双方持续请求、固定相位的独立命令/WDF 背压，以及固定/变化读延迟；共享 DDR 模型使用完整高地址和 131072×256 bit 内存，不截断高位来适配容量。4096 像素双帧使用 5/7 的 stream_ready，完整 384000 像素双帧使用 15/16 的 stream_ready，完成时刻分别为仿真时间 22.657245 ms 和 45.313875 ms；这些是仿真条件下的时间，不是实板帧率结论。

双模式展开证明可选模块配置和复位输出；mapping 和共享 DDR 回归采用事务级模型，未运行厂商 PHY 与真实 DRAM 颗粒仿真。本文记录的融合与综合阶段未运行 PNR；后续限时 PNR 见独立闭环记录。没有烧录或连接真实电机。

## 后续验收

1. 对最终源码分别生成蓝牙/UDP 构建，核对活动工程、CST/SDC、输入指纹、位流 SHA-256 与内部时序；尤其复核 SLAM 分支仅 4 ps 的 sensor setup 余量。
2. 准备匹配的厂商网表、Gowin 原语和 Micron 官方模型，运行真实 DDR3 仿真；固定车体后单独检查实板 DDR 校准、清图、持续读写与复位恢复。
3. 在已标定的传感器与底盘上记录建图掉点、FIFO 背压、关键帧、回环和地图重建结果；控制、租约停车与独立避障分别验收。
4. 地图需要网页或 ROS 接入时，先定义读取快照、坐标系、分辨率、修正版本及带宽契约，再实施传输和显示。
