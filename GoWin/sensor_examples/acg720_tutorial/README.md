# ACG720 教程完整提取、审计与复现工作区

本目录是对 `D:\BaiduNetdiskDownload\小梅哥ACG720国产高云FPGA教学开发板资料` 的只读提取与验证副本。D 盘原教程没有被修改。

## 先看这些文件

- `ACG720教程模块_25K复现与138K迁移报告.md`：完整结论、通过/失败边界和实板证据。
- `reports/chapter_matrix.md`：70 章逐章工程、模块、综合、P&R、时序和外设矩阵。
- `reports/all_unique_modules.csv`：全部 395 个唯一模块名及其文件位置。
- `reports/compatibility_pnr_results.md`：六个规范化兼容工程的位流和时序结果。
- `reports/pnr_timing_audit.csv`：原始与兼容工程的逐工程 setup/hold 违例端点统计。
- `reports/KNOWN_ISSUES.md`：教程原始资料中的命名、顶层、器件、IP、约束与工具兼容问题。

## 目录

- `raw/`：从教程压缩包提取的原始副本，按章保存。
- `reference_text/`：70 份 PDF 的文本提取结果。
- `normalized/`：不覆盖原始源码的兼容工程与缺失工程补全。
- `portable_25k_bist/`：Tang Primer 25K 仿真、P&R、SRAM 下载和 UART 实板自检。
- `portable_138k_bist/`：同一 BIST 向比赛最终器件 GW5AT-138B 的迁移构建。
- `reports/`：所有机器可读清单、日志、矩阵和结果。
- `scripts/`：提取、审计、综合、P&R、仿真和报告生成脚本。

## 重要边界

P&R/位流成功与时序收敛分别统计。25K 实板只验证了七个可移植教程模块及片上合成激励，没有连接 ACG720 的 DDR3、摄像头、显示、RGMII PHY、音频、ADC/DAC 或 FX2 外设。

