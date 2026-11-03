# BX72 FPGA 工程

活动工程为 [BX72_138K_robot.gprj](boards/BX72_138K_robot/BX72_138K_robot.gprj)。`src/` 内包含 RTL、约束、GoRV32 固件及已生成的 ROM 初始化；`sim/lite/` 保留与固件相关的主机定向检查。

构建需要 Gowin V1.9.12.04、Python 3，以及 WSL 中的 `riscv64-unknown-elf-gcc` 工具链。固件构建脚本从 Windows 调用 WSL；以下示例在 Windows PowerShell 中运行：

```powershell
cd src/FPGA_ws
python Scripts/build_lite_firmware.py --output-dir boards/BX72_138K_robot/build/firmware
python Scripts/verify_lite_project.py
python Scripts/build_bx72.py --gowin-shell '<Gowin IDE>/bin/gw_sh.exe' --output-dir boards/BX72_138K_robot/build/candidate
```

修改固件后必须重新生成 ROM，再进行综合与布局布线。构建目录、工具软件及已烧录的位流未纳入仓库。
