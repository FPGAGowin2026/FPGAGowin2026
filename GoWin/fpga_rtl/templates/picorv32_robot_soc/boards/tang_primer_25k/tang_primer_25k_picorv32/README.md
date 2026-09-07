# GW5A-LV25MG121 PicoRV32 实板工程

这是机器人服务核模板的 25K 验证工程，器件为 `GW5A-LV25MG121NC1/I0`，CPU 为 RV32IMC，运行频率 50 MHz。完整架构、实测资源和 138K 迁移规则见模板根目录的 `README.md`。

## 构建

```powershell
.\Scripts\prepare_vendor_ip.ps1
& 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe' .\Scripts\run_gowin.tcl
```

生成文件为 `impl\pnr\tang_primer_25k_picorv32.fs`。本次已验证镜像 SHA-256：

```text
FE68877E7A1EB68D002E5BB49CADD3BCBD5A8C8EF92853A4426F9199AAF32E83
```

该哈希对应 2026-08-27 最终重建并通过 SRAM 烧录、UART 回归的当前位流；再次执行布局布线可能得到不同的实现文件，应以新文件重新计算哈希并做实板回归。

## 运行时调试

烧录 SRAM 后执行：

```powershell
.\Scripts\serial_debug.ps1 -Port COM5
```

脚本会验证启动 banner、Wishbone scratch 回读，以及 `P`、`I`、`S`、`?` 四条双向 UART 命令。它验证的是串口运行时调试，不等同于 GDB/JTAG 源码断点调试。
