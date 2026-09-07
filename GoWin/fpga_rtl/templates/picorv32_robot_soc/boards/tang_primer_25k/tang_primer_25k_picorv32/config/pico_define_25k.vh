// Gowin_PicoRV32 1.4：GW5A-25B 首板验证配置。
// 本文件的宏名与本机官方 GowinPicoRV32.dll 生成模板保持一致。
`ifndef ROBOT_PICO_DEFINE_VH
`define ROBOT_PICO_DEFINE_VH

// CPU：RV32IMC，保留计数器、内部中断和定时器；关闭 JTAG Debug Module。
`define SUPPORT_RV32C 1
`define SUPPORT_RV32M 1
`define ENABLE_COUNTERS 1
`define ENABLE_IRQ 1
`define ENABLE_TIMER 1
`define TWO_STAGE_SHIFT 1
// 首版保留单周期 ALU/比较；时序修复放在官方模板的复位跨边沿路径。
`define TWO_CYCLE_COMPARE 0
`define TWO_CYCLE_ALU 0
`define BARREL_SHIFTER 0
`define CATCH_MISALIGN 1
`define CATCH_ILLINSN 1

// 16 KiB ITCM + 8 KiB DTCM，从 ITCM 启动并运行。
`define I_MEM_K_BYTE 16
`define D_MEM_K_BYTE 8
`define BUILD_LOAD
`define SW_BIN_PATH "firmware/picorv32_selftest.hex"

// 栈保护先关闭；首板工程不允许 CPU 直接控制执行器。
`define ENABLE_HSP 0
`define HSP_VALUE_DEF 32'h0000_0000

// 只启用串口与开放 Wishbone，避免在 25K 上浪费逻辑资源。
`define ENABLE_SIMPLE_UART
`define ENABLE_OPEN_WB_INTERFACE

// Gowin Arora V 25K 器件族选择。
`define DEVICE_ARORA_25K

`endif
