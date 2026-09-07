// ACG720 / GW5AT-138K 服务核建议配置；由最终板级工程复制并设置固件绝对路径。
`ifndef ROBOT_PICO_DEFINE_138K_VH
`define ROBOT_PICO_DEFINE_138K_VH

`define SUPPORT_RV32C 1
`define SUPPORT_RV32M 1
`define ENABLE_COUNTERS 1
`define ENABLE_IRQ 1
`define ENABLE_TIMER 1
`define TWO_STAGE_SHIFT 1
`define TWO_CYCLE_COMPARE 0
`define TWO_CYCLE_ALU 0
`define BARREL_SHIFTER 0
`define CATCH_MISALIGN 1
`define CATCH_ILLINSN 1

`define I_MEM_K_BYTE 32
`define D_MEM_K_BYTE 16
`define BUILD_LOAD
`define SW_BIN_PATH "firmware/robot_service.hex"

`define ENABLE_HSP 0
`define HSP_VALUE_DEF 32'h0000_0000
`define ENABLE_SIMPLE_UART
`define ENABLE_OPEN_WB_INTERFACE
`define DEVICE_ARORA_138K

`endif
