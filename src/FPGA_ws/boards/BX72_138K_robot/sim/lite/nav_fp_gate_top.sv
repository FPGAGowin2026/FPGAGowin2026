`timescale 1ns/1ps

// 验证顶层包装真实Gowin导出的GoRV32 FP门级平台；没有CPU行为替代。
module nav_fp_gate_top (
    input  wire        clk,
    input  wire        rst,
    input  wire        gsri,
    output wire        cpu_ready_o,
    output wire        cpu_fault_o,
    output wire [7:0]  fault_code_o,
    output wire [31:0] heartbeat_o,
    output wire        io_valid_o,
    output wire        io_write_o,
    output wire [11:0] io_addr_o,
    output wire [31:0] io_wdata_o,
    output wire [3:0]  io_wstrb_o,
    input  wire        io_ready_i,
    input  wire [31:0] io_rdata_i
);
    // Gowin GSR低有效；C++测试台先保持0，再置1释放全局复位。
    GSR GSR(.GSRI(gsri));

    bx72_nav_platform dut (
        .clk(clk), .rst(rst),
        .cpu_ready_o(cpu_ready_o), .cpu_fault_o(cpu_fault_o),
        .fault_code_o(fault_code_o), .heartbeat_o(heartbeat_o),
        .io_valid_o(io_valid_o), .io_write_o(io_write_o),
        .io_addr_o(io_addr_o), .io_wdata_o(io_wdata_o),
        .io_wstrb_o(io_wstrb_o), .io_ready_i(io_ready_i),
        .io_rdata_i(io_rdata_i)
    );
endmodule
