`timescale 1ns/1ps

// OV5640 DVP 最小 QVGA/RGB565 上电初始化。
// 该表用于接口联调与基础图像采集；自动曝光、白平衡、镜头阴影和画质参数
// 仍应按实际镜头/帧率标定，不能把本表当成所有模组通用的量产配置。
module ov5640_sccb_init #(
    parameter integer SCCB_CLK_DIV = 32,
    parameter integer STRETCH_TIMEOUT_TICKS = 8192,
    parameter integer POWERDOWN_HOLD_CYCLES = 240000,
    parameter integer RESET_HOLD_CYCLES = 240000,
    parameter integer STARTUP_DELAY_CYCLES = 480000,
    parameter integer SOFT_RESET_DELAY_CYCLES = 120000
) (
    input wire clk,
    input wire reset_n,
    output wire camera_xclk,
    output reg camera_reset_n,
    output reg camera_pwdn,
    inout wire camera_scl,
    inout wire camera_sda,
    output reg configured,
    output reg identity_error,
    output reg bus_error,
    output reg [15:0] chip_id
);
    localparam [6:0] SCCB_ADDR = 7'h3C;
    localparam [5:0] INIT_LAST = 6'd62;
    localparam [3:0] ST_POWER_WAIT=0,ST_RESET_WAIT=1,ST_STARTUP_WAIT=2,
      ST_IDH_START=3,ST_IDH_WAIT=4,ST_IDL_START=5,ST_IDL_WAIT=6,
      ST_INIT_START=7,ST_INIT_WAIT=8,ST_SOFT_RESET_WAIT=9,ST_DONE=10;
    localparam integer POWERDOWN_HOLD=(POWERDOWN_HOLD_CYCLES<1)?1:POWERDOWN_HOLD_CYCLES;
    localparam integer RESET_HOLD=(RESET_HOLD_CYCLES<1)?1:RESET_HOLD_CYCLES;
    localparam integer STARTUP_DELAY=(STARTUP_DELAY_CYCLES<1)?1:STARTUP_DELAY_CYCLES;
    localparam integer SOFT_RESET_DELAY=(SOFT_RESET_DELAY_CYCLES<1)?1:SOFT_RESET_DELAY_CYCLES;

    reg [3:0] state;
    reg [5:0] init_index;
    reg [31:0] timer;
    reg txn_start,txn_read;
    reg [15:0] txn_reg;
    reg [7:0] txn_data;
    wire txn_busy,txn_done,txn_error;
    wire [7:0] txn_read_data;

    // 本例要求 clk 本身为合法的 OV5640 XCLK（工程约束为 24 MHz）。
    assign camera_xclk = clk;

    function [15:0] init_reg_addr;
        input [5:0] index;
        begin
            case(index)
              // 先把系统时钟源切到 PLL，再执行软件复位。
              0:init_reg_addr=16'h3103;  1:init_reg_addr=16'h3008;
              2:init_reg_addr=16'h3008;  3:init_reg_addr=16'h3103;
              4:init_reg_addr=16'h3017;  5:init_reg_addr=16'h3018;
              6:init_reg_addr=16'h3034;  7:init_reg_addr=16'h3035;
              8:init_reg_addr=16'h3036;  9:init_reg_addr=16'h3037;
              10:init_reg_addr=16'h3108; 11:init_reg_addr=16'h3630;
              12:init_reg_addr=16'h3631; 13:init_reg_addr=16'h3632;
              14:init_reg_addr=16'h3633; 15:init_reg_addr=16'h3621;
              16:init_reg_addr=16'h3704; 17:init_reg_addr=16'h3703;
              18:init_reg_addr=16'h3715; 19:init_reg_addr=16'h3717;
              20:init_reg_addr=16'h370B; 21:init_reg_addr=16'h3705;
              22:init_reg_addr=16'h3905; 23:init_reg_addr=16'h3906;
              24:init_reg_addr=16'h3901; 25:init_reg_addr=16'h3731;
              26:init_reg_addr=16'h3600; 27:init_reg_addr=16'h3601;
              28:init_reg_addr=16'h302D; 29:init_reg_addr=16'h3620;
              30:init_reg_addr=16'h371B; 31:init_reg_addr=16'h471C;
              32:init_reg_addr=16'h3800; 33:init_reg_addr=16'h3801;
              34:init_reg_addr=16'h3802; 35:init_reg_addr=16'h3803;
              36:init_reg_addr=16'h3804; 37:init_reg_addr=16'h3805;
              38:init_reg_addr=16'h3806; 39:init_reg_addr=16'h3807;
              40:init_reg_addr=16'h3808; 41:init_reg_addr=16'h3809;
              42:init_reg_addr=16'h380A; 43:init_reg_addr=16'h380B;
              44:init_reg_addr=16'h380C; 45:init_reg_addr=16'h380D;
              46:init_reg_addr=16'h380E; 47:init_reg_addr=16'h380F;
              48:init_reg_addr=16'h3814; 49:init_reg_addr=16'h3815;
              50:init_reg_addr=16'h4300; 51:init_reg_addr=16'h501F;
              52:init_reg_addr=16'h5000; 53:init_reg_addr=16'h5001;
              54:init_reg_addr=16'h300E; 55:init_reg_addr=16'h460B;
              56:init_reg_addr=16'h4713; 57:init_reg_addr=16'h4407;
              58:init_reg_addr=16'h460C; 59:init_reg_addr=16'h3824;
              60:init_reg_addr=16'h3008; 61:init_reg_addr=16'h503D;
              default:init_reg_addr=16'h4740;
            endcase
        end
    endfunction

    function [7:0] init_reg_data;
        input [5:0] index;
        begin
            case(index)
              0:init_reg_data=8'h11;  1:init_reg_data=8'h82;
              2:init_reg_data=8'h42;  3:init_reg_data=8'h03;
              4:init_reg_data=8'hFF;  5:init_reg_data=8'hFF;
              6:init_reg_data=8'h1A;  7:init_reg_data=8'h11;
              8:init_reg_data=8'h46;  9:init_reg_data=8'h13;
              10:init_reg_data=8'h01; 11:init_reg_data=8'h36;
              12:init_reg_data=8'h0E; 13:init_reg_data=8'hE2;
              14:init_reg_data=8'h12; 15:init_reg_data=8'hE0;
              16:init_reg_data=8'hA0; 17:init_reg_data=8'h5A;
              18:init_reg_data=8'h78; 19:init_reg_data=8'h01;
              20:init_reg_data=8'h60; 21:init_reg_data=8'h1A;
              22:init_reg_data=8'h02; 23:init_reg_data=8'h10;
              24:init_reg_data=8'h0A; 25:init_reg_data=8'h12;
              26:init_reg_data=8'h08; 27:init_reg_data=8'h33;
              28:init_reg_data=8'h60; 29:init_reg_data=8'h52;
              30:init_reg_data=8'h20; 31:init_reg_data=8'h50;
              32:init_reg_data=8'h00; 33:init_reg_data=8'h00;
              34:init_reg_data=8'h00; 35:init_reg_data=8'h04;
              36:init_reg_data=8'h0A; 37:init_reg_data=8'h3F;
              38:init_reg_data=8'h07; 39:init_reg_data=8'h9B;
              40:init_reg_data=8'h01; 41:init_reg_data=8'h40;
              42:init_reg_data=8'h00; 43:init_reg_data=8'hF0;
              44:init_reg_data=8'h07; 45:init_reg_data=8'h68;
              46:init_reg_data=8'h03; 47:init_reg_data=8'hD8;
              48:init_reg_data=8'h31; 49:init_reg_data=8'h31;
              50:init_reg_data=8'h6F; 51:init_reg_data=8'h01;
              52:init_reg_data=8'hA7; 53:init_reg_data=8'hA3;
              54:init_reg_data=8'h58; 55:init_reg_data=8'h35;
              56:init_reg_data=8'h03; 57:init_reg_data=8'h04;
              58:init_reg_data=8'h22; 59:init_reg_data=8'h02;
              60:init_reg_data=8'h02; 61:init_reg_data=8'h00;
              // PCLK 正沿采样，HREF/VSYNC 高有效，与采集器约定一致。
              default:init_reg_data=8'h23;
            endcase
        end
    endfunction

    i2c_register16_master #(
        .CLK_DIV(SCCB_CLK_DIV),.STRETCH_TIMEOUT_TICKS(STRETCH_TIMEOUT_TICKS),
        .IGNORE_WRITE_NINTH_BIT(1'b1)
    ) u_sccb (
        .clk(clk),.reset_n(reset_n),.start(txn_start),.device_addr(SCCB_ADDR),
        .reg_addr(txn_reg),.is_read(txn_read),.write_data(txn_data),
        .busy(txn_busy),.done(txn_done),.error(txn_error),.read_data(txn_read_data),
        .i2c_scl(camera_scl),.i2c_sda(camera_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=ST_POWER_WAIT;init_index<=0;timer<=0;txn_start<=0;txn_read<=0;
            txn_reg<=0;txn_data<=0;camera_reset_n<=0;camera_pwdn<=1;
            configured<=0;identity_error<=0;bus_error<=0;chip_id<=0;
        end else begin
            txn_start<=0;
            case(state)
              ST_POWER_WAIT: begin
                camera_pwdn<=1;camera_reset_n<=0;configured<=0;
                if(timer>=POWERDOWN_HOLD-1)begin timer<=0;camera_pwdn<=0;state<=ST_RESET_WAIT;end
                else timer<=timer+1'b1;
              end
              ST_RESET_WAIT: begin
                camera_pwdn<=0;camera_reset_n<=0;
                if(timer>=RESET_HOLD-1)begin timer<=0;camera_reset_n<=1;state<=ST_STARTUP_WAIT;end
                else timer<=timer+1'b1;
              end
              ST_STARTUP_WAIT: begin
                if(timer>=STARTUP_DELAY-1)begin timer<=0;state<=ST_IDH_START;end
                else timer<=timer+1'b1;
              end
              ST_IDH_START:if(!txn_busy)begin
                // bus_error 表示当前一次配置尝试的状态；重试开始时清除，
                // 外部若需要故障历史可在其上升沿自行锁存。
                bus_error<=0;
                txn_read<=1;txn_reg<=16'h300A;txn_start<=1;state<=ST_IDH_WAIT;
              end
              ST_IDH_WAIT:if(txn_done)begin
                if(txn_error)begin bus_error<=1;camera_pwdn<=1;camera_reset_n<=0;timer<=0;state<=ST_POWER_WAIT;end
                else begin chip_id[15:8]<=txn_read_data;state<=ST_IDL_START;end
              end
              ST_IDL_START:if(!txn_busy)begin
                txn_read<=1;txn_reg<=16'h300B;txn_start<=1;state<=ST_IDL_WAIT;
              end
              ST_IDL_WAIT:if(txn_done)begin
                if(txn_error)begin bus_error<=1;camera_pwdn<=1;camera_reset_n<=0;timer<=0;state<=ST_POWER_WAIT;end
                else if({chip_id[15:8],txn_read_data}!=16'h5640)begin
                    chip_id[7:0]<=txn_read_data;identity_error<=1;camera_pwdn<=1;
                    camera_reset_n<=0;timer<=0;state<=ST_POWER_WAIT;
                end else begin
                    chip_id[7:0]<=txn_read_data;identity_error<=0;init_index<=0;state<=ST_INIT_START;
                end
              end
              ST_INIT_START:if(!txn_busy)begin
                txn_read<=0;txn_reg<=init_reg_addr(init_index);txn_data<=init_reg_data(init_index);
                txn_start<=1;state<=ST_INIT_WAIT;
              end
              ST_INIT_WAIT:if(txn_done)begin
                if(txn_error)begin bus_error<=1;camera_pwdn<=1;camera_reset_n<=0;timer<=0;state<=ST_POWER_WAIT;end
                else if(init_index==1)begin init_index<=2;timer<=0;state<=ST_SOFT_RESET_WAIT;end
                else if(init_index==INIT_LAST)begin configured<=1;state<=ST_DONE;end
                else begin init_index<=init_index+1'b1;state<=ST_INIT_START;end
              end
              ST_SOFT_RESET_WAIT:begin
                if(timer>=SOFT_RESET_DELAY-1)begin timer<=0;state<=ST_INIT_START;end
                else timer<=timer+1'b1;
              end
              ST_DONE: configured<=1;
              default:state<=ST_POWER_WAIT;
            endcase
        end
    end
endmodule
