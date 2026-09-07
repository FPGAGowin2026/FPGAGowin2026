`timescale 1ns/1ps

// 7 位设备地址、16 位寄存器地址、8 位数据的 I2C/SCCB 主机。
// 物理层复用 i2c_register8_master.v 中的 i2c_open_drain_byte_engine。
module i2c_register16_master #(
    parameter integer CLK_DIV = 32,
    parameter integer STRETCH_TIMEOUT_TICKS = 8192,
    // SCCB 的第九位是 Don't-Care，不是 I2C ACK。普通 I2C 保持为 0；
    // 仅在连接明确遵循 SCCB 的摄像头时置 1，SCL 超时仍然会报错。
    parameter IGNORE_WRITE_NINTH_BIT = 1'b0
) (
    input wire clk, input wire reset_n, input wire start,
    input wire [6:0] device_addr, input wire [15:0] reg_addr,
    input wire is_read, input wire [7:0] write_data,
    output reg busy, output reg done, output reg error,
    output reg [7:0] read_data,
    inout wire i2c_scl, inout wire i2c_sda
);
    localparam [4:0] S_IDLE=0,S_START=1,S_START_W=2,S_ADDRW=3,S_ADDRW_W=4,
      S_REGH=5,S_REGH_W=6,S_REGL=7,S_REGL_W=8,S_WDATA=9,S_WDATA_W=10,
      S_RESTART=11,S_RESTART_W=12,S_ADDRR=13,S_ADDRR_W=14,S_RDATA=15,
      S_RDATA_W=16,S_STOP=17,S_STOP_W=18,S_FINISH=19;
    localparam [2:0] CMD_START=0,CMD_STOP=1,CMD_WRITE=2,CMD_READ_NACK=4;
    reg [4:0] state;
    reg phy_start;
    reg [2:0] phy_cmd;
    reg [7:0] phy_tx;
    wire phy_busy,phy_done,phy_error,phy_nack;
    wire [7:0] phy_rx;
    reg [6:0] job_addr;
    reg [15:0] job_reg;
    reg job_read;
    reg [7:0] job_data;

    i2c_open_drain_byte_engine #(
        .CLK_DIV(CLK_DIV),.STRETCH_TIMEOUT_TICKS(STRETCH_TIMEOUT_TICKS)
    ) u_phy (
        .clk(clk),.reset_n(reset_n),.cmd_start(phy_start),.cmd(phy_cmd),
        .tx_data(phy_tx),.busy(phy_busy),.done(phy_done),.error(phy_error),.nack(phy_nack),
        .rx_data(phy_rx),.i2c_scl(i2c_scl),.i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=S_IDLE;phy_start<=0;phy_cmd<=0;phy_tx<=0;
            busy<=0;done<=0;error<=0;read_data<=0;
            job_addr<=0;job_reg<=0;job_read<=0;job_data<=0;
        end else begin
            phy_start<=0;done<=0;
            case(state)
                S_IDLE: begin
                    busy<=0;
                    if(start) begin
                        busy<=1;error<=0;job_addr<=device_addr;job_reg<=reg_addr;
                        job_read<=is_read;job_data<=write_data;state<=S_START;
                    end
                end
                S_START:if(!phy_busy)begin phy_cmd<=CMD_START;phy_start<=1;state<=S_START_W;end
                S_START_W:if(phy_done)begin if(phy_error)begin error<=1;state<=S_STOP;end else state<=S_ADDRW;end
                S_ADDRW:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<={job_addr,1'b0};phy_start<=1;state<=S_ADDRW_W;end
                S_ADDRW_W:if(phy_done)begin if(phy_error && !(IGNORE_WRITE_NINTH_BIT && phy_nack))begin error<=1;state<=S_STOP;end else state<=S_REGH;end
                S_REGH:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<=job_reg[15:8];phy_start<=1;state<=S_REGH_W;end
                S_REGH_W:if(phy_done)begin if(phy_error && !(IGNORE_WRITE_NINTH_BIT && phy_nack))begin error<=1;state<=S_STOP;end else state<=S_REGL;end
                S_REGL:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<=job_reg[7:0];phy_start<=1;state<=S_REGL_W;end
                S_REGL_W:if(phy_done)begin
                    if(phy_error && !(IGNORE_WRITE_NINTH_BIT && phy_nack))begin error<=1;state<=S_STOP;end
                    else state<=job_read?S_RESTART:S_WDATA;
                end
                S_WDATA:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<=job_data;phy_start<=1;state<=S_WDATA_W;end
                S_WDATA_W:if(phy_done)begin if(phy_error && !(IGNORE_WRITE_NINTH_BIT && phy_nack))error<=1;state<=S_STOP;end
                S_RESTART:if(!phy_busy)begin phy_cmd<=CMD_START;phy_start<=1;state<=S_RESTART_W;end
                S_RESTART_W:if(phy_done)begin if(phy_error)begin error<=1;state<=S_STOP;end else state<=S_ADDRR;end
                S_ADDRR:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<={job_addr,1'b1};phy_start<=1;state<=S_ADDRR_W;end
                S_ADDRR_W:if(phy_done)begin if(phy_error && !(IGNORE_WRITE_NINTH_BIT && phy_nack))begin error<=1;state<=S_STOP;end else state<=S_RDATA;end
                S_RDATA:if(!phy_busy)begin phy_cmd<=CMD_READ_NACK;phy_start<=1;state<=S_RDATA_W;end
                S_RDATA_W:if(phy_done)begin
                    if(phy_error)error<=1;else read_data<=phy_rx;state<=S_STOP;
                end
                S_STOP:if(!phy_busy)begin phy_cmd<=CMD_STOP;phy_start<=1;state<=S_STOP_W;end
                S_STOP_W:if(phy_done)begin if(phy_error)error<=1;state<=S_FINISH;end
                S_FINISH:begin busy<=0;done<=1;state<=S_IDLE;end
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
