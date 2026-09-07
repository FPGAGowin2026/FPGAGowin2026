`timescale 1ns/1ps

// INA219 例程：配置 32 V/320 mV、12 位连续转换，回读配置后周期读取分流与总线电压。
module ina219_i2c #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input wire clk,input wire reset_n,inout wire i2c_scl,inout wire i2c_sda,
    output reg sensor_ok,output reg initialized,output reg bus_error,
    output reg signed [15:0] shunt_voltage_raw,
    output reg [15:0] bus_voltage_raw,output reg [15:0] bus_voltage_mv,
    output reg sample_valid
);
    localparam [6:0] I2C_ADDR=7'h40;
    localparam [7:0] REG_CONFIG=8'h00,REG_SHUNT=8'h01,REG_BUS=8'h02;
    localparam [15:0] CONFIG_VALUE=16'h399F;
    localparam integer STARTUP_DELAY=(STARTUP_DELAY_CYCLES<1)?1:STARTUP_DELAY_CYCLES;
    localparam integer SAMPLE_INTERVAL=(SAMPLE_INTERVAL_CYCLES<1)?1:SAMPLE_INTERVAL_CYCLES;
    localparam [3:0] ST_DELAY=0,ST_CFG_START=1,ST_CFG_WAIT=2,ST_VERIFY_START=3,
      ST_VERIFY_WAIT=4,ST_SAMPLE_DELAY=5,ST_SHUNT_START=6,ST_SHUNT_WAIT=7,
      ST_SHUNT_STORE=8,ST_BUS_START=9,ST_BUS_WAIT=10,ST_PUBLISH=11;
    reg [3:0] state;
    reg [31:0] delay_count;
    reg txn_start,txn_read;
    reg [7:0] txn_reg_addr;
    reg [15:0] txn_write_data;
    reg [1:0] txn_write_length,txn_read_length;
    wire txn_busy,txn_done,txn_error,txn_rx_valid;
    wire [1:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes[0:1];
    integer i;

    i2c_register_transaction8_ina219 #(.CLK_DIV(I2C_CLK_DIV)) u_i2c (
      .clk(clk),.reset_n(reset_n),.start(txn_start),.device_addr(I2C_ADDR),
      .reg_addr(txn_reg_addr),.is_read(txn_read),.write_data(txn_write_data),
      .write_length(txn_write_length),.read_length(txn_read_length),
      .busy(txn_busy),.done(txn_done),.error(txn_error),
      .rx_valid(txn_rx_valid),.rx_index(txn_rx_index),.rx_data(txn_rx_data),
      .i2c_scl(i2c_scl),.i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
      if(!reset_n) begin
        state<=ST_DELAY;delay_count<=0;txn_start<=0;txn_read<=0;txn_reg_addr<=0;
        txn_write_data<=0;txn_write_length<=0;txn_read_length<=0;sensor_ok<=0;
        initialized<=0;bus_error<=0;shunt_voltage_raw<=0;bus_voltage_raw<=0;
        bus_voltage_mv<=0;sample_valid<=0;
        for(i=0;i<2;i=i+1) rx_bytes[i]<=0;
      end else begin
        txn_start<=0;sample_valid<=0;
        if(txn_rx_valid && txn_rx_index<2) rx_bytes[txn_rx_index]<=txn_rx_data;
        case(state)
          ST_DELAY: begin
            if(delay_count>=STARTUP_DELAY-1) begin delay_count<=0;state<=ST_CFG_START;end
            else delay_count<=delay_count+1'b1;
          end
          ST_CFG_START: if(!txn_busy) begin
            txn_read<=0;txn_reg_addr<=REG_CONFIG;txn_write_data<=CONFIG_VALUE;
            txn_write_length<=2;txn_read_length<=0;txn_start<=1;state<=ST_CFG_WAIT;
          end
          ST_CFG_WAIT: if(txn_done) begin
            if(txn_error) begin
              bus_error<=1;sensor_ok<=0;initialized<=0;delay_count<=0;state<=ST_DELAY;
            end else state<=ST_VERIFY_START;
          end
          ST_VERIFY_START: if(!txn_busy) begin
            txn_read<=1;txn_reg_addr<=REG_CONFIG;txn_write_length<=0;
            txn_read_length<=2;txn_start<=1;state<=ST_VERIFY_WAIT;
          end
          ST_VERIFY_WAIT: if(txn_done) begin
            if(!txn_error && {rx_bytes[0],rx_bytes[1]}==CONFIG_VALUE) begin
              sensor_ok<=1;initialized<=1;delay_count<=0;state<=ST_SAMPLE_DELAY;
            end else begin
              if(txn_error) bus_error<=1;
              sensor_ok<=0;initialized<=0;delay_count<=0;state<=ST_DELAY;
            end
          end
          ST_SAMPLE_DELAY: begin
            if(delay_count>=SAMPLE_INTERVAL-1) begin delay_count<=0;state<=ST_SHUNT_START;end
            else delay_count<=delay_count+1'b1;
          end
          ST_SHUNT_START: if(!txn_busy) begin
            txn_read<=1;txn_reg_addr<=REG_SHUNT;txn_read_length<=2;
            txn_start<=1;state<=ST_SHUNT_WAIT;
          end
          ST_SHUNT_WAIT: if(txn_done) begin
            if(txn_error) begin bus_error<=1;state<=ST_SAMPLE_DELAY;end
            else state<=ST_SHUNT_STORE;
          end
          ST_SHUNT_STORE: begin
            shunt_voltage_raw<={rx_bytes[0],rx_bytes[1]};state<=ST_BUS_START;
          end
          ST_BUS_START: if(!txn_busy) begin
            txn_read<=1;txn_reg_addr<=REG_BUS;txn_read_length<=2;
            txn_start<=1;state<=ST_BUS_WAIT;
          end
          ST_BUS_WAIT: if(txn_done) begin
            if(txn_error) begin bus_error<=1;state<=ST_SAMPLE_DELAY;end
            else state<=ST_PUBLISH;
          end
          ST_PUBLISH: begin
            bus_voltage_raw<={rx_bytes[0],rx_bytes[1]};
            bus_voltage_mv<={1'b0,rx_bytes[0],rx_bytes[1][7:3],2'b00};
            sample_valid<=1;state<=ST_SAMPLE_DELAY;
          end
          default: state<=ST_DELAY;
        endcase
      end
    end
endmodule

module i2c_register_transaction8_ina219 #(
    parameter integer CLK_DIV=32
) (
    input wire clk,input wire reset_n,input wire start,input wire [6:0] device_addr,
    input wire [7:0] reg_addr,input wire is_read,input wire [15:0] write_data,
    input wire [1:0] write_length,input wire [1:0] read_length,
    output reg busy,output reg done,output reg error,output reg rx_valid,
    output reg [1:0] rx_index,output reg [7:0] rx_data,
    inout wire i2c_scl,inout wire i2c_sda
);
    localparam [4:0] S_IDLE=0,S_START=1,S_START_W=2,S_ADDRW=3,S_ADDRW_W=4,
      S_REG=5,S_REG_W=6,S_WDATA=7,S_WDATA_W=8,S_RSTART=9,S_RSTART_W=10,
      S_ADDRR=11,S_ADDRR_W=12,S_RDATA=13,S_RDATA_W=14,S_STOP=15,S_STOP_W=16,S_FINISH=17;
    localparam [2:0] CMD_START=0,CMD_STOP=1,CMD_WRITE=2,CMD_READ_ACK=3,CMD_READ_NACK=4;
    reg [4:0] state;
    reg phy_start;reg [2:0] phy_cmd;reg [7:0] phy_tx;
    wire phy_busy,phy_done,phy_ack_error;wire [7:0] phy_rx;
    reg [6:0] job_addr;reg [7:0] job_reg;reg [15:0] job_wdata;
    reg job_read;reg [1:0] job_wlen,job_rlen,byte_index;
    i2c_byte_engine_ina219 #(.CLK_DIV(CLK_DIV)) u_phy(
      .clk(clk),.reset_n(reset_n),.cmd_start(phy_start),.cmd(phy_cmd),.tx_data(phy_tx),
      .busy(phy_busy),.done(phy_done),.ack_error(phy_ack_error),.rx_data(phy_rx),
      .i2c_scl(i2c_scl),.i2c_sda(i2c_sda));
    always @(posedge clk or negedge reset_n) begin
      if(!reset_n) begin
        state<=S_IDLE;phy_start<=0;phy_cmd<=0;phy_tx<=0;busy<=0;done<=0;error<=0;
        rx_valid<=0;rx_index<=0;rx_data<=0;job_addr<=0;job_reg<=0;job_wdata<=0;
        job_read<=0;job_wlen<=0;job_rlen<=0;byte_index<=0;
      end else begin
        phy_start<=0;done<=0;rx_valid<=0;
        case(state)
          S_IDLE: begin busy<=0;if(start) begin busy<=1;error<=0;job_addr<=device_addr;
            job_reg<=reg_addr;job_wdata<=write_data;job_read<=is_read;job_wlen<=write_length;
            job_rlen<=read_length;byte_index<=0;state<=S_START;end end
          S_START:if(!phy_busy)begin phy_cmd<=CMD_START;phy_start<=1;state<=S_START_W;end
          S_START_W:if(phy_done)begin if(phy_ack_error)begin error<=1;state<=S_FINISH;end else state<=S_ADDRW;end
          S_ADDRW:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<={job_addr,1'b0};phy_start<=1;state<=S_ADDRW_W;end
          S_ADDRW_W:if(phy_done)begin if(phy_ack_error)error<=1;state<=S_REG;end
          S_REG:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<=job_reg;phy_start<=1;state<=S_REG_W;end
          S_REG_W:if(phy_done)begin if(phy_ack_error)error<=1;byte_index<=0;state<=job_read?S_RSTART:S_WDATA;end
          S_WDATA:if(!phy_busy)begin phy_cmd<=CMD_WRITE;
            phy_tx<=(job_wlen==2&&byte_index==0)?job_wdata[15:8]:job_wdata[7:0];
            phy_start<=1;state<=S_WDATA_W;end
          S_WDATA_W:if(phy_done)begin if(phy_ack_error)error<=1;
            if(byte_index+1'b1>=job_wlen)state<=S_STOP;
            else begin byte_index<=byte_index+1'b1;state<=S_WDATA;end end
          S_RSTART:if(!phy_busy)begin phy_cmd<=CMD_START;phy_start<=1;state<=S_RSTART_W;end
          S_RSTART_W:if(phy_done)begin if(phy_ack_error)begin error<=1;state<=S_FINISH;end else state<=S_ADDRR;end
          S_ADDRR:if(!phy_busy)begin phy_cmd<=CMD_WRITE;phy_tx<={job_addr,1'b1};phy_start<=1;state<=S_ADDRR_W;end
          S_ADDRR_W:if(phy_done)begin if(phy_ack_error)error<=1;byte_index<=0;state<=S_RDATA;end
          S_RDATA:if(!phy_busy)begin phy_cmd<=(byte_index+1'b1>=job_rlen)?CMD_READ_NACK:CMD_READ_ACK;
            phy_start<=1;state<=S_RDATA_W;end
          S_RDATA_W:if(phy_done)begin
            if(phy_ack_error)begin error<=1;state<=S_STOP;end
            else begin rx_valid<=1;rx_index<=byte_index;rx_data<=phy_rx;
              if(byte_index+1'b1>=job_rlen)state<=S_STOP;
              else begin byte_index<=byte_index+1'b1;state<=S_RDATA;end end
          end
          S_STOP:if(!phy_busy)begin phy_cmd<=CMD_STOP;phy_start<=1;state<=S_STOP_W;end
          S_STOP_W:if(phy_done)begin if(phy_ack_error)error<=1;state<=S_FINISH;end
          S_FINISH:begin busy<=0;done<=1;state<=S_IDLE;end
          default:state<=S_IDLE;
        endcase
      end
    end
endmodule

module i2c_byte_engine_ina219 #(
    parameter integer CLK_DIV=32,
    parameter integer STRETCH_TIMEOUT_TICKS=8192
) (
    input wire clk,input wire reset_n,input wire cmd_start,input wire [2:0] cmd,
    input wire [7:0] tx_data,output reg busy,output reg done,output reg ack_error,
    output reg [7:0] rx_data,inout wire i2c_scl,inout wire i2c_sda
);
    localparam [2:0] C_START=0,C_STOP=1,C_WRITE=2,C_READ_ACK=3,C_READ_NACK=4;
    localparam [4:0] P_IDLE=0,P_START_SETUP=1,P_START_RAISE=2,P_START_FALL=3,
      P_STOP_SETUP=4,P_STOP_RAISE=5,P_STOP_RELEASE=6,P_W_SETUP=7,P_W_RAISE=8,
      P_W_HIGH=9,P_W_FALL=10,P_ACK_SETUP=11,P_ACK_RAISE=12,P_ACK_HIGH=13,
      P_ACK_FALL=14,P_R_SETUP=15,P_R_RAISE=16,P_R_HIGH=17,P_R_FALL=18,
      P_MACK_SETUP=19,P_MACK_RAISE=20,P_MACK_HIGH=21,P_MACK_FALL=22,P_DONE=23;
    localparam integer SAFE_CLK_DIV=(CLK_DIV<1)?1:CLK_DIV;
    localparam integer STRETCH_LIMIT=(STRETCH_TIMEOUT_TICKS<1)?1:STRETCH_TIMEOUT_TICKS;
    reg [4:0] state;reg [31:0] div_count;reg [31:0] stretch_count;
    reg scl_low,sda_low;reg [2:0] bit_index;
    reg [7:0] tx_latched,rx_shift;reg [2:0] job_cmd;
    wire scl_in=i2c_scl;wire sda_in=i2c_sda;
    assign i2c_scl=scl_low?1'b0:1'bz;assign i2c_sda=sda_low?1'b0:1'bz;
    always @(posedge clk or negedge reset_n) begin
      if(!reset_n)begin state<=P_IDLE;div_count<=0;stretch_count<=0;scl_low<=0;sda_low<=0;bit_index<=0;
        tx_latched<=0;rx_shift<=0;job_cmd<=0;busy<=0;done<=0;ack_error<=0;rx_data<=0;end
      else begin done<=0;
        if(state==P_IDLE)begin div_count<=0;stretch_count<=0;busy<=0;if(cmd_start)begin busy<=1;ack_error<=0;
          job_cmd<=cmd;tx_latched<=tx_data;bit_index<=7;rx_shift<=0;
          case(cmd)C_START:state<=P_START_SETUP;C_STOP:state<=P_STOP_SETUP;
            C_WRITE:state<=P_W_SETUP;default:state<=P_R_SETUP;endcase end end
        else if(div_count>=SAFE_CLK_DIV-1)begin div_count<=0;case(state)
          P_START_SETUP:begin scl_low<=1;sda_low<=0;state<=P_START_RAISE;end
          P_START_RAISE:begin
            scl_low<=0;
            if(scl_in===1'b1 && sda_in===1'b1)begin stretch_count<=0;state<=P_START_FALL;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;
          end
          P_START_FALL:begin sda_low<=1;scl_low<=0;state<=P_W_FALL;end
          P_STOP_SETUP:begin scl_low<=1;sda_low<=1;state<=P_STOP_RAISE;end
          P_STOP_RAISE:begin scl_low<=0;if(scl_in===1'b1)begin stretch_count<=0;state<=P_STOP_RELEASE;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;end
          P_STOP_RELEASE:begin sda_low<=0;state<=P_DONE;end
          P_W_SETUP:begin scl_low<=1;sda_low<=~tx_latched[bit_index];state<=P_W_RAISE;end
          P_W_RAISE:begin scl_low<=0;if(scl_in===1'b1)begin stretch_count<=0;state<=P_W_HIGH;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;end
          P_W_HIGH:state<=P_W_FALL;
          P_W_FALL:begin scl_low<=1;if(job_cmd==C_START)begin job_cmd<=C_WRITE;bit_index<=7;state<=P_DONE;end
            else if(bit_index==0)state<=P_ACK_SETUP;else begin bit_index<=bit_index-1'b1;state<=P_W_SETUP;end end
          P_ACK_SETUP:begin scl_low<=1;sda_low<=0;state<=P_ACK_RAISE;end
          P_ACK_RAISE:begin scl_low<=0;if(scl_in===1'b1)begin stretch_count<=0;state<=P_ACK_HIGH;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;end
          P_ACK_HIGH:begin ack_error<=(sda_in!==1'b0);state<=P_ACK_FALL;end
          P_ACK_FALL:begin scl_low<=1;state<=P_DONE;end
          P_R_SETUP:begin scl_low<=1;sda_low<=0;state<=P_R_RAISE;end
          P_R_RAISE:begin scl_low<=0;if(scl_in===1'b1)begin stretch_count<=0;state<=P_R_HIGH;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;end
          P_R_HIGH:begin rx_shift[bit_index]<=sda_in;state<=P_R_FALL;end
          P_R_FALL:begin scl_low<=1;if(bit_index==0)state<=P_MACK_SETUP;
            else begin bit_index<=bit_index-1'b1;state<=P_R_SETUP;end end
          P_MACK_SETUP:begin scl_low<=1;sda_low<=(job_cmd==C_READ_ACK);state<=P_MACK_RAISE;end
          P_MACK_RAISE:begin scl_low<=0;if(scl_in===1'b1)begin stretch_count<=0;state<=P_MACK_HIGH;end
            else if(stretch_count>=STRETCH_LIMIT-1)begin ack_error<=1;scl_low<=0;sda_low<=0;stretch_count<=0;state<=P_DONE;end
            else stretch_count<=stretch_count+1'b1;end
          P_MACK_HIGH:state<=P_MACK_FALL;
          P_MACK_FALL:begin scl_low<=1;sda_low<=0;rx_data<=rx_shift;state<=P_DONE;end
          P_DONE:begin busy<=0;done<=1;state<=P_IDLE;end default:state<=P_IDLE;
        endcase end else div_count<=div_count+1'b1;
      end
    end
endmodule
