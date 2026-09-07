`timescale 1ns/1ps

// VL53L0X 完整基础初始化与连续测距例程。
// 流程基于 ST API/Pololu 实现：型号核对、stop variable、工厂 SPAD 映射、
// 默认调参表、VHV/phase 参考校准、back-to-back 连续测距。
module vl53l0x_i2c #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer I2C_STRETCH_TIMEOUT_TICKS = 8192,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000,
    parameter integer POLL_RETRY_LIMIT = 255,
    parameter IOVDD_2V8 = 1'b1
) (
    input wire clk,input wire reset_n,inout wire i2c_scl,inout wire i2c_sda,
    output reg sensor_ok,output reg initialized,output reg identity_error,
    output reg bus_error,output reg timeout_error,
    output reg [7:0] model_id,output reg [2:0] interrupt_status,
    // RESULT_RANGE_STATUS(0x14) 原始字节，仅供上层诊断；本例不把它误解为
    // ST API 已映射的质量枚举。
    output reg [7:0] range_status_raw,
    // 单周期 ready 脉冲：表示本轮就绪、距离读取和中断清除均成功；
    // 它不等价于“测距精度/目标质量一定合格”。
    output reg [15:0] distance_mm,output reg distance_valid
);
    localparam [6:0] I2C_ADDR=7'h29;
    localparam [4:0] ST_DELAY=0,ST_ID_START=1,ST_ID_WAIT=2,
      ST_PRE_START=3,ST_PRE_WAIT=4,ST_SPAD_START=5,ST_SPAD_WAIT=6,
      ST_SPAD_FILTER=7,ST_SPADCFG_START=8,ST_SPADCFG_WAIT=9,
      ST_TUNE_START=10,ST_TUNE_WAIT=11,ST_POST_START=12,ST_POST_WAIT=13,
      ST_CAL_START=14,ST_CAL_WAIT=15,ST_SAMPLE_DELAY=16,
      ST_STATUS_START=17,ST_STATUS_WAIT=18,ST_RANGE_START=19,
      ST_RANGE_WAIT=20,ST_CLEAR_START=21,ST_CLEAR_WAIT=22,ST_PUBLISH=23,
      ST_RANGE_STATUS_START=24,ST_RANGE_STATUS_WAIT=25;
    localparam integer STARTUP_DELAY=(STARTUP_DELAY_CYCLES<1)?1:STARTUP_DELAY_CYCLES;
    localparam integer SAMPLE_INTERVAL=(SAMPLE_INTERVAL_CYCLES<1)?1:SAMPLE_INTERVAL_CYCLES;
    localparam integer POLL_LIMIT=(POLL_RETRY_LIMIT<1)?1:POLL_RETRY_LIMIT;
    reg [4:0] state;
    reg [6:0] op_index;
    reg [31:0] delay_count;
    reg [15:0] poll_count;
    reg txn_start,txn_read;
    reg [7:0] txn_reg;
    reg [31:0] txn_write_data;
    reg [2:0] txn_write_length;
    reg [4:0] txn_read_length;
    wire txn_busy,txn_done,txn_error,txn_rx_valid;
    wire [4:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes[0:5];
    reg [7:0] scratch,stop_variable,spad_count;
    reg spad_is_aperture;
    reg [7:0] spad_map[0:5];
    reg [5:0] spad_bit_index;
    reg [6:0] spads_enabled;
    reg [15:0] pending_distance;
    integer i;

    function pre_is_read;input[6:0]n;begin pre_is_read=(n==0||n==6||n==10);end endfunction
    function [7:0] pre_reg;input[6:0]n;begin case(n)
      0:pre_reg=8'h89;1:pre_reg=8'h89;2:pre_reg=8'h88;3:pre_reg=8'h80;
      4:pre_reg=8'hFF;5:pre_reg=8'h00;6:pre_reg=8'h91;7:pre_reg=8'h00;
      8:pre_reg=8'hFF;9:pre_reg=8'h80;10:pre_reg=8'h60;11:pre_reg=8'h60;
      12:pre_reg=8'h44;default:pre_reg=8'h01;endcase end endfunction
    function [31:0] pre_data;input[6:0]n;input[7:0]pad;input[7:0]tmp;begin case(n)
      1:pre_data={24'd0,(IOVDD_2V8?(pad|8'h01):pad)};2:pre_data=0;
      3:pre_data=1;4:pre_data=1;5:pre_data=0;7:pre_data=1;8:pre_data=0;
      9:pre_data=0;11:pre_data={24'd0,(tmp|8'h12)};12:pre_data=32'h00000020;
      default:pre_data=32'h000000FF;endcase end endfunction
    function [2:0] pre_wlen;input[6:0]n;begin pre_wlen=(n==12)?2:1;end endfunction

    function spad_is_read_op;input[6:0]n;begin
      spad_is_read_op=(n==4||n==11||n==13||n==16||n==22);end endfunction
    function [7:0] spad_reg;input[6:0]n;begin case(n)
      0:spad_reg=8'h80;1:spad_reg=8'hFF;2:spad_reg=8'h00;3:spad_reg=8'hFF;
      4:spad_reg=8'h83;5:spad_reg=8'h83;6:spad_reg=8'hFF;7:spad_reg=8'h81;
      8:spad_reg=8'h80;9:spad_reg=8'h94;10:spad_reg=8'h83;11:spad_reg=8'h83;
      12:spad_reg=8'h83;13:spad_reg=8'h92;14:spad_reg=8'h81;15:spad_reg=8'hFF;
      16:spad_reg=8'h83;17:spad_reg=8'h83;18:spad_reg=8'hFF;19:spad_reg=8'h00;
      20:spad_reg=8'hFF;21:spad_reg=8'h80;default:spad_reg=8'hB0;endcase end endfunction
    function [7:0] spad_data;input[6:0]n;input[7:0]tmp;begin case(n)
      0:spad_data=1;1:spad_data=1;2:spad_data=0;3:spad_data=6;
      5:spad_data=tmp|8'h04;6:spad_data=7;7:spad_data=1;8:spad_data=1;
      9:spad_data=8'h6B;10:spad_data=0;12:spad_data=1;14:spad_data=0;
      15:spad_data=6;17:spad_data=tmp&8'hFB;18:spad_data=1;19:spad_data=1;
      20:spad_data=0;default:spad_data=0;endcase end endfunction

    function [7:0] spadcfg_reg;input[6:0]n;begin case(n)
      0:spadcfg_reg=8'hFF;1:spadcfg_reg=8'h4F;2:spadcfg_reg=8'h4E;
      3:spadcfg_reg=8'hFF;4:spadcfg_reg=8'hB6;
      5:spadcfg_reg=8'hB0;6:spadcfg_reg=8'hB1;7:spadcfg_reg=8'hB2;
      8:spadcfg_reg=8'hB3;9:spadcfg_reg=8'hB4;default:spadcfg_reg=8'hB5;
      endcase end endfunction
    function [7:0] spadcfg_data;input[6:0]n;begin case(n)
      0:spadcfg_data=1;1:spadcfg_data=0;2:spadcfg_data=8'h2C;
      3:spadcfg_data=0;4:spadcfg_data=8'hB4;
      5:spadcfg_data=spad_map[0];6:spadcfg_data=spad_map[1];
      7:spadcfg_data=spad_map[2];8:spadcfg_data=spad_map[3];
      9:spadcfg_data=spad_map[4];default:spadcfg_data=spad_map[5];endcase end endfunction

    function [7:0] tune_reg;input[6:0]n;begin case(n)
      0:tune_reg=8'hFF;1:tune_reg=8'h00;2:tune_reg=8'hFF;3:tune_reg=8'h09;
      4:tune_reg=8'h10;5:tune_reg=8'h11;6:tune_reg=8'h24;7:tune_reg=8'h25;
      8:tune_reg=8'h75;9:tune_reg=8'hFF;10:tune_reg=8'h4E;11:tune_reg=8'h48;
      12:tune_reg=8'h30;13:tune_reg=8'hFF;14:tune_reg=8'h30;15:tune_reg=8'h54;
      16:tune_reg=8'h31;17:tune_reg=8'h32;18:tune_reg=8'h40;19:tune_reg=8'h46;
      20:tune_reg=8'h60;21:tune_reg=8'h27;22:tune_reg=8'h50;23:tune_reg=8'h51;
      24:tune_reg=8'h52;25:tune_reg=8'h56;26:tune_reg=8'h57;27:tune_reg=8'h61;
      28:tune_reg=8'h62;29:tune_reg=8'h64;30:tune_reg=8'h65;31:tune_reg=8'h66;
      32:tune_reg=8'hFF;33:tune_reg=8'h22;34:tune_reg=8'h47;35:tune_reg=8'h49;
      36:tune_reg=8'h4A;37:tune_reg=8'hFF;38:tune_reg=8'h7A;39:tune_reg=8'h7B;
      40:tune_reg=8'h78;41:tune_reg=8'hFF;42:tune_reg=8'h23;43:tune_reg=8'h42;
      44:tune_reg=8'h44;45:tune_reg=8'h45;46:tune_reg=8'h46;47:tune_reg=8'h40;
      48:tune_reg=8'h0E;49:tune_reg=8'h20;50:tune_reg=8'h43;51:tune_reg=8'hFF;
      52:tune_reg=8'h34;53:tune_reg=8'h35;54:tune_reg=8'hFF;55:tune_reg=8'h31;
      56:tune_reg=8'h4B;57:tune_reg=8'h4C;58:tune_reg=8'h4D;59:tune_reg=8'hFF;
      60:tune_reg=8'h44;61:tune_reg=8'h45;62:tune_reg=8'h47;63:tune_reg=8'h48;
      64:tune_reg=8'h67;65:tune_reg=8'h70;66:tune_reg=8'h71;67:tune_reg=8'h72;
      68:tune_reg=8'h76;69:tune_reg=8'h77;70:tune_reg=8'hFF;71:tune_reg=8'h0D;
      72:tune_reg=8'hFF;73:tune_reg=8'h80;74:tune_reg=8'h01;75:tune_reg=8'hFF;
      76:tune_reg=8'h8E;77:tune_reg=8'h00;78:tune_reg=8'hFF;default:tune_reg=8'h80;
    endcase end endfunction
    function [7:0] tune_data;input[6:0]n;begin case(n)
      0:tune_data=1;1:tune_data=0;2:tune_data=0;3:tune_data=0;4:tune_data=0;
      5:tune_data=0;6:tune_data=1;7:tune_data=8'hFF;8:tune_data=0;9:tune_data=1;
      10:tune_data=8'h2C;11:tune_data=0;12:tune_data=8'h20;13:tune_data=0;
      14:tune_data=9;15:tune_data=0;16:tune_data=4;17:tune_data=3;18:tune_data=8'h83;
      19:tune_data=8'h25;20:tune_data=0;21:tune_data=0;22:tune_data=6;
      23:tune_data=0;24:tune_data=8'h96;25:tune_data=8;26:tune_data=8'h30;
      27:tune_data=0;28:tune_data=0;29:tune_data=0;30:tune_data=0;
      31:tune_data=8'hA0;32:tune_data=1;33:tune_data=8'h32;34:tune_data=8'h14;
      35:tune_data=8'hFF;36:tune_data=0;37:tune_data=0;38:tune_data=8'h0A;
      39:tune_data=0;40:tune_data=8'h21;41:tune_data=1;42:tune_data=8'h34;
      43:tune_data=0;44:tune_data=8'hFF;45:tune_data=8'h26;46:tune_data=5;
      47:tune_data=8'h40;48:tune_data=6;49:tune_data=8'h1A;50:tune_data=8'h40;
      51:tune_data=0;52:tune_data=3;53:tune_data=8'h44;54:tune_data=1;
      55:tune_data=4;56:tune_data=9;57:tune_data=5;58:tune_data=4;59:tune_data=0;
      60:tune_data=0;61:tune_data=8'h20;62:tune_data=8;63:tune_data=8'h28;
      64:tune_data=0;65:tune_data=4;66:tune_data=1;67:tune_data=8'hFE;
      68:tune_data=0;69:tune_data=0;70:tune_data=1;71:tune_data=1;
      72:tune_data=0;73:tune_data=1;74:tune_data=8'hF8;75:tune_data=1;
      76:tune_data=1;77:tune_data=1;78:tune_data=0;default:tune_data=0;
    endcase end endfunction

    function post_is_read;input[6:0]n;begin post_is_read=(n==1);end endfunction
    function [7:0] post_reg;input[6:0]n;begin case(n)
      0:post_reg=8'h0A;1:post_reg=8'h84;2:post_reg=8'h84;
      3:post_reg=8'h0B;default:post_reg=8'h01;endcase end endfunction
    function [7:0] post_data;input[6:0]n;input[7:0]tmp;begin case(n)
      0:post_data=4;2:post_data=tmp&8'hEF;3:post_data=1;default:post_data=8'hE8;
    endcase end endfunction

    function cal_is_read;input[6:0]n;begin cal_is_read=(n==2||n==7);end endfunction
    function [7:0] cal_reg;input[6:0]n;begin case(n)
      0:cal_reg=1;1:cal_reg=0;2:cal_reg=8'h13;3:cal_reg=8'h0B;4:cal_reg=0;
      5:cal_reg=1;6:cal_reg=0;7:cal_reg=8'h13;8:cal_reg=8'h0B;9:cal_reg=0;
      10:cal_reg=1;11:cal_reg=8'h80;12:cal_reg=8'hFF;13:cal_reg=0;
      14:cal_reg=8'h91;15:cal_reg=0;16:cal_reg=8'hFF;17:cal_reg=8'h80;
      default:cal_reg=0;endcase end endfunction
    function [7:0] cal_data;input[6:0]n;begin case(n)
      0:cal_data=1;1:cal_data=8'h41;3:cal_data=1;4:cal_data=0;
      5:cal_data=2;6:cal_data=1;8:cal_data=1;9:cal_data=0;
      10:cal_data=8'hE8;11:cal_data=1;12:cal_data=1;13:cal_data=0;
      14:cal_data=stop_variable;15:cal_data=1;16:cal_data=0;17:cal_data=0;
      default:cal_data=2;endcase end endfunction

    i2c_register8_master #(.CLK_DIV(I2C_CLK_DIV),
      .STRETCH_TIMEOUT_TICKS(I2C_STRETCH_TIMEOUT_TICKS))u_i2c(
      .clk(clk),.reset_n(reset_n),.start(txn_start),.device_addr(I2C_ADDR),
      .reg_addr(txn_reg),.is_read(txn_read),.write_data(txn_write_data),
      .write_length(txn_write_length),.read_length(txn_read_length),
      .busy(txn_busy),.done(txn_done),.error(txn_error),.rx_valid(txn_rx_valid),
      .rx_index(txn_rx_index),.rx_data(txn_rx_data),.i2c_scl(i2c_scl),.i2c_sda(i2c_sda));

    always @(posedge clk or negedge reset_n) begin
      if(!reset_n)begin
        state<=ST_DELAY;op_index<=0;delay_count<=0;poll_count<=0;txn_start<=0;
        txn_read<=0;txn_reg<=0;txn_write_data<=0;txn_write_length<=0;txn_read_length<=0;
        sensor_ok<=0;initialized<=0;identity_error<=0;bus_error<=0;timeout_error<=0;
        model_id<=0;interrupt_status<=0;range_status_raw<=0;distance_mm<=0;distance_valid<=0;
        scratch<=0;stop_variable<=0;spad_count<=0;spad_is_aperture<=0;
        spad_bit_index<=0;spads_enabled<=0;pending_distance<=0;
        for(i=0;i<6;i=i+1)begin rx_bytes[i]<=0;spad_map[i]<=0;end
      end else begin
        txn_start<=0;distance_valid<=0;
        if(txn_rx_valid&&txn_rx_index<6)rx_bytes[txn_rx_index]<=txn_rx_data;
        case(state)
          ST_DELAY:if(delay_count>=STARTUP_DELAY-1)begin delay_count<=0;state<=ST_ID_START;end
                   else delay_count<=delay_count+1'b1;
          ST_ID_START:if(!txn_busy)begin txn_read<=1;txn_reg<=8'hC0;txn_write_length<=0;
            txn_read_length<=1;txn_start<=1;state<=ST_ID_WAIT;end
          ST_ID_WAIT:if(txn_done)begin
            model_id<=rx_bytes[0];
            if(txn_error)begin bus_error<=1;sensor_ok<=0;initialized<=0;delay_count<=0;state<=ST_DELAY;end
            else if(rx_bytes[0]!=8'hEE)begin identity_error<=1;sensor_ok<=0;initialized<=0;delay_count<=0;state<=ST_DELAY;end
            else begin identity_error<=0;sensor_ok<=1;op_index<=0;state<=ST_PRE_START;end
          end
          ST_PRE_START:if(!txn_busy)begin txn_read<=pre_is_read(op_index);txn_reg<=pre_reg(op_index);
            txn_write_data<=pre_data(op_index,scratch,scratch);txn_write_length<=pre_is_read(op_index)?0:pre_wlen(op_index);
            txn_read_length<=pre_is_read(op_index)?1:0;txn_start<=1;state<=ST_PRE_WAIT;end
          ST_PRE_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else begin
              if(op_index==0)scratch<=rx_bytes[0];
              if(op_index==6)stop_variable<=rx_bytes[0];
              if(op_index==10)scratch<=rx_bytes[0];
              if(op_index==13)begin op_index<=0;poll_count<=0;state<=ST_SPAD_START;end
              else begin op_index<=op_index+1'b1;state<=ST_PRE_START;end
            end
          end
          ST_SPAD_START:if(!txn_busy)begin txn_read<=spad_is_read_op(op_index);txn_reg<=spad_reg(op_index);
            txn_write_data<={24'd0,spad_data(op_index,scratch)};txn_write_length<=spad_is_read_op(op_index)?0:1;
            txn_read_length<=(op_index==22)?6:(spad_is_read_op(op_index)?1:0);txn_start<=1;state<=ST_SPAD_WAIT;end
          ST_SPAD_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else if(op_index==11&&rx_bytes[0]==0)begin
              if(poll_count>=POLL_LIMIT-1)begin timeout_error<=1;initialized<=0;delay_count<=0;state<=ST_DELAY;end
              else begin poll_count<=poll_count+1'b1;state<=ST_SPAD_START;end
            end else begin
              if(op_index==4||op_index==16)scratch<=rx_bytes[0];
              if(op_index==13)begin spad_count<=rx_bytes[0]&8'h7F;spad_is_aperture<=rx_bytes[0][7];end
              if(op_index==22)begin
                for(i=0;i<6;i=i+1)spad_map[i]<=rx_bytes[i];
                spad_bit_index<=0;spads_enabled<=0;state<=ST_SPAD_FILTER;
              end else begin op_index<=op_index+1'b1;state<=ST_SPAD_START;end
            end
          end
          ST_SPAD_FILTER:begin
            if(spad_bit_index<(spad_is_aperture?12:0)||spads_enabled>=spad_count)
              spad_map[spad_bit_index>>3][spad_bit_index[2:0]]<=1'b0;
            else if(spad_map[spad_bit_index>>3][spad_bit_index[2:0]])spads_enabled<=spads_enabled+1'b1;
            if(spad_bit_index==47)begin op_index<=0;state<=ST_SPADCFG_START;end
            else spad_bit_index<=spad_bit_index+1'b1;
          end
          ST_SPADCFG_START:if(!txn_busy)begin txn_read<=0;txn_reg<=spadcfg_reg(op_index);
            txn_write_data<={24'd0,spadcfg_data(op_index)};txn_write_length<=1;txn_read_length<=0;
            txn_start<=1;state<=ST_SPADCFG_WAIT;end
          ST_SPADCFG_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else if(op_index==10)begin op_index<=0;state<=ST_TUNE_START;end
            else begin op_index<=op_index+1'b1;state<=ST_SPADCFG_START;end
          end
          ST_TUNE_START:if(!txn_busy)begin txn_read<=0;txn_reg<=tune_reg(op_index);
            txn_write_data<={24'd0,tune_data(op_index)};txn_write_length<=1;txn_read_length<=0;
            txn_start<=1;state<=ST_TUNE_WAIT;end
          ST_TUNE_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else if(op_index==79)begin op_index<=0;state<=ST_POST_START;end
            else begin op_index<=op_index+1'b1;state<=ST_TUNE_START;end
          end
          ST_POST_START:if(!txn_busy)begin txn_read<=post_is_read(op_index);txn_reg<=post_reg(op_index);
            txn_write_data<={24'd0,post_data(op_index,scratch)};txn_write_length<=post_is_read(op_index)?0:1;
            txn_read_length<=post_is_read(op_index)?1:0;txn_start<=1;state<=ST_POST_WAIT;end
          ST_POST_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else begin
              if(op_index==1)scratch<=rx_bytes[0];
              if(op_index==4)begin op_index<=0;poll_count<=0;state<=ST_CAL_START;end
              else begin op_index<=op_index+1'b1;state<=ST_POST_START;end
            end
          end
          ST_CAL_START:if(!txn_busy)begin txn_read<=cal_is_read(op_index);txn_reg<=cal_reg(op_index);
            txn_write_data<={24'd0,cal_data(op_index)};txn_write_length<=cal_is_read(op_index)?0:1;
            txn_read_length<=cal_is_read(op_index)?1:0;txn_start<=1;state<=ST_CAL_WAIT;end
          ST_CAL_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;initialized<=0;sensor_ok<=0;delay_count<=0;state<=ST_DELAY;end
            else if((op_index==2||op_index==7)&&(rx_bytes[0]&7)==0)begin
              if(poll_count>=POLL_LIMIT-1)begin timeout_error<=1;initialized<=0;delay_count<=0;state<=ST_DELAY;end
              else begin poll_count<=poll_count+1'b1;state<=ST_CAL_START;end
            end else if(op_index==18)begin initialized<=1;delay_count<=0;state<=ST_SAMPLE_DELAY;end
            else begin if(op_index==2||op_index==7)poll_count<=0;op_index<=op_index+1'b1;state<=ST_CAL_START;end
          end
          ST_SAMPLE_DELAY:if(delay_count>=SAMPLE_INTERVAL-1)begin delay_count<=0;state<=ST_STATUS_START;end
                          else delay_count<=delay_count+1'b1;
          ST_STATUS_START:if(!txn_busy)begin txn_read<=1;txn_reg<=8'h13;txn_write_length<=0;
            txn_read_length<=1;txn_start<=1;state<=ST_STATUS_WAIT;end
          ST_STATUS_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;state<=ST_SAMPLE_DELAY;end
            else begin interrupt_status<=rx_bytes[0][2:0];if((rx_bytes[0]&7)!=0)state<=ST_RANGE_STATUS_START;else state<=ST_SAMPLE_DELAY;end
          end
          ST_RANGE_STATUS_START:if(!txn_busy)begin txn_read<=1;txn_reg<=8'h14;txn_write_length<=0;
            txn_read_length<=1;txn_start<=1;state<=ST_RANGE_STATUS_WAIT;end
          ST_RANGE_STATUS_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;state<=ST_SAMPLE_DELAY;end
            else begin range_status_raw<=rx_bytes[0];state<=ST_RANGE_START;end
          end
          ST_RANGE_START:if(!txn_busy)begin txn_read<=1;txn_reg<=8'h1E;txn_write_length<=0;
            txn_read_length<=2;txn_start<=1;state<=ST_RANGE_WAIT;end
          ST_RANGE_WAIT:if(txn_done)begin
            if(txn_error)begin bus_error<=1;state<=ST_SAMPLE_DELAY;end
            else begin pending_distance<={rx_bytes[0],rx_bytes[1]};state<=ST_CLEAR_START;end
          end
          ST_CLEAR_START:if(!txn_busy)begin txn_read<=0;txn_reg<=8'h0B;txn_write_data<=1;
            txn_write_length<=1;txn_read_length<=0;txn_start<=1;state<=ST_CLEAR_WAIT;end
          ST_CLEAR_WAIT:if(txn_done)begin if(txn_error)begin bus_error<=1;state<=ST_SAMPLE_DELAY;end else state<=ST_PUBLISH;end
          ST_PUBLISH:begin distance_mm<=pending_distance;distance_valid<=1;state<=ST_SAMPLE_DELAY;end
          default:state<=ST_DELAY;
        endcase
      end
    end
endmodule
