// ==================== camera_init.v ====================
/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2019/05/01 00:00:00
// Module Name   : camera_init
// Description   : 摄像头初始化驱动模块，仅支持本板OV5647 CSI-2 RAW10
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module camera_init #(
  parameter SYS_CLOCK       = 50_000_000,//系统时钟采用50MHz
  parameter SCL_CLOCK       = 400_000,   //SCL总线时钟采用400kHz
  parameter integer LP11_RESET_PROBE = 0 //仅隔离验证候选启用
)
(
  input      Clk,
  input      Rst_n,

  output reg Init_Done,
  output     camera_rst_n,
  output     camera_pwdn,

  output     i2c_sclk,
  inout      i2c_sdat,
  input      [1:0] lp_clk_state,
  output     dphy_rx_reset_n,
  output     [30:0] lp11_probe_status
);
  localparam DELAY_1ms_CNT  = 1_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_21ms_CNT = 21_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_5ms_CNT  = 5_000_000/(1_000_000_000/SYS_CLOCK);
  localparam integer LP11_PROBE_ACTIVE = LP11_RESET_PROBE;
  localparam integer LP11_QUIET_CYCLES = (SYS_CLOCK >= 1000) ? (SYS_CLOCK / 1000) : 1;

  localparam [3:0] PROBE_STOP       = 4'd0;
  localparam [3:0] PROBE_WAIT_LP11  = 4'd1;
  localparam [3:0] PROBE_HOLD_LP11  = 4'd2;
  localparam [3:0] PROBE_RUN        = 4'd3;
  localparam [3:0] PROBE_FAULT      = 4'd4;

  wire [15:0]addr;
  wire [15:0]table_addr;
  reg wrreg_req;
  wire [7:0]wrdata;
  wire [7:0]table_wrdata;
  wire [7:0]rddata;
  wire RW_Done;
  wire ack;
  reg [31:0] i2c_dly_cnt_max;

  reg [7:0]cnt;
  wire [23:0]lut;

  wire [7:0]lut_size;
  wire [7:0]device_id;
  wire addr_mode;
  reg [3:0] probe_state;
  reg [1:0] probe_stop_index;
  reg [31:0] probe_quiet_count;
  reg [7:0] probe_reset_release_count;
  reg probe_fault;
  reg probe_lp11_seen;
  reg [1:0] probe_lp_clk00_count;
  reg probe_rx_release_done;
  (* ASYNC_REG="TRUE" *) reg [1:0] lp_clk_meta;
  (* ASYNC_REG="TRUE" *) reg [1:0] lp_clk_sync;
  wire probe_stop_override = LP11_PROBE_ACTIVE && (cnt == 8'd82) &&
                             (probe_state == PROBE_STOP);

  assign addr = probe_stop_override ?
                ((probe_stop_index == 2'd0) ? 16'h4800 :
                 (probe_stop_index == 2'd1) ? 16'h4202 : 16'h300d) : table_addr;
  assign wrdata = probe_stop_override ?
                  ((probe_stop_index == 2'd0) ? 8'h25 :
                   (probe_stop_index == 2'd1) ? 8'h0f : 8'h01) : table_wrdata;
  assign dphy_rx_reset_n = LP11_PROBE_ACTIVE ?
                           ((probe_state == PROBE_RUN) &&
                            probe_rx_release_done && !probe_fault) : Rst_n;
  assign lp11_probe_status = {
    probe_state,
    (LP11_PROBE_ACTIVE ? lp_clk_sync : 2'b00),
    (LP11_PROBE_ACTIVE && (lp_clk_sync == 2'b11)),
    dphy_rx_reset_n,
    probe_reset_release_count,
    probe_fault,
    Init_Done,
    probe_stop_index,
    probe_quiet_count[10:0]
  };

  assign camera_pwdn = 0;

  // 本板固定使用 OV5647 CSI-2 RAW10 初始化表。
    assign device_id = 8'h6c;
    assign addr_mode = 1'b1;
    assign table_addr = lut[23:8];
    assign table_wrdata = lut[7:0];
    // OV5647 初始化表共 96 项，含 Linux 流启动阶段的控制寄存器。
    assign lut_size = 96;

    ov5647_init_table_raw10 ov5647_init_table_raw10_inst
    (
      .addr (cnt ),
      .clk  (Clk ),
      .q    (lut )
    );

  // ACK/NACK 由 camera_init 的现有 I2C 完成边界提供；探针状态机只在 ACK 后推进。
  // CEN 探针在 LUT 即将写 4800=04 时插入 Linux 停流三写，并用 50 MHz 计数确认 LP-11。
  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)begin
    lp_clk_meta <= 2'b00;
    lp_clk_sync <= 2'b00;
    probe_state <= LP11_PROBE_ACTIVE ? PROBE_STOP : PROBE_RUN;
    probe_stop_index <= 2'd0;
    probe_quiet_count <= 32'd0;
    probe_reset_release_count <= 8'd0;
    probe_fault <= 1'b0;
    probe_lp11_seen <= 1'b0;
    probe_lp_clk00_count <= 2'd0;
    probe_rx_release_done <= 1'b0;
  end
  else begin
    lp_clk_meta <= lp_clk_state;
    lp_clk_sync <= lp_clk_meta;
    if(LP11_PROBE_ACTIVE)begin
      if(RW_Done && ack)begin
        probe_fault <= 1'b1;
        probe_state <= PROBE_FAULT;
        probe_quiet_count <= 32'd0;
      end
      else begin
        case(probe_state)
          PROBE_STOP:
            if((cnt == 8'd82) && RW_Done && (!ack))begin
              if(probe_stop_index == 2'd2)begin
                probe_stop_index <= 2'd0;
                probe_quiet_count <= 32'd0;
                probe_state <= PROBE_WAIT_LP11;
              end
              else
                probe_stop_index <= probe_stop_index + 1'b1;
            end
          PROBE_WAIT_LP11:
            if(lp_clk_sync == 2'b11)begin
              if(probe_quiet_count >= LP11_QUIET_CYCLES - 1)begin
                probe_quiet_count <= 32'd0;
                probe_state <= PROBE_HOLD_LP11;
              end
              else
                probe_quiet_count <= probe_quiet_count + 1'b1;
            end
            else
              probe_quiet_count <= 32'd0;
          PROBE_HOLD_LP11:
            if(lp_clk_sync == 2'b11)begin
              if(probe_quiet_count >= LP11_QUIET_CYCLES - 1)begin
                probe_quiet_count <= 32'd0;
                probe_state <= PROBE_RUN;
              end
              else
                probe_quiet_count <= probe_quiet_count + 1'b1;
            end
            else begin
              // LP-11 在第二段保持窗口中断时，先重新断言接收器复位再从头计时。
              probe_quiet_count <= 32'd0;
              probe_state <= PROBE_WAIT_LP11;
            end
          default:
            probe_quiet_count <= 32'd0;
        endcase

        // WAIT 完成完整 1 ms LP-11 窗口后才允许后续流启动触发 D-PHY 释放。
        if((probe_state == PROBE_WAIT_LP11) &&
           (lp_clk_sync == 2'b11) &&
           (probe_quiet_count >= LP11_QUIET_CYCLES - 1))
          probe_lp11_seen <= 1'b1;

        // I2C 初始化独立运行在 50 MHz；整表完成后，需观测 LP-11 后连续三拍 LP-clock=00。
        // 释放只锁存一次；释放后的 LP-11 返回不会重新断言接收器复位。
        if(!probe_rx_release_done)begin
          if((probe_state == PROBE_RUN) && !probe_fault && probe_lp11_seen &&
             Init_Done && (lp_clk_sync == 2'b00))begin
            if(probe_lp_clk00_count == 2'd2)begin
              probe_rx_release_done <= 1'b1;
              probe_reset_release_count <= probe_reset_release_count + 1'b1;
            end
            else
              probe_lp_clk00_count <= probe_lp_clk00_count + 1'b1;
          end
          else
            probe_lp_clk00_count <= 2'd0;
        end
      end
    end
    else begin
      probe_state <= PROBE_RUN;
      probe_stop_index <= 2'd0;
      probe_quiet_count <= 32'd0;
      probe_reset_release_count <= 8'd0;
      probe_fault <= 1'b0;
      probe_lp11_seen <= 1'b0;
      probe_lp_clk00_count <= 2'd0;
      probe_rx_release_done <= 1'b0;
    end
  end


  i2c_control #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_control(
    .Clk         (Clk             ),
    .Rst_n       (Rst_n           ),
    .wrreg_req   (wrreg_req       ),
    .rdreg_req   (1'd0            ),
    .addr        (addr            ),
    .addr_mode   (addr_mode       ),
    .wrdata      (wrdata          ),
    .rddata      (rddata          ),
    .device_id   (device_id       ),
    .RW_Done     (RW_Done         ),
    .ack         (ack             ),
    .dly_cnt_max (i2c_dly_cnt_max ),
    .i2c_sclk    (i2c_sclk        ),
    .i2c_sdat    (i2c_sdat        )
  );

  wire Go;   //initial enable
  reg [20:0] delay_cnt;

  //上电并复位完成20ms后再配置摄像头，所以从上电到开始配置应该是1 + 20 = 21ms
  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    delay_cnt <= 21'd0;
  else if (delay_cnt >= DELAY_21ms_CNT)
    delay_cnt <= DELAY_21ms_CNT;
  else
    delay_cnt <= delay_cnt + 1'd1;

  //当延时时间到，开始使能初始化模块对OV5647的寄存器进行写入
  assign Go = (delay_cnt == DELAY_21ms_CNT - 1'b1) ? 1'b1 : 1'b0;

  //摄像头上电后保持复位1ms，所以上电后需要1ms之后再使能释放摄像头的复位信号
  //这里为了优化逻辑，简化比较器逻辑，直接使延迟比较值为DELAY_1ms_CNT
  assign camera_rst_n = (delay_cnt > DELAY_1ms_CNT);

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    cnt <= 0;
  else if(Go)
    cnt <= 0;
  else if(cnt < lut_size)begin
    if(RW_Done && (!ack))begin
      if(LP11_PROBE_ACTIVE && (cnt == 8'd82) && (probe_state != PROBE_RUN))
        cnt <= cnt;
      else
        cnt <= cnt + 1'b1;
    end
    else
      cnt <= cnt;
  end
  else
    cnt <= 0;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    Init_Done <= 0;
  else if(Go)
    Init_Done <= 0;
  else if(LP11_PROBE_ACTIVE && probe_fault)
    Init_Done <= 0;
  else if(cnt == lut_size)
    Init_Done <= 1;

  reg [1:0]state;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)begin
    state <= 0;
    wrreg_req <= 1'b0;
    i2c_dly_cnt_max <= 32'd0;
  end
  else if(LP11_PROBE_ACTIVE && probe_fault)begin
    wrreg_req <= 1'b0;
    state <= 1;
  end
  else if(LP11_PROBE_ACTIVE && (cnt == 8'd82) &&
          ((probe_state == PROBE_WAIT_LP11) || (probe_state == PROBE_HOLD_LP11)))begin
    wrreg_req <= 1'b0;
    state <= 1;
  end
  else if(cnt < lut_size)begin
    case(state)
      0:
        if(Go)
          state <= 1;
        else
          state <= 0;
      1:
        begin
          wrreg_req <= 1'b1;
          state <= 2;
          if(cnt == 1)
            i2c_dly_cnt_max <= DELAY_5ms_CNT; //延时5ms
          else
            i2c_dly_cnt_max <= 32'd0;
        end
      2:
        begin
          wrreg_req <= 1'b0;
          if(RW_Done)
            state <= 1;
          else
            state <= 2;
        end
      default:state <= 0;
    endcase
  end
  else
    state <= 0;

endmodule

// ==================== i2c_control.v ====================
/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_control
// Description   : I2C控制器顶层文件
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module i2c_control #(
  //系统时钟采用50MHz
  parameter SYS_CLOCK = 50_000_000,
  //SCL总线时钟采用400kHz
  parameter SCL_CLOCK = 400_000
)
(
  input            Clk,
  input            Rst_n,

  input            wrreg_req,
  input            rdreg_req,
  input     [15:0] addr,
  input            addr_mode,
  input     [7:0]  wrdata,
  output reg[7:0]  rddata,
  input     [7:0]  device_id,
  output reg       RW_Done,

  output reg       ack,

  input     [31:0] dly_cnt_max,
  output           i2c_sclk,
  inout            i2c_sdat
);

  reg [5:0]Cmd;
  reg [7:0]Tx_DATA;
  wire [7:0]Rx_DATA;
  wire Trans_Done;
  wire ack_o;
  reg Go;
  wire [15:0] reg_addr;

  assign reg_addr = addr_mode?addr:{addr[7:0],addr[15:8]};

  localparam
    WR   = 6'b000001,   //写请求
    STA  = 6'b000010,   //起始位请求
    RD   = 6'b000100,   //读请求
    STO  = 6'b001000,   //停止位请求
    ACK  = 6'b010000,   //应答位请求
    NACK = 6'b100000;   //无应答请求

  i2c_bit_shift #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_bit_shift(
    .Clk       (Clk        ),
    .Rst_n     (Rst_n      ),
    .Cmd       (Cmd        ),
    .Go        (Go         ),
    .Rx_DATA   (Rx_DATA    ),
    .Tx_DATA   (Tx_DATA    ),
    .Trans_Done(Trans_Done ),
    .ack_o     (ack_o      ),
    .i2c_sclk  (i2c_sclk   ),
    .i2c_sdat  (i2c_sdat   )
  );

  reg [7:0]state;
  reg [7:0]cnt;
  reg [31:0]dly_cnt;

  localparam
    IDLE         = 8'b0000_0001,   //空闲状态
    WR_REG       = 8'b0000_0010,   //写寄存器状态
    WAIT_WR_DONE = 8'b0000_0100,   //等待写寄存器完成状态
    WR_REG_DONE  = 8'b0000_1000,   //写寄存器完成状态
    RD_REG       = 8'b0001_0000,   //读寄存器状态
    WAIT_RD_DONE = 8'b0010_0000,   //等待读寄存器完成状态
    RD_REG_DONE  = 8'b0100_0000,   //读寄存器完成状态
    WAIT_DLY     = 8'b1000_0000;   //等带读写完成后延迟完成
  always@(posedge Clk or negedge Rst_n)
  if(!Rst_n)begin
    Cmd <= 6'd0;
    Tx_DATA <= 8'd0;
    Go <= 1'b0;
    rddata <= 0;
    state <= IDLE;
    ack <= 0;
    dly_cnt <= 0;
    cnt <= 0;
    RW_Done <= 1'b0;
  end
  else begin
    case(state)
      IDLE:
        begin
          cnt <= 0;
          dly_cnt <= 0;
          ack <= 0;
          RW_Done <= 1'b0;
          if(wrreg_req)
            state <= WR_REG;
          else if(rdreg_req)
            state <= RD_REG;
          else
            state <= IDLE;
        end

      WR_REG:
        begin
          state <= WAIT_WR_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1://write_byte(WR, reg_addr[15:8]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end

            2://write_byte(WR, reg_addr[7:0]);
            begin Cmd <= WR; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STO, wrdata);
            begin Cmd <= WR | STO; Tx_DATA <= wrdata; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_WR_DONE:
        begin
          Go <= 1'b0;
          if(Trans_Done)begin
            ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= WR_REG;end
              1:
                begin
                  state <= WR_REG;
                  if(addr_mode)
                    cnt <= 2;
                  else
                    cnt <= 3;
                end

              2: begin
                  cnt <= 3;
                  state <= WR_REG;
                end
              3:state <= WR_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      WR_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          state <= WAIT_DLY;
        end

      RD_REG:
        begin
          state <= WAIT_RD_DONE;
          case(cnt)
            0://write_byte(WR | STA, device_id);
            begin Cmd <= WR | STA; Tx_DATA <= device_id; Go <= 1'b1; end

            1: begin
              if(addr_mode)
                //write_byte(WR, reg_addr[15:8]);
                begin Cmd <= WR; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
              else
                //write_byte(WR | STO, reg_addr[15:8]);
                begin Cmd <= WR | STO; Tx_DATA <= reg_addr[15:8]; Go <= 1'b1; end
            end

            2://write_byte(WR | STO, reg_addr[7:0]);
            begin Cmd <= WR | STO; Tx_DATA <= reg_addr[7:0]; Go <= 1'b1; end

            3://write_byte(WR | STA, device_id | 8'd1);
            begin Cmd <= WR | STA; Tx_DATA <= device_id | 8'd1; Go <= 1'b1; end

            4://read_byte(RD | NACK | STO);
            begin Cmd <= RD | NACK | STO; Go <= 1'b1; end

            default:;
          endcase
        end

      WAIT_RD_DONE:
        begin
          Go <= 1'b0;
          if(Trans_Done)begin
            if(cnt <= 3)
              ack <= ack | ack_o;
            case(cnt)
              0: begin cnt <= 1; state <= RD_REG;end
              1:
                begin
                  state <= RD_REG;
                  if(addr_mode)
                    cnt <= 2;
                  else
                    cnt <= 3;
                end
              2: begin
                  cnt <= 3;
                  state <= RD_REG;
                end
              3:begin
                  cnt <= 4;
                  state <= RD_REG;
                end
              4:state <= RD_REG_DONE;
              default:state <= IDLE;
            endcase
          end
        end

      RD_REG_DONE:
        begin
//          RW_Done <= 1'b1;
          rddata <= Rx_DATA;
          state <= WAIT_DLY;
        end
      default:state <= IDLE;
      WAIT_DLY:
        begin
          if(dly_cnt <= dly_cnt_max) begin
            dly_cnt <= dly_cnt + 1'b1;
            state <= WAIT_DLY;
          end
         else begin
           dly_cnt <= 0;
           RW_Done <= 1'b1;
           state <= IDLE;
         end
       end
    endcase
  end

//考虑部分国产编译器暂不支持task，去掉该种写法
//  task read_byte;
//    input [5:0]Ctrl_Cmd;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Go <= 1'b1;
//    end
//  endtask
//
//  task write_byte;
//    input [5:0]Ctrl_Cmd;
//    input [7:0]Wr_Byte_Data;
//    begin
//      Cmd <= Ctrl_Cmd;
//      Tx_DATA <= Wr_Byte_Data;
//      Go <= 1'b1;
//    end
//  endtask

endmodule

/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2019/05/01 00:00:00
// Module Name   : i2c_bit_shift
// Description   : I2C协议基本数据数据字节传输模块
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module i2c_bit_shift #(
  //系统时钟采用50MHz
  parameter SYS_CLOCK = 50_000_000,
  //SCL总线时钟采用400kHz
  parameter SCL_CLOCK = 400_000
)
(
  input           Clk,
  input           Rst_n,

  input     [5:0] Cmd,
  input           Go,
  output reg[7:0] Rx_DATA,
  input     [7:0] Tx_DATA,
  output reg      Trans_Done,
  output reg      ack_o,
  output reg      i2c_sclk,
  inout           i2c_sdat
);

  //产生时钟SCL计数器最大值
  localparam SCL_CNT_M = SYS_CLOCK/SCL_CLOCK/4 - 1;

  localparam
    WR   = 6'b000001,   //写请求
    STA  = 6'b000010,   //起始位请求
    RD   = 6'b000100,   //读请求
    STO  = 6'b001000,   //停止位请求
    ACK  = 6'b010000,   //应答位请求
    NACK = 6'b100000;   //无应答请求

  reg i2c_sdat_oe;
  reg i2c_sdat_o;

  reg [19:0]div_cnt;
  reg en_div_cnt;
  always@(posedge Clk or negedge Rst_n)
  if(!Rst_n)
    div_cnt <= 20'd0;
  else if(en_div_cnt)begin
    if(div_cnt < SCL_CNT_M)
      div_cnt <= div_cnt + 1'b1;
    else
      div_cnt <= 0;
  end
  else
    div_cnt <= 0;

  wire sclk_plus = div_cnt == SCL_CNT_M;

  assign i2c_sdat = (!i2c_sdat_o) && i2c_sdat_oe ? 1'b0:1'bz;

  reg [7:0]state;

  localparam
    IDLE      = 8'b00000001,   //空闲状态
    GEN_STA   = 8'b00000010,   //产生起始信号
    WR_DATA   = 8'b00000100,   //写数据状态
    RD_DATA   = 8'b00001000,   //读数据状态
    CHECK_ACK = 8'b00010000,   //检测应答状态
    GEN_ACK   = 8'b00100000,   //产生应答状态
    GEN_STO   = 8'b01000000;   //产生停止信号

  reg [4:0]cnt;

  always@(posedge Clk or negedge Rst_n)
  if(!Rst_n)begin
    Rx_DATA <= 0;
    i2c_sdat_oe <= 1'd0;
    en_div_cnt <= 1'b0;
    i2c_sdat_o <= 1'd1;
    i2c_sclk <= 1'd1;
    Trans_Done <= 1'b0;
    ack_o <= 0;
    state <= IDLE;
    cnt <= 0;
  end
  else begin
    case(state)
      IDLE:
      begin
        Trans_Done <= 1'b0;
        i2c_sdat_oe <= 1'd1;
        if(Go)begin
          en_div_cnt <= 1'b1;
          if(Cmd & STA)
            state <= GEN_STA;
          else if(Cmd & WR)
            state <= WR_DATA;
          else if(Cmd & RD)
            state <= RD_DATA;
          else
            state <= IDLE;
        end
        else begin
          en_div_cnt <= 1'b0;
          state <= IDLE;
        end
      end

      GEN_STA:
        begin
          if(sclk_plus)begin
            if(cnt == 3)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0:begin i2c_sdat_o <= 1; i2c_sdat_oe <= 1'd1;end
              1:begin i2c_sclk <= 1;end
              2:begin i2c_sdat_o <= 0; i2c_sclk <= 1;end
              3:begin i2c_sclk <= 0;end
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 3)begin
              if(Cmd & WR)
                state <= WR_DATA;
              else if(Cmd & RD)
                state <= RD_DATA;
            end
          end
        end

      WR_DATA:
        begin
          if(sclk_plus)begin
            if(cnt == 31)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0,4,8,12,16,20,24,28:begin i2c_sdat_o <= Tx_DATA[7-cnt[4:2]]; i2c_sdat_oe <= 1'd1;end  //set data;
              1,5,9,13,17,21,25,29:begin i2c_sclk <= 1;end  //sclk posedge
              2,6,10,14,18,22,26,30:begin i2c_sclk <= 1;end  //sclk keep high
              3,7,11,15,19,23,27,31:begin i2c_sclk <= 0;end  //sclk negedge
/*
              0 :begin i2c_sdat_o <= Tx_DATA[7];end
              1 :begin i2c_sclk <= 1;end  //sclk posedge
              2 :begin i2c_sclk <= 1;end  //sclk keep high
              3 :begin i2c_sclk <= 0;end  //sclk negedge

              4 :begin i2c_sdat_o <= Tx_DATA[6];end
              5 :begin i2c_sclk <= 1;end  //sclk posedge
              6 :begin i2c_sclk <= 1;end  //sclk keep high
              7 :begin i2c_sclk <= 0;end  //sclk negedge

              8 :begin i2c_sdat_o <= Tx_DATA[5];end
              9 :begin i2c_sclk <= 1;end  //sclk posedge
              10:begin i2c_sclk <= 1;end  //sclk keep high
              11:begin i2c_sclk <= 0;end  //sclk negedge

              12:begin i2c_sdat_o <= Tx_DATA[4];end
              13:begin i2c_sclk <= 1;end  //sclk posedge
              14:begin i2c_sclk <= 1;end  //sclk keep high
              15:begin i2c_sclk <= 0;end  //sclk negedge

              16:begin i2c_sdat_o <= Tx_DATA[3];end
              17:begin i2c_sclk <= 1;end  //sclk posedge
              18:begin i2c_sclk <= 1;end  //sclk keep high
              19:begin i2c_sclk <= 0;end  //sclk negedge

              20:begin i2c_sdat_o <= Tx_DATA[2];end
              21:begin i2c_sclk <= 1;end  //sclk posedge
              22:begin i2c_sclk <= 1;end  //sclk keep high
              23:begin i2c_sclk <= 0;end  //sclk negedge

              24:begin i2c_sdat_o <= Tx_DATA[1];end
              25:begin i2c_sclk <= 1;end  //sclk posedge
              26:begin i2c_sclk <= 1;end  //sclk keep high
              27:begin i2c_sclk <= 0;end  //sclk negedge

              28:begin i2c_sdat_o <= Tx_DATA[0];end
              29:begin i2c_sclk <= 1;end  //sclk posedge
              30:begin i2c_sclk <= 1;end  //sclk keep high
              31:begin i2c_sclk <= 0;end  //sclk negedge
*/
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 31)begin
              state <= CHECK_ACK;
            end
          end
        end

      RD_DATA:
        begin
          if(sclk_plus)begin
            if(cnt == 31)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0,4,8,12,16,20,24,28:begin i2c_sdat_oe <= 1'd0; i2c_sclk <= 0;end  //set data;
              1,5,9,13,17,21,25,29:begin i2c_sclk <= 1;end  //sclk posedge
              2,6,10,14,18,22,26,30:begin i2c_sclk <= 1; Rx_DATA <= {Rx_DATA[6:0],i2c_sdat};end  //sclk keep high
              3,7,11,15,19,23,27,31:begin i2c_sclk <= 0;end  //sclk negedge
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 31)begin
              state <= GEN_ACK;
            end
          end
        end

      CHECK_ACK:
        begin
          if(sclk_plus)begin
            if(cnt == 3)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0:begin i2c_sdat_oe <= 1'd0; i2c_sclk <= 0;end
              1:begin i2c_sclk <= 1;end
              2:begin ack_o <= i2c_sdat; i2c_sclk <= 1;end
              3:begin i2c_sclk <= 0;end
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 3)begin
              if(Cmd & STO)
                state <= GEN_STO;
              else begin
                state <= IDLE;
                Trans_Done <= 1'b1;
              end
            end
          end
        end

      GEN_ACK:
        begin
          if(sclk_plus)begin
            if(cnt == 3)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0:begin
                  i2c_sdat_oe <= 1'd1;
                  i2c_sclk <= 0;
                  if(Cmd & ACK)
                    i2c_sdat_o <= 1'b0;
                  else if(Cmd & NACK)
                    i2c_sdat_o <= 1'b1;
                end
              1:begin i2c_sclk <= 1;end
              2:begin i2c_sclk <= 1;end
              3:begin i2c_sclk <= 0;end
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 3)begin
              if(Cmd & STO)
                state <= GEN_STO;
              else begin
                state <= IDLE;
                Trans_Done <= 1'b1;
              end
            end
          end
        end

      GEN_STO:
        begin
          if(sclk_plus)begin
            if(cnt == 3)
              cnt <= 0;
            else
              cnt <= cnt + 1'b1;
            case(cnt)
              0:begin i2c_sdat_o <= 0; i2c_sdat_oe <= 1'd1;end
              1:begin i2c_sclk <= 1;end
              2:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
              3:begin i2c_sclk <= 1;end
              default:begin i2c_sdat_o <= 1; i2c_sclk <= 1;end
            endcase
            if(cnt == 3)begin
              Trans_Done <= 1'b1;
              state <= IDLE;
            end
          end
        end
      default:state <= IDLE;
    endcase
  end

endmodule
