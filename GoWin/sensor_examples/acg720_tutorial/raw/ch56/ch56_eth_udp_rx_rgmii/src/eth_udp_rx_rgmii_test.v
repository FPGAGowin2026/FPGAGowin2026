module eth_udp_rx_rgmii_test(
    input Clk,
    input reset_n,
    
    //以太网接收
    input rgmii_rx_clk,
    input [3:0]rgmii_rxd,
    input rgmii_rxdv,
    output eth_reset_n, //以太网复位信号
    output uart_tx
    );
   wire rgmii_rx_clk_i; 
   wire gmii_rx_clk; //以太网接收时钟
   wire [7:0]gmii_rxd; //以太网接收数据
   wire gmii_rxdv;  //以太网接收数据有效标志
   wire reset_p;
   wire clk125m;
   wire [47:0] exter_mac;
   wire [31:0] exter_ip;
   wire [15:0] exter_port;
   wire [15:0] rx_data_length;
   wire        data_overflow;
   wire        rx_payload_valid;
   wire [7:0]  rx_payload_dat;
   wire        rx_pkt_done;
   wire        rx_pkt_err;
  
   wire fifo_rd_req;
   wire rx_empty;
   wire [7:0] dout;
   wire uart_send_en;
   wire uart_tx_done;
   wire [7:0] uart_tx_data;
   
   parameter LOCAL_MAC = 48'h00_0a_35_01_fe_c0;
   parameter LOCAL_IP = 32'hc0_a8_00_02;
   parameter LOCAL_PORT = 16'd5000;
   
   assign reset_p = ~reset_n;
   assign eth_reset_n = 1;
   
   
    Gowin_PLL Gowin_PLL(
        .clkout0(rgmii_rx_clk_i), //output clkout0
        .clkin(rgmii_rx_clk) //input clkin
    );

  rgmii_to_gmii rgmii_to_gmii(
    .reset(reset_p),
    
    .rgmii_rx_clk(rgmii_rx_clk_i),
    .rgmii_rxd(rgmii_rxd),
    .rgmii_rxdv(rgmii_rxdv),
    
    .gmii_rx_clk(gmii_rx_clk),
    .gmii_rxdv(gmii_rxdv),
    .gmii_rxd(gmii_rxd),
    .gmii_rxer()
  );

   
  //以太网接收
  eth_udp_rx_gmii eth_udp_rx_gmii(
    .reset_p         (reset_p               ),

    .local_mac       (LOCAL_MAC             ),
    .local_ip        (LOCAL_IP              ),
    .local_port      (LOCAL_PORT            ),

    .clk125m_o       (clk125m               ),
    .exter_mac       (exter_mac             ),
    .exter_ip        (exter_ip              ),
    .exter_port      (exter_port            ),
    .rx_data_length  (rx_data_length        ),
    .data_overflow_i (data_overflow         ),
    .payload_valid_o (rx_payload_valid      ),
    .payload_dat_o   (rx_payload_dat        ),

    .one_pkt_done    (rx_pkt_done           ),
    .pkt_error       (rx_pkt_err            ),
    .debug_crc_check (                      ),

    .gmii_rx_clk     (gmii_rx_clk           ),
    .gmii_rxdv       (gmii_rxdv             ),
    .gmii_rxd        (gmii_rxd              )
  );


	fifo_top fifo_top(
		.Data(rx_payload_dat), //input [7:0] Data
		.WrClk(clk125m), //input WrClk
		.RdClk(Clk), //input RdClk
		.WrEn(rx_payload_valid), //input WrEn
		.RdEn(fifo_rd_req), //input RdEn
		.Q(dout), //output [7:0] Q
		.Empty(rx_empty), //output Empty
		.Full() //output Full
	);


    uart_send_ctrl uart_send_ctrl(
        .clk(Clk),
        .reset_p(reset_p),
        .fifo_rd_data(dout),
        .fifo_empty(rx_empty),
        .fifo_rd_req(fifo_rd_req),
        
        .uart_tx_done(uart_tx_done),
        .uart_send_en(uart_send_en),
        .uart_tx_data(uart_tx_data)
    );

    uart_byte_tx uart_byte_tx(
        .clk(Clk),
        .reset_n(reset_n),
        
        .data_byte(uart_tx_data),
        .send_en(uart_send_en),
        .baud_set(3'd0),
        
        .uart_tx(uart_tx),
        .tx_done(uart_tx_done),
        .uart_state()
    );
endmodule
