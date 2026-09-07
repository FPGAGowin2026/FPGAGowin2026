`timescale 1ns/1ps

// 无外设实板验收 wrapper：只使用板载时钟、复位和 USB-UART。
module choice3_robot_25k_demo_top (
    input wire clk_50m_i,input wire reset_i,
    input wire uart_rx_i,output wire uart_tx_o
);
    wire tmds_clk_p,tmds_clk_n,motor_stby;
    wire [2:0] tmds_data_p,tmds_data_n;
    wire [3:0] motor_pwm,motor_in1,motor_in2;
    wire ov_xclk,ov_reset_n,ov_pwdn;
    tri ov_scl,ov_sda;tri [3:0] reserve;
    choice3_robot_25k_top #(.ONCHIP_DEMO(1)) u_demo_top(
      .clk_50m_i(clk_50m_i),.reset_i(reset_i),.uart_rx_i(uart_rx_i),.uart_tx_o(uart_tx_o),
      .tmds_clk_p_o(tmds_clk_p),.tmds_clk_n_o(tmds_clk_n),
      .tmds_data_p_o(tmds_data_p),.tmds_data_n_o(tmds_data_n),
      .ov_pclk_i(1'b0),.ov_href_i(1'b0),.ov_vsync_i(1'b0),.ov_data_i(8'd0),
      .ov_xclk_o(ov_xclk),.ov_reset_n_o(ov_reset_n),.ov_pwdn_o(ov_pwdn),
      .ov_sccb_scl_io(ov_scl),.ov_sccb_sda_io(ov_sda),
      .imu_uart_rx_i(1'b1),.range_uart_rx_i(1'b1),.ir_sensor_i(4'hf),
      .encoder_a_i(4'd0),.encoder_b_i(4'd0),.estop_n_i(1'b0),
      .motor_pwm_o(motor_pwm),.motor_in1_o(motor_in1),.motor_in2_o(motor_in2),
      .motor_stby_o(motor_stby),.reserve_io(reserve));
endmodule
