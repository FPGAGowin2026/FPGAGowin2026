`timescale 1ns/1ps

// 离线综合/布局布线 wrapper，只保留来源明确的官方 RGMII 与复位管脚。
// 这不是最终上板顶层；未确定的系统时钟、急停、电机和传感器管脚均未猜测。
module acg720_robot_network_offline_top (
    input  wire       reset_n,
    input  wire       rgmii_rx_clk_i,
    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rxdv,
    output wire       eth_rst_n,
    output wire       rgmii_tx_clk,
    output wire [3:0] rgmii_txd,
    output wire       rgmii_txen
);
    reg offline_system_clk_q;

    // 派生 62.5 MHz 离线系统时钟；该分频只用于无真实系统时钟管脚时的工具实现。
    always @(posedge rgmii_rx_clk_i or negedge reset_n) begin
        if (!reset_n)
            offline_system_clk_q <= 1'b0;
        else
            offline_system_clk_q <= ~offline_system_clk_q;
    end

    // 离线 wrapper 用已知 RGMII RX 时钟的 /2 分频驱动系统域，仅用于工具完整实现。
    // 所有未接物理输入都选择失效安全值：急停/严重故障有效，测距无效，路径中止。
    acg720_robot_network_rgmii #(
        .SYSTEM_CLOCK_HZ(62500000)
    ) robot_network (
        .reset_n(reset_n),
        .system_clk_i(offline_system_clk_q),
        .rgmii_rx_clk_i(rgmii_rx_clk_i),
        .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv),
        .eth_rst_n(eth_rst_n),
        .rgmii_tx_clk(rgmii_tx_clk),
        .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen),
        .requested_mode_i(1'b0),
        .motion_stopped_i(1'b1),
        .physical_estop_i(1'b1),
        .critical_fault_i(1'b1),
        .front_sample_valid_i(1'b0),
        .front_distance_mm_i(16'd0),
        .left_sample_valid_i(1'b0),
        .left_distance_mm_i(16'd0),
        .right_sample_valid_i(1'b0),
        .right_distance_mm_i(16'd0),
        .imu_sample_start_i(1'b0),
        .imu_byte_valid_i(1'b0),
        .imu_byte_data_i(8'd0),
        .path_start_i(1'b0),
        .path_abort_i(1'b1),
        .avoidance_tick_i(1'b0),
        .segment_progress_i(32'd0),
        .motor_sample_tick_i(1'b0),
        .left_ticks_i(32'sd0),
        .right_ticks_i(32'sd0),
        .left_velocity_mm_s_i(32'sd0),
        .right_velocity_mm_s_i(32'sd0),
        .accel_x_mm_s2_i(32'sd0),
        .accel_y_mm_s2_i(32'sd0),
        .accel_z_mm_s2_i(32'sd0),
        .gyro_x_mrad_s_i(32'sd0),
        .gyro_y_mrad_s_i(32'sd0),
        .gyro_z_mrad_s_i(32'sd0),
        .control_packet_accepted_o(),
        .control_packet_rejected_o(),
        .motion_enabled_o(),
        .final_linear_mm_s_o(),
        .final_angular_mrad_s_o(),
        .software_estop_o(),
        .control_timeout_pulse_o(),
        .status_bits_o(),
        .fault_bits_o(),
        .rx_control_fifo_full_o(),
        .tx_frame_fifo_full_o()
    );
endmodule
