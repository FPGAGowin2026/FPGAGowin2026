# 25K RTL inventory

Canonical Gowin profiles are defined only by `choice3_robot_25k.gprj` and
`choice3_robot_25k_demo.gprj`. Do not add every file under `src` blindly.

- Test-only RTL, compiled by `Scripts/run_sim.ps1` but intentionally absent from
  both Gowin profiles: `bringup_uart_rx.v`, `bringup_uart_tx.v`,
  `choice3_uart_console.v`, and `choice3_imu_sample_guard.v`.
- Optional expansion/reference drivers, not instantiated by either canonical
  top: `choice3_attitude_tilt_proxy.v`, `sensors/hc_sr04_controller.v`,
  `sensors/icm42688p_spi.v`, and `sensors/vl53l0x_i2c.v`.
- Release/Demo production RTL is the explicit file list in each GPRJ. The
  current integrated sensor path is OV5640 + JY61P + TF-Luna + four digital IR
  channels + four quadrature encoders.

Optional drivers require their own protocol, freshness, electrical, pin and
board-timing integration before use. Their presence in `src` is not evidence
that they are active in the production top.
