#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RTL_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
SIM_TMP_DIR=$(mktemp -d /tmp/fpga_gowin_sim.XXXXXX)
trap 'rm -rf -- "${SIM_TMP_DIR}"' EXIT

IVERILOG_ARGS=()
if [[ -n "${IVERILOG_BIN:-}" ]]; then
  IVERILOG=${IVERILOG_BIN}
elif command -v iverilog >/dev/null 2>&1; then
  IVERILOG=$(command -v iverilog)
elif [[ -x /tmp/fpga_gowin_iverilog/usr/bin/iverilog ]]; then
  IVERILOG=/tmp/fpga_gowin_iverilog/usr/bin/iverilog
  IVERILOG_ARGS=(-B /tmp/fpga_gowin_iverilog/usr/lib/x86_64-linux-gnu/ivl)
else
  echo "未找到 iverilog。可系统安装，或把 Ubuntu 包解压到 /tmp/fpga_gowin_iverilog。" >&2
  echo "也可通过 IVERILOG_BIN、VVP_BIN 和 IVERILOG_BASE 指定本地工具链。" >&2
  exit 2
fi

if [[ -n "${IVERILOG_BASE:-}" ]]; then
  IVERILOG_ARGS=(-B "${IVERILOG_BASE}")
fi

if [[ -n "${VVP_BIN:-}" ]]; then
  VVP=${VVP_BIN}
elif command -v vvp >/dev/null 2>&1; then
  VVP=$(command -v vvp)
elif [[ -x /tmp/fpga_gowin_iverilog/usr/bin/vvp ]]; then
  VVP=/tmp/fpga_gowin_iverilog/usr/bin/vvp
else
  echo "未找到 vvp；请通过 VVP_BIN 指定。" >&2
  exit 2
fi

compile_and_run() {
  local top=$1
  shift
  local output="${SIM_TMP_DIR}/${top}.vvp"
  echo "[RUN] ${top}"
  "${IVERILOG}" "${IVERILOG_ARGS[@]}" -g2012 -Wall -s "${top}" -o "${output}" "$@"
  "${VVP}" "${output}"
}

cd "${RTL_ROOT}"

compile_and_run tb_motion_mode_arbiter \
  src/motion_mode_arbiter.v sim/tb_motion_mode_arbiter.sv

compile_and_run tb_fpga_control_packet_rx \
  src/protocol/fpga_control_packet_rx.v sim/tb_fpga_control_packet_rx.sv

compile_and_run tb_fpga_telemetry_packet_tx \
  src/protocol/fpga_telemetry_packet_tx.v sim/tb_fpga_telemetry_packet_tx.sv

compile_and_run tb_sensor_frontends \
  src/sensors/ov5640_dvp_capture.v \
  src/sensors/imu_sample_commit.v \
  src/sensors/range_validity_monitor.v \
  sim/tb_sensor_frontends.sv

compile_and_run tb_autonomous_motion \
  src/control/range_safety_limiter.v \
  src/control/local_avoidance_fsm.v \
  src/control/path_segment_executor.v \
  src/control/autonomous_motion_controller.v \
  sim/tb_autonomous_motion.sv

compile_and_run tb_motor_control \
  src/motor/quadrature_decoder.v \
  src/motor/encoder_velocity_estimator.v \
  src/motor/wheel_speed_controller.v \
  src/motor/differential_drive_control.v \
  sim/tb_motor_control.sv

compile_and_run tb_robot_rtl_core \
  src/protocol/fpga_control_packet_rx.v \
  src/sensors/ov5640_dvp_capture.v \
  src/sensors/imu_sample_commit.v \
  src/sensors/range_validity_monitor.v \
  src/control/range_safety_limiter.v \
  src/control/local_avoidance_fsm.v \
  src/control/path_segment_executor.v \
  src/control/autonomous_motion_controller.v \
  src/motion_mode_arbiter.v \
  src/motor/wheel_speed_controller.v \
  src/motor/differential_drive_control.v \
  src/system/control_command_watchdog.v \
  src/system/demo_path_rom.v \
  src/system/robot_rtl_core.v \
  sim/tb_robot_rtl_core.sv

compile_and_run tb_async_frame_fifo \
  src/cdc/async_frame_fifo.v sim/tb_async_frame_fifo.sv

GOWIN_PRIM=${GOWIN_PRIM:-"${RTL_ROOT}/../GoWin/Gowin_V1.9.12.03/IDE/simlib/gw5a/prim_sim.v"}
compile_and_run tb_eth_udp_loopback_rgmii \
  "${GOWIN_PRIM}" \
  src/ethernet/eth_msg_buf.v \
  src/ethernet/crc32_d8.v \
  src/ethernet/ip_checksum.v \
  src/ethernet/eth_udp_rx_gmii.v \
  src/ethernet/eth_udp_tx_gmii.v \
  src/ethernet/rgmii_to_gmii.v \
  src/ethernet/gmii_to_rgmii.v \
  src/ethernet/eth_udp_loopback_rgmii.v \
  sim/tb_eth_udp_loopback_rgmii.sv

compile_and_run tb_robot_network_e2e \
  "${GOWIN_PRIM}" \
  src/cdc/async_frame_fifo.v \
  src/ethernet/crc32_d8.v \
  src/ethernet/ip_checksum.v \
  src/ethernet/eth_udp_rx_gmii.v \
  src/ethernet/eth_udp_tx_gmii.v \
  src/ethernet/eth_arp_icmp_rx_gmii.v \
  src/ethernet/eth_arp_icmp_tx_gmii.v \
  src/ethernet/rgmii_to_gmii.v \
  src/ethernet/gmii_to_rgmii.v \
  src/protocol/fpga_control_packet_rx.v \
  src/protocol/fpga_telemetry_packet_tx.v \
  src/protocol/fpga_status_packet_tx.v \
  src/sensors/ov5640_dvp_capture.v \
  src/sensors/imu_sample_commit.v \
  src/sensors/range_validity_monitor.v \
  src/control/range_safety_limiter.v \
  src/control/local_avoidance_fsm.v \
  src/control/path_segment_executor.v \
  src/control/autonomous_motion_controller.v \
  src/motion_mode_arbiter.v \
  src/motor/wheel_speed_controller.v \
  src/motor/differential_drive_control.v \
  src/system/control_command_watchdog.v \
  src/system/demo_path_rom.v \
  src/system/robot_rtl_core.v \
  src/ethernet/acg720_robot_network_rgmii.v \
  sim/tb_robot_network_e2e.sv

compile_and_run tb_acg720_udp_loopback_project \
  "${GOWIN_PRIM}" \
  src/ethernet/eth_msg_buf.v \
  src/ethernet/crc32_d8.v \
  src/ethernet/ip_checksum.v \
  src/ethernet/eth_udp_rx_gmii.v \
  src/ethernet/eth_udp_tx_gmii.v \
  src/ethernet/rgmii_to_gmii.v \
  src/ethernet/gmii_to_rgmii.v \
  src/vendor/gowin/gowin_pll.v \
  src/vendor/gowin/gowin_pll_mod.v \
  src/vendor/gowin/pll_init.v \
  src/ethernet/eth_udp_loopback_rgmii.v \
  sim/tb_acg720_udp_loopback_project.sv

echo "[PASS] 10 个 RTL 功能自检与 1 个板级展开检查全部通过"
