#include "fpga_gateway/protocol.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

TEST(Protocol, GoldenControlPacket)
{
  // 黄金向量锁定跨语言/FPGA 端最易错的字节布局、网络字节序、负数二补码和 CRC。
  const fpga_gateway::ControlPayload payload{-1000, 2500, 1};
  const auto packet = fpga_gateway::encode_control(
    0x01020304U, 0x0102030405060708ULL, payload);
  const std::vector<std::uint8_t> expected = {
    0x46, 0x50, 0x47, 0x41, 0x01, 0x01, 0x00, 0x09,
    0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0xFF, 0xFF, 0xFC, 0x18,
    0x00, 0x00, 0x09, 0xC4, 0x01, 0x85, 0xAA,
  };
  EXPECT_EQ(packet, expected);

  fpga_gateway::ControlPayload decoded;
  ASSERT_TRUE(fpga_gateway::decode_control(packet, &decoded));
  EXPECT_EQ(decoded.linear_mm_s, payload.linear_mm_s);
  EXPECT_EQ(decoded.angular_mrad_s, payload.angular_mrad_s);
  EXPECT_EQ(decoded.flags, payload.flags);

  const std::vector<std::uint8_t> check = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(fpga_gateway::crc16_ccitt(check.data(), check.size()), 0x29B1U);
}

TEST(Protocol, RoundTripsTelemetryAndStatus)
{
  // 往返测试确认两类固定长度载荷不会在字段偏移或有符号值上发生错位。
  const fpga_gateway::TelemetryPayload telemetry{
    -1, 2, -3, 4, -5, 6, -7, 8, -9, 10, 100, 200, 300, 0xAABBCCDDU};
  const auto telemetry_packet = fpga_gateway::encode_telemetry(7, 8, telemetry);
  fpga_gateway::TelemetryPayload telemetry_decoded;
  fpga_gateway::PacketHeader telemetry_header;
  ASSERT_TRUE(fpga_gateway::decode_telemetry(
    telemetry_packet, &telemetry_decoded, &telemetry_header));
  EXPECT_EQ(telemetry_decoded.left_ticks, telemetry.left_ticks);
  EXPECT_EQ(telemetry_decoded.gyro_z_mrad_s, telemetry.gyro_z_mrad_s);
  EXPECT_EQ(telemetry_decoded.range_right_mm, telemetry.range_right_mm);
  EXPECT_EQ(telemetry_decoded.status_bits, telemetry.status_bits);
  EXPECT_EQ(telemetry_header.sequence, 7U);
  EXPECT_EQ(telemetry_header.timestamp_us, 8U);

  const fpga_gateway::StatusPayload status{1, 1, 0, 1, 0x11223344U, 1234, 6};
  const auto status_packet = fpga_gateway::encode_status(9, 10, status);
  fpga_gateway::StatusPayload status_decoded;
  ASSERT_TRUE(fpga_gateway::decode_status(status_packet, &status_decoded));
  EXPECT_EQ(status_decoded.mode, status.mode);
  EXPECT_EQ(status_decoded.safety_stop, status.safety_stop);
  EXPECT_EQ(status_decoded.fault_bits, status.fault_bits);
  EXPECT_EQ(status_decoded.last_control_sequence, status.last_control_sequence);
}

TEST(Protocol, RejectsMalformedPackets)
{
  // 分别覆盖 CRC、头字段、严格长度和截断包，确保异常 UDP 数据不会进入载荷解码。
  const auto packet = fpga_gateway::encode_control(1, 2, {3, 4, 0});
  fpga_gateway::PacketHeader header;
  std::string error;

  auto bad = packet;
  bad.back() ^= 0x01;
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "crc mismatch");

  bad = packet;
  bad[0] = 0;
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "invalid magic");

  bad = packet;
  bad[4] = 2;
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "unsupported version");

  bad = packet;
  bad[5] = 99;
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "unknown message type");

  bad = packet;
  bad[7] = 8;
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "payload length mismatch");

  bad.assign(packet.begin(), packet.begin() + 10);
  EXPECT_FALSE(fpga_gateway::decode_header(bad, &header, &error));
  EXPECT_EQ(error, "packet too short");
}

TEST(Protocol, HandlesSequenceWraparound)
{
  // 验证序号由最大值回绕至零仍递增，同时拒绝旧包、重复包和反向跨越半区间的包。
  EXPECT_TRUE(fpga_gateway::is_newer_sequence(11, 10));
  EXPECT_FALSE(fpga_gateway::is_newer_sequence(10, 10));
  EXPECT_FALSE(fpga_gateway::is_newer_sequence(10, 11));
  EXPECT_TRUE(fpga_gateway::is_newer_sequence(0, 0xFFFFFFFFU));
  EXPECT_TRUE(fpga_gateway::is_newer_sequence(1, 0xFFFFFFFEU));
  EXPECT_FALSE(fpga_gateway::is_newer_sequence(0xFFFFFFFFU, 0));
}
