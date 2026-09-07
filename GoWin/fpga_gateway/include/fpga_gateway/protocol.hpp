#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fpga_gateway {

// 固定报文依次为 20 字节头、类型固定的载荷和 2 字节 CRC；所有多字节整数均按网络字节序传输。
constexpr std::uint32_t kProtocolMagic = 0x46504741U;
constexpr std::uint8_t kProtocolVersion = 1U;
constexpr std::size_t kHeaderSize = 20U;
constexpr std::size_t kCrcSize = 2U;
constexpr std::uint8_t kControlEstop = 1U << 0;

enum class MessageType : std::uint8_t {
  Control = 1,
  Telemetry = 2,
  Status = 3,
};

struct PacketHeader {
  std::uint32_t magic = kProtocolMagic;
  std::uint8_t version = kProtocolVersion;
  std::uint8_t message_type = 0;
  std::uint16_t payload_length = 0;
  std::uint32_t sequence = 0;
  std::uint64_t timestamp_us = 0;
};

struct ControlPayload {
  // 速度量采用有符号二补码 int32，单位固定为 mm/s 和 mrad/s，避免浮点跨端差异。
  std::int32_t linear_mm_s = 0;
  std::int32_t angular_mrad_s = 0;
  std::uint8_t flags = 0;  // 第 0 位：急停
};

struct TelemetryPayload {
  // 字段顺序就是线上载荷顺序；编码器、速度、IMU、测距和状态位不带可选字段。
  std::int32_t left_ticks = 0;
  std::int32_t right_ticks = 0;
  std::int32_t left_velocity_mm_s = 0;
  std::int32_t right_velocity_mm_s = 0;
  std::int32_t accel_x_mm_s2 = 0;
  std::int32_t accel_y_mm_s2 = 0;
  std::int32_t accel_z_mm_s2 = 0;
  std::int32_t gyro_x_mrad_s = 0;
  std::int32_t gyro_y_mrad_s = 0;
  std::int32_t gyro_z_mrad_s = 0;
  std::uint16_t range_front_mm = 0;
  std::uint16_t range_left_mm = 0;
  std::uint16_t range_right_mm = 0;
  std::uint32_t status_bits = 0;
};

struct StatusPayload {
  std::uint8_t mode = 0;  // 0：独立模式，1：Jetson 模式
  std::uint8_t motion_enabled = 0;
  std::uint8_t estop = 0;
  std::uint8_t safety_stop = 0;
  std::uint32_t fault_bits = 0;
  std::uint32_t uptime_ms = 0;
  std::uint32_t last_control_sequence = 0;
};

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t length);

std::vector<std::uint8_t> encode_control(
    std::uint32_t sequence, std::uint64_t timestamp_us,
    const ControlPayload& payload);
std::vector<std::uint8_t> encode_telemetry(
    std::uint32_t sequence, std::uint64_t timestamp_us,
    const TelemetryPayload& payload);
std::vector<std::uint8_t> encode_status(
    std::uint32_t sequence, std::uint64_t timestamp_us,
    const StatusPayload& payload);

bool decode_header(const std::vector<std::uint8_t>& packet,
                   PacketHeader* header, std::string* error = nullptr);
bool decode_control(const std::vector<std::uint8_t>& packet,
                    ControlPayload* payload, PacketHeader* header = nullptr,
                    std::string* error = nullptr);
bool decode_telemetry(const std::vector<std::uint8_t>& packet,
                      TelemetryPayload* payload, PacketHeader* header = nullptr,
                      std::string* error = nullptr);
bool decode_status(const std::vector<std::uint8_t>& packet,
                   StatusPayload* payload, PacketHeader* header = nullptr,
                   std::string* error = nullptr);

// 以模 2^32 的半区间判断先后：差值落在 (0, 2^31) 才视为新包，支持正常回绕。
bool is_newer_sequence(std::uint32_t candidate, std::uint32_t previous);

}  // namespace fpga_gateway
