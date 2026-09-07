#include "fpga_gateway/protocol.hpp"

#include <limits>

namespace fpga_gateway {
namespace {

constexpr std::size_t kControlPayloadSize = 9U;
constexpr std::size_t kTelemetryPayloadSize = 50U;
constexpr std::size_t kStatusPayloadSize = 16U;

void set_error(std::string* error, const char* message) {
  if (error != nullptr) {
    *error = message;
  }
}

void append_u8(std::vector<std::uint8_t>* bytes, std::uint8_t value) {
  bytes->push_back(value);
}

void append_u16(std::vector<std::uint8_t>* bytes, std::uint16_t value) {
  bytes->push_back(static_cast<std::uint8_t>(value >> 8));
  bytes->push_back(static_cast<std::uint8_t>(value));
}

void append_u32(std::vector<std::uint8_t>* bytes, std::uint32_t value) {
  bytes->push_back(static_cast<std::uint8_t>(value >> 24));
  bytes->push_back(static_cast<std::uint8_t>(value >> 16));
  bytes->push_back(static_cast<std::uint8_t>(value >> 8));
  bytes->push_back(static_cast<std::uint8_t>(value));
}

void append_u64(std::vector<std::uint8_t>* bytes, std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    bytes->push_back(static_cast<std::uint8_t>(value >> shift));
  }
}

void append_i32(std::vector<std::uint8_t>* bytes, std::int32_t value) {
  // C++ 中先转为无符号数保留二补码位型，再复用网络字节序写入。
  append_u32(bytes, static_cast<std::uint32_t>(value));
}

std::uint16_t read_u16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(bytes[offset]) << 8) | bytes[offset + 1]);
}

std::uint32_t read_u32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return (static_cast<std::uint32_t>(bytes[offset]) << 24) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 16) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 8) |
         static_cast<std::uint32_t>(bytes[offset + 3]);
}

std::uint64_t read_u64(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    value = (value << 8) | bytes[offset + i];
  }
  return value;
}

std::int32_t read_i32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  const std::uint32_t value = read_u32(bytes, offset);
  if (value <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return static_cast<std::int32_t>(value);
  }
  if (value == 0x80000000U) {
    return std::numeric_limits<std::int32_t>::min();
  }
  // 避免将超出 int32 正数范围的无符号值直接转换；按二补码手工还原负数。
  return -static_cast<std::int32_t>((~value) + 1U);
}

bool is_known_type(std::uint8_t type) {
  return type == static_cast<std::uint8_t>(MessageType::Control) ||
         type == static_cast<std::uint8_t>(MessageType::Telemetry) ||
         type == static_cast<std::uint8_t>(MessageType::Status);
}

std::vector<std::uint8_t> encode_packet(MessageType type, std::uint32_t sequence,
                                        std::uint64_t timestamp_us,
                                        const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> packet;
  packet.reserve(kHeaderSize + payload.size() + kCrcSize);
  append_u32(&packet, kProtocolMagic);
  append_u8(&packet, kProtocolVersion);
  append_u8(&packet, static_cast<std::uint8_t>(type));
  append_u16(&packet, static_cast<std::uint16_t>(payload.size()));
  append_u32(&packet, sequence);
  append_u64(&packet, timestamp_us);
  packet.insert(packet.end(), payload.begin(), payload.end());
  // CRC 覆盖头和载荷，不覆盖末尾正在写入的 CRC 自身。
  append_u16(&packet, crc16_ccitt(packet.data(), packet.size()));
  return packet;
}

bool decode_payload_header(const std::vector<std::uint8_t>& packet,
                           MessageType expected_type, std::size_t expected_size,
                           PacketHeader* decoded, std::string* error) {
  PacketHeader local_header;
  if (!decode_header(packet, &local_header, error)) {
    return false;
  }
  if (local_header.message_type != static_cast<std::uint8_t>(expected_type)) {
    set_error(error, "unexpected message type");
    return false;
  }
  if (local_header.payload_length != expected_size) {
    set_error(error, "unexpected payload length");
    return false;
  }
  if (decoded != nullptr) {
    *decoded = local_header;
  }
  return true;
}

}  // namespace

std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t length) {
  // CRC16-CCITT-FALSE：初值 0xFFFF，多项式 0x1021，无额外反射或异或输出。
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> encode_control(std::uint32_t sequence,
                                         std::uint64_t timestamp_us,
                                         const ControlPayload& payload) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(kControlPayloadSize);
  append_i32(&bytes, payload.linear_mm_s);
  append_i32(&bytes, payload.angular_mrad_s);
  append_u8(&bytes, payload.flags);
  return encode_packet(MessageType::Control, sequence, timestamp_us, bytes);
}

std::vector<std::uint8_t> encode_telemetry(std::uint32_t sequence,
                                           std::uint64_t timestamp_us,
                                           const TelemetryPayload& payload) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(kTelemetryPayloadSize);
  append_i32(&bytes, payload.left_ticks);
  append_i32(&bytes, payload.right_ticks);
  append_i32(&bytes, payload.left_velocity_mm_s);
  append_i32(&bytes, payload.right_velocity_mm_s);
  append_i32(&bytes, payload.accel_x_mm_s2);
  append_i32(&bytes, payload.accel_y_mm_s2);
  append_i32(&bytes, payload.accel_z_mm_s2);
  append_i32(&bytes, payload.gyro_x_mrad_s);
  append_i32(&bytes, payload.gyro_y_mrad_s);
  append_i32(&bytes, payload.gyro_z_mrad_s);
  append_u16(&bytes, payload.range_front_mm);
  append_u16(&bytes, payload.range_left_mm);
  append_u16(&bytes, payload.range_right_mm);
  append_u32(&bytes, payload.status_bits);
  return encode_packet(MessageType::Telemetry, sequence, timestamp_us, bytes);
}

std::vector<std::uint8_t> encode_status(std::uint32_t sequence,
                                        std::uint64_t timestamp_us,
                                        const StatusPayload& payload) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(kStatusPayloadSize);
  append_u8(&bytes, payload.mode);
  append_u8(&bytes, payload.motion_enabled);
  append_u8(&bytes, payload.estop);
  append_u8(&bytes, payload.safety_stop);
  append_u32(&bytes, payload.fault_bits);
  append_u32(&bytes, payload.uptime_ms);
  append_u32(&bytes, payload.last_control_sequence);
  return encode_packet(MessageType::Status, sequence, timestamp_us, bytes);
}

bool decode_header(const std::vector<std::uint8_t>& packet, PacketHeader* header,
                   std::string* error) {
  if (header == nullptr) {
    set_error(error, "header output is null");
    return false;
  }
  if (packet.size() < kHeaderSize + kCrcSize) {
    set_error(error, "packet too short");
    return false;
  }

  PacketHeader local_header;
  local_header.magic = read_u32(packet, 0);
  local_header.version = packet[4];
  local_header.message_type = packet[5];
  local_header.payload_length = read_u16(packet, 6);
  local_header.sequence = read_u32(packet, 8);
  local_header.timestamp_us = read_u64(packet, 12);

  if (local_header.magic != kProtocolMagic) {
    set_error(error, "invalid magic");
    return false;
  }
  if (local_header.version != kProtocolVersion) {
    set_error(error, "unsupported version");
    return false;
  }
  if (!is_known_type(local_header.message_type)) {
    set_error(error, "unknown message type");
    return false;
  }
  // UDP 保留消息边界，但仍要求实际长度恰好匹配头中的载荷长度，拒绝截断和尾随字节。
  if (packet.size() != kHeaderSize + local_header.payload_length + kCrcSize) {
    set_error(error, "payload length mismatch");
    return false;
  }
  const std::uint16_t received_crc = read_u16(packet, packet.size() - kCrcSize);
  const std::uint16_t computed_crc = crc16_ccitt(packet.data(), packet.size() - kCrcSize);
  if (received_crc != computed_crc) {
    set_error(error, "crc mismatch");
    return false;
  }
  *header = local_header;
  return true;
}

bool decode_control(const std::vector<std::uint8_t>& packet, ControlPayload* payload,
                    PacketHeader* header, std::string* error) {
  if (payload == nullptr) {
    set_error(error, "control output is null");
    return false;
  }
  PacketHeader local_header;
  if (!decode_payload_header(packet, MessageType::Control, kControlPayloadSize,
                             &local_header, error)) {
    return false;
  }
  const std::size_t offset = kHeaderSize;
  payload->linear_mm_s = read_i32(packet, offset);
  payload->angular_mrad_s = read_i32(packet, offset + 4);
  payload->flags = packet[offset + 8];
  if (header != nullptr) {
    *header = local_header;
  }
  return true;
}

bool decode_telemetry(const std::vector<std::uint8_t>& packet,
                      TelemetryPayload* payload, PacketHeader* header,
                      std::string* error) {
  if (payload == nullptr) {
    set_error(error, "telemetry output is null");
    return false;
  }
  PacketHeader local_header;
  if (!decode_payload_header(packet, MessageType::Telemetry, kTelemetryPayloadSize,
                             &local_header, error)) {
    return false;
  }
  const std::size_t offset = kHeaderSize;
  payload->left_ticks = read_i32(packet, offset);
  payload->right_ticks = read_i32(packet, offset + 4);
  payload->left_velocity_mm_s = read_i32(packet, offset + 8);
  payload->right_velocity_mm_s = read_i32(packet, offset + 12);
  payload->accel_x_mm_s2 = read_i32(packet, offset + 16);
  payload->accel_y_mm_s2 = read_i32(packet, offset + 20);
  payload->accel_z_mm_s2 = read_i32(packet, offset + 24);
  payload->gyro_x_mrad_s = read_i32(packet, offset + 28);
  payload->gyro_y_mrad_s = read_i32(packet, offset + 32);
  payload->gyro_z_mrad_s = read_i32(packet, offset + 36);
  payload->range_front_mm = read_u16(packet, offset + 40);
  payload->range_left_mm = read_u16(packet, offset + 42);
  payload->range_right_mm = read_u16(packet, offset + 44);
  payload->status_bits = read_u32(packet, offset + 46);
  if (header != nullptr) {
    *header = local_header;
  }
  return true;
}

bool decode_status(const std::vector<std::uint8_t>& packet, StatusPayload* payload,
                   PacketHeader* header, std::string* error) {
  if (payload == nullptr) {
    set_error(error, "status output is null");
    return false;
  }
  PacketHeader local_header;
  if (!decode_payload_header(packet, MessageType::Status, kStatusPayloadSize,
                             &local_header, error)) {
    return false;
  }
  const std::size_t offset = kHeaderSize;
  payload->mode = packet[offset];
  payload->motion_enabled = packet[offset + 1];
  payload->estop = packet[offset + 2];
  payload->safety_stop = packet[offset + 3];
  payload->fault_bits = read_u32(packet, offset + 4);
  payload->uptime_ms = read_u32(packet, offset + 8);
  payload->last_control_sequence = read_u32(packet, offset + 12);
  if (header != nullptr) {
    *header = local_header;
  }
  return true;
}

bool is_newer_sequence(std::uint32_t candidate, std::uint32_t previous) {
  // 无符号减法自然按模 2^32 回绕；半区间排除相等和相差恰为 2^31 的歧义。
  const std::uint32_t difference = candidate - previous;
  return difference != 0U && difference < 0x80000000U;
}

}  // namespace fpga_gateway
