#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "fpga_gateway/protocol.hpp"

namespace {

bool valid_port(int port)
{
  return port > 0 && port <= std::numeric_limits<uint16_t>::max();
}

uint64_t now_us()
{
  // 协议时间戳使用可与主机侧日志对齐的系统时钟；运动积分和看门狗则始终使用单调时钟。
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

int udp_socket(const std::string & bind_ip, uint16_t port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("cannot create UDP socket: " + std::string(std::strerror(errno)));
  }
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot set UDP socket nonblocking: " + error);
  }
  // 遥测发送端使用临时端口；只有控制接收端需要绑定固定端口。
  if (port == 0) {
    return fd;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (::inet_pton(AF_INET, bind_ip.c_str(), &address.sin_addr) != 1 ||
    ::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
  {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot bind UDP " + bind_ip + ":" + std::to_string(port) + ": " + error);
  }
  return fd;
}

sockaddr_in endpoint(const std::string & ip, uint16_t port)
{
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
    throw std::runtime_error("invalid IPv4 address: " + ip);
  }
  return address;
}

}  // namespace

class FakeFpgaNode : public rclcpp::Node
{
public:
  FakeFpgaNode()
  : Node("fake_fpga_node"), start_time_(std::chrono::steady_clock::now()), last_telemetry_time_(start_time_)
  {
    const auto listen_ip = declare_parameter<std::string>("listen_ip", "127.0.0.1");
    const auto jetson_ip = declare_parameter<std::string>("jetson_ip", "127.0.0.1");
    const int control_port = declare_parameter<int>("control_port", 15000);
    const int telemetry_port = declare_parameter<int>("telemetry_port", 15001);
    const double telemetry_rate = declare_parameter<double>("telemetry_rate_hz", 100.0);
    const double status_rate = declare_parameter<double>("status_rate_hz", 10.0);
    const int command_watchdog_ms = declare_parameter<int>("command_watchdog_ms", 250);
    watchdog_ = std::chrono::milliseconds(command_watchdog_ms);
    wheel_separation_ = declare_parameter<double>("wheel_separation_m", 0.30);
    wheel_radius_ = declare_parameter<double>("wheel_radius_m", 0.05);
    ticks_per_rev_ = declare_parameter<int>("encoder_ticks_per_rev", 2048);
    drop_every_n_ = declare_parameter<int>("drop_every_n", 0);
    corrupt_every_n_ = declare_parameter<int>("corrupt_every_n", 0);
    if (!valid_port(control_port) || !valid_port(telemetry_port) ||
      telemetry_rate <= 0.0 || status_rate <= 0.0 || command_watchdog_ms <= 0 ||
      wheel_separation_ <= 0.0 ||
      wheel_radius_ <= 0.0 || ticks_per_rev_ <= 0 || drop_every_n_ < 0 || corrupt_every_n_ < 0)
    {
      throw std::runtime_error("rates, calibration, and fault injection parameters are invalid");
    }
    control_socket_ = udp_socket(listen_ip, static_cast<uint16_t>(control_port));
    telemetry_socket_ = udp_socket("0.0.0.0", 0);
    jetson_endpoint_ = endpoint(jetson_ip, static_cast<uint16_t>(telemetry_port));
    // 这是主机侧理想差速运动学和看门狗模型，不代表 FPGA RTL、PWM 或电机已经验证。
    receive_timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&FakeFpgaNode::receive_control, this));
    telemetry_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / telemetry_rate),
      std::bind(&FakeFpgaNode::send_telemetry, this));
    status_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / status_rate),
      std::bind(&FakeFpgaNode::send_status, this));
    RCLCPP_INFO(
      get_logger(), "fake FPGA listening on %s:%d, sending telemetry to %s:%d",
      listen_ip.c_str(), control_port, jetson_ip.c_str(), telemetry_port);
  }

  ~FakeFpgaNode() override
  {
    if (control_socket_ >= 0) {::close(control_socket_);}
    if (telemetry_socket_ >= 0) {::close(telemetry_socket_);}
  }

private:
  void receive_control()
  {
    for (;;) {
      std::vector<uint8_t> packet(2048);
      const ssize_t bytes = ::recvfrom(control_socket_, packet.data(), packet.size(), 0, nullptr, nullptr);
      if (bytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "control UDP receive failed: %s", std::strerror(errno));
        }
        return;
      }
      packet.resize(static_cast<size_t>(bytes));
      fpga_gateway::ControlPayload control{};
      fpga_gateway::PacketHeader header{};
      std::string error;
      if (!fpga_gateway::decode_control(packet, &control, &header, &error)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "discarding invalid control packet: %s",
          error.c_str());
        continue;
      }
      // UDP 可能乱序或重复；旧序号不能覆盖已经生效的较新命令。
      if (have_control_sequence_ &&
        !fpga_gateway::is_newer_sequence(header.sequence, last_control_sequence_)) {
        continue;
      }
      last_control_sequence_ = header.sequence;
      have_control_sequence_ = true;
      command_linear_ = control.linear_mm_s / 1000.0;
      command_angular_ = control.angular_mrad_s / 1000.0;
      command_estop_ = (control.flags & fpga_gateway::kControlEstop) != 0U;
      last_control_time_ = std::chrono::steady_clock::now();
      have_control_ = true;
    }
  }

  bool control_active() const
  {
    // 急停优先于普通命令；命令流中断超过看门狗期限也必须自然停车。
    return have_control_ && !command_estop_ &&
      std::chrono::steady_clock::now() - last_control_time_ <= watchdog_;
  }

  void send_telemetry()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(current_time - last_telemetry_time_).count();
    last_telemetry_time_ = current_time;
    const double linear = control_active() ? command_linear_ : 0.0;
    const double angular = control_active() ? command_angular_ : 0.0;
    // 用理想差速模型把车体速度换算为轮速和编码器累计值，仅供主机闭环测试。
    const double left = linear - angular * wheel_separation_ * 0.5;
    const double right = linear + angular * wheel_separation_ * 0.5;
    constexpr double kTau = 6.28318530717958647692;
    left_ticks_ += left * dt / (kTau * wheel_radius_) * ticks_per_rev_;
    right_ticks_ += right * dt / (kTau * wheel_radius_) * ticks_per_rev_;
    fpga_gateway::TelemetryPayload telemetry{};
    telemetry.left_ticks = static_cast<int32_t>(std::lround(left_ticks_));
    telemetry.right_ticks = static_cast<int32_t>(std::lround(right_ticks_));
    telemetry.left_velocity_mm_s = static_cast<int32_t>(std::lround(left * 1000.0));
    telemetry.right_velocity_mm_s = static_cast<int32_t>(std::lround(right * 1000.0));
    telemetry.accel_z_mm_s2 = 9810;
    telemetry.gyro_z_mrad_s = static_cast<int32_t>(std::lround(angular * 1000.0));
    telemetry.range_front_mm = 1200;
    telemetry.range_left_mm = 800;
    telemetry.range_right_mm = 800;
    send_packet(fpga_gateway::encode_telemetry(telemetry_sequence_++, now_us(), telemetry));
  }

  void send_status()
  {
    // 状态字段复用与硬件相同的协议语义，使网关可验证使能、急停和安全停车路径。
    fpga_gateway::StatusPayload status{};
    status.mode = 1;
    status.motion_enabled = control_active() ? 1U : 0U;
    status.estop = command_estop_ ? 1U : 0U;
    status.safety_stop = control_active() ? 0U : 1U;
    status.uptime_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time_).count());
    status.last_control_sequence = last_control_sequence_;
    send_packet(fpga_gateway::encode_status(status_sequence_++, now_us(), status));
  }

  void send_packet(std::vector<uint8_t> packet)
  {
    ++packet_count_;
    // 仅用于在主机上注入丢包和 CRC 错误，验证网关容错路径，不模拟真实链路质量。
    if (drop_every_n_ > 0 && packet_count_ % static_cast<uint64_t>(drop_every_n_) == 0U) {
      return;
    }
    if (corrupt_every_n_ > 0 && packet_count_ % static_cast<uint64_t>(corrupt_every_n_) == 0U && packet.size() > 4U) {
      packet[packet.size() / 2U] ^= 0x01U;
    }
    if (::sendto(telemetry_socket_, packet.data(), packet.size(), 0,
        reinterpret_cast<const sockaddr *>(&jetson_endpoint_), sizeof(jetson_endpoint_)) < 0 &&
      errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "telemetry UDP send failed: %s", std::strerror(errno));
    }
  }

  int control_socket_{-1};
  int telemetry_socket_{-1};
  sockaddr_in jetson_endpoint_{};
  std::chrono::steady_clock::time_point start_time_, last_control_time_{}, last_telemetry_time_;
  std::chrono::milliseconds watchdog_{};
  double wheel_separation_{}, wheel_radius_{}, command_linear_{}, command_angular_{}, left_ticks_{}, right_ticks_{};
  int ticks_per_rev_{}, drop_every_n_{}, corrupt_every_n_{};
  uint32_t telemetry_sequence_{}, status_sequence_{}, last_control_sequence_{};
  uint64_t packet_count_{};
  bool have_control_{false}, command_estop_{false}, have_control_sequence_{false};
  rclcpp::TimerBase::SharedPtr receive_timer_, telemetry_timer_, status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeFpgaNode>());
  rclcpp::shutdown();
  return 0;
}
