#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"

#include "fpga_gateway/protocol.hpp"

namespace {

uint64_t now_us()
{
  // 协议时间戳是可与 FPGA 日志/抓包对齐的 Unix 墙钟时间；它可能被 NTP 校时调整，不能用于超时判断。
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count());
}

int udp_socket(const std::string & bind_ip, uint16_t port)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("cannot create UDP socket: " + std::string(std::strerror(errno)));
  }
  // 定时器回调不得被网络阻塞：收包端在每次回调中排空当前队列，发包端忙时允许下一周期重试。
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("cannot set UDP socket nonblocking: " + error);
  }
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

double seconds_since(const std::chrono::steady_clock::time_point & time)
{
  // 超时与积分使用单调时钟，避免系统时间调整把命令或链路误判为新鲜/过期。
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - time).count();
}

bool valid_port(int port)
{
  return port > 0 && port <= std::numeric_limits<uint16_t>::max();
}

int32_t scaled_i32(double value, double scale)
{
  if (!std::isfinite(value)) {
    return 0;
  }
  const double scaled = value * scale;
  const double limited = std::clamp(
    scaled, static_cast<double>(std::numeric_limits<int32_t>::min()),
    static_cast<double>(std::numeric_limits<int32_t>::max()));
  return static_cast<int32_t>(std::lround(limited));
}

}  // namespace

class FpgaGatewayNode : public rclcpp::Node
{
public:
  FpgaGatewayNode()
  : Node("fpga_gateway_node")
  {
    fpga_ip_ = declare_parameter<std::string>("fpga_ip", "192.168.50.2");
    bind_ip_ = declare_parameter<std::string>("bind_ip", "0.0.0.0");
    control_port_ = declare_parameter<int>("control_port", 5000);
    telemetry_port_ = declare_parameter<int>("telemetry_port", 5001);
    const double control_rate = declare_parameter<double>("control_rate_hz", 100.0);
    const double receive_rate = declare_parameter<double>("receive_rate_hz", 500.0);
    const int command_timeout_ms = declare_parameter<int>("command_timeout_ms", 200);
    const int link_timeout_ms = declare_parameter<int>("link_timeout_ms", 300);
    command_timeout_ = std::chrono::milliseconds(command_timeout_ms);
    link_timeout_ = std::chrono::milliseconds(link_timeout_ms);
    wheel_separation_ = declare_parameter<double>("wheel_separation_m", 0.30);
    wheel_radius_ = declare_parameter<double>("wheel_radius_m", 0.05);
    ticks_per_rev_ = declare_parameter<int>("encoder_ticks_per_rev", 2048);
    wheel_position_variance_ = declare_parameter<double>("wheel_position_variance_m2", 0.01);
    wheel_yaw_variance_ = declare_parameter<double>("wheel_yaw_variance_rad2", 0.02);
    wheel_linear_variance_ = declare_parameter<double>("wheel_linear_velocity_variance", 0.02);
    wheel_angular_variance_ = declare_parameter<double>("wheel_angular_velocity_variance", 0.04);
    imu_angular_variance_ = declare_parameter<double>("imu_angular_velocity_variance", 0.01);
    imu_acceleration_variance_ = declare_parameter<double>("imu_linear_acceleration_variance", 0.10);
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    front_frame_ = declare_parameter<std::string>("range_front_frame_id", "range_front_link");
    left_frame_ = declare_parameter<std::string>("range_left_frame_id", "range_left_link");
    right_frame_ = declare_parameter<std::string>("range_right_frame_id", "range_right_link");
    if (!valid_port(control_port_) || !valid_port(telemetry_port_) ||
      control_rate <= 0.0 || receive_rate <= 0.0 || command_timeout_ms <= 0 ||
      link_timeout_ms <= 0 || wheel_separation_ <= 0.0 || wheel_radius_ <= 0.0 ||
      ticks_per_rev_ <= 0 || wheel_position_variance_ <= 0.0 || wheel_yaw_variance_ <= 0.0 ||
      wheel_linear_variance_ <= 0.0 || wheel_angular_variance_ <= 0.0 ||
      imu_angular_variance_ <= 0.0 || imu_acceleration_variance_ <= 0.0)
    {
      throw std::runtime_error("ports, rates, timeouts, and wheel calibration must be valid");
    }

    control_socket_ = udp_socket("0.0.0.0", 0);
    telemetry_socket_ = udp_socket(bind_ip_, static_cast<uint16_t>(telemetry_port_));
    fpga_control_endpoint_ = endpoint(fpga_ip_, static_cast<uint16_t>(control_port_));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    front_pub_ = create_publisher<sensor_msgs::msg::Range>("/range/front", 10);
    left_pub_ = create_publisher<sensor_msgs::msg::Range>("/range/left", 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Range>("/range/right", 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fpga/status", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        // 急停期间丢弃命令；解除急停不会恢复旧命令，必须收到新的 /cmd_vel 才会重新授权运动。
        if (emergency_stop_) {
          have_command_ = false;
          return;
        }
        linear_command_ = message->linear.x;
        angular_command_ = message->angular.z;
        command_time_ = std::chrono::steady_clock::now();
        have_command_ = true;
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        emergency_stop_ = message->data;
        if (emergency_stop_) {
          // 立即清除缓存命令，防止解除急停后因旧速度再次启动。
          linear_command_ = 0.0;
          angular_command_ = 0.0;
          have_command_ = false;
        }
      });

    control_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_rate),
      std::bind(&FpgaGatewayNode::send_control, this));
    receive_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / receive_rate),
      std::bind(&FpgaGatewayNode::receive_packets, this));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&FpgaGatewayNode::publish_diagnostics, this));
    RCLCPP_INFO(
      get_logger(), "UDP gateway listening on %s:%d, sending control to %s:%d",
      bind_ip_.c_str(), telemetry_port_, fpga_ip_.c_str(), control_port_);
  }

  ~FpgaGatewayNode() override
  {
    if (control_socket_ >= 0) {::close(control_socket_);}
    if (telemetry_socket_ >= 0) {::close(telemetry_socket_);}
  }

private:
  void send_control()
  {
    double linear = 0.0;
    double angular = 0.0;
    bool estop = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      estop = emergency_stop_;
      // 未收到命令、命令超过单调时钟超时或急停时，都持续向 FPGA 发送零速度。
      if (have_command_ && std::chrono::steady_clock::now() - command_time_ <= command_timeout_ && !estop) {
        linear = linear_command_;
        angular = angular_command_;
      }
    }
    fpga_gateway::ControlPayload payload{};
    // ROS 使用 m/s、rad/s；固定二进制协议使用 mm/s、mrad/s 的整数，避免浮点字节序问题。
    payload.linear_mm_s = scaled_i32(linear, 1000.0);
    payload.angular_mrad_s = scaled_i32(angular, 1000.0);
    payload.flags = estop ? fpga_gateway::kControlEstop : 0U;
    const auto packet = fpga_gateway::encode_control(control_sequence_++, now_us(), payload);
    if (::sendto(control_socket_, packet.data(), packet.size(), 0,
        reinterpret_cast<const sockaddr *>(&fpga_control_endpoint_), sizeof(fpga_control_endpoint_)) < 0 &&
      errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "control UDP send failed: %s", std::strerror(errno));
    }
  }

  void receive_packets()
  {
    // 非阻塞 socket：循环排空本周期已到达的数据报，直到 EAGAIN，避免旧遥测在内核队列中累积。
    for (;;) {
      std::vector<uint8_t> packet(2048);
      const ssize_t bytes = ::recvfrom(telemetry_socket_, packet.data(), packet.size(), 0, nullptr, nullptr);
      if (bytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "telemetry UDP receive failed: %s", std::strerror(errno));
        }
        return;
      }
      packet.resize(static_cast<size_t>(bytes));
      fpga_gateway::PacketHeader header{};
      std::string error;
      if (!fpga_gateway::decode_header(packet, &header, &error)) {
        // decode_header 同时验证固定头、长度、CRC 与协议版本，失败包不影响已有状态。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "discarding invalid FPGA packet: %s",
          error.c_str());
        continue;
      }
      if (header.message_type == static_cast<uint8_t>(fpga_gateway::MessageType::Telemetry)) {
        fpga_gateway::TelemetryPayload telemetry{};
        if (!fpga_gateway::decode_telemetry(packet, &telemetry, &header, &error)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "discarding invalid telemetry: %s",
            error.c_str());
          continue;
        }
        // 类型、载荷、CRC 均通过后才按独立遥测序号门禁，丢弃重复包和回退包。
        if (accept_sequence(header.sequence, telemetry_sequence_, have_telemetry_sequence_)) {
          last_link_time_ = std::chrono::steady_clock::now();
          have_link_ = true;
          publish_telemetry(telemetry);
        }
      } else if (header.message_type == static_cast<uint8_t>(fpga_gateway::MessageType::Status)) {
        fpga_gateway::StatusPayload status{};
        if (!fpga_gateway::decode_status(packet, &status, &header, &error)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "discarding invalid status: %s",
            error.c_str());
          continue;
        }
        // 状态与遥测各自维护序号，避免两类报文交错时相互误判为过期。
        if (accept_sequence(header.sequence, status_sequence_, have_status_sequence_)) {
          last_link_time_ = std::chrono::steady_clock::now();
          have_link_ = true;
          last_status_ = status;
          last_status_time_ = std::chrono::steady_clock::now();
          have_status_ = true;
        }
      }
    }
  }

  bool accept_sequence(uint32_t sequence, uint32_t & previous, bool & valid)
  {
    if (valid && !fpga_gateway::is_newer_sequence(sequence, previous)) {
      return false;
    }
    previous = sequence;
    valid = true;
    return true;
  }

  void publish_telemetry(const fpga_gateway::TelemetryPayload & value)
  {
    const auto stamp = now();
    // 遥测中的左右轮线速度为 mm/s，转换为 ROS 约定的 m/s 后再计算差速底盘速度。
    const double left = value.left_velocity_mm_s / 1000.0;
    const double right = value.right_velocity_mm_s / 1000.0;
    const auto telemetry_time = std::chrono::steady_clock::now();
    double dt = 0.0;
    if (have_telemetry_time_) {
      dt = seconds_since(last_telemetry_time_);
      // 主机被抢占或断链恢复后最多按 0.1 s 积分，避免位置突跳；长间隔内的运动会被舍弃。
      dt = std::min(dt, 0.1);
    }
    last_telemetry_time_ = telemetry_time;
    have_telemetry_time_ = true;
    const double linear = (left + right) * 0.5;
    const double angular = (right - left) / wheel_separation_;
    const double middle_yaw = yaw_ + angular * dt * 0.5;
    x_ += linear * std::cos(middle_yaw) * dt;
    y_ += linear * std::sin(middle_yaw) * dt;
    yaw_ += angular * dt;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    // 非零协方差是融合器区分编码器与 IMU 权重的契约；实车需通过直行和定点旋转数据重新标定。
    odom.pose.covariance[0] = wheel_position_variance_;
    odom.pose.covariance[7] = wheel_position_variance_;
    odom.pose.covariance[35] = wheel_yaw_variance_;
    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;
    odom.twist.covariance[0] = wheel_linear_variance_;
    odom.twist.covariance[35] = wheel_angular_variance_;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    // 当前遥测协议只提供角速度和线加速度，未提供姿态解算；ROS 约定以 -1 标记 orientation 不可用。
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = value.gyro_x_mrad_s / 1000.0;
    imu.angular_velocity.y = value.gyro_y_mrad_s / 1000.0;
    imu.angular_velocity.z = value.gyro_z_mrad_s / 1000.0;
    imu.angular_velocity_covariance[0] = imu_angular_variance_;
    imu.angular_velocity_covariance[4] = imu_angular_variance_;
    imu.angular_velocity_covariance[8] = imu_angular_variance_;
    imu.linear_acceleration.x = value.accel_x_mm_s2 / 1000.0;
    imu.linear_acceleration.y = value.accel_y_mm_s2 / 1000.0;
    imu.linear_acceleration.z = value.accel_z_mm_s2 / 1000.0;
    imu.linear_acceleration_covariance[0] = imu_acceleration_variance_;
    imu.linear_acceleration_covariance[4] = imu_acceleration_variance_;
    imu.linear_acceleration_covariance[8] = imu_acceleration_variance_;
    imu_pub_->publish(imu);

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"left_wheel_joint", "right_wheel_joint"};
    constexpr double kTau = 6.28318530717958647692;
    // 编码器累计 tick 转关节角度，轮速 m/s 除以半径得到关节 rad/s。
    const double ticks_to_rad = kTau / ticks_per_rev_;
    joints.position = {value.left_ticks * ticks_to_rad, value.right_ticks * ticks_to_rad};
    joints.velocity = {left / wheel_radius_, right / wheel_radius_};
    joint_pub_->publish(joints);
    publish_range(front_pub_, front_frame_, value.range_front_mm, stamp);
    publish_range(left_pub_, left_frame_, value.range_left_mm, stamp);
    publish_range(right_pub_, right_frame_, value.range_right_mm, stamp);
  }

  void publish_range(const rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr & publisher,
    const std::string & frame, uint16_t millimeters, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Range range;
    range.header.stamp = stamp;
    range.header.frame_id = frame;
    range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range.field_of_view = 0.35F;
    range.min_range = 0.02F;
    range.max_range = 4.0F;
    range.range = millimeters / 1000.0F;
    publisher->publish(range);
  }

  void publish_diagnostics()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    // online 表示任一被接受的遥测或状态报文仍在到达；status_fresh 额外要求专用状态报文未超时。
    const bool online = have_link_ && steady_now - last_link_time_ <= link_timeout_;
    const bool status_fresh = have_status_ && steady_now - last_status_time_ <= link_timeout_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fpga_gateway";
    status.hardware_id = fpga_ip_;
    if (!online) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "FPGA link offline";
    } else if (!status_fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "FPGA status stale";
    } else if (last_status_.fault_bits != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "FPGA fault";
    } else if (last_status_.estop != 0U || last_status_.safety_stop != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "FPGA safety stop";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    }
    status.values = {
      value_of("mode", last_status_.mode),
      value_of("motion_enabled", last_status_.motion_enabled),
      value_of("estop", last_status_.estop),
      value_of("safety_stop", last_status_.safety_stop),
      value_of("fault_bits", last_status_.fault_bits),
      value_of("uptime_ms", last_status_.uptime_ms),
      value_of("last_control_sequence", last_status_.last_control_sequence),
      value_of("online", online), value_of("status_fresh", status_fresh)};
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    status_pub_->publish(array);
  }

  template<typename T>
  diagnostic_msgs::msg::KeyValue value_of(const std::string & key, T value) const
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = std::to_string(value);
    return result;
  }

  diagnostic_msgs::msg::KeyValue value_of(const std::string & key, bool value) const
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value ? "true" : "false";
    return result;
  }

  int control_socket_{-1};
  int telemetry_socket_{-1};
  sockaddr_in fpga_control_endpoint_{};
  std::string fpga_ip_, bind_ip_, odom_frame_, base_frame_, imu_frame_, front_frame_, left_frame_, right_frame_;
  int control_port_{}, telemetry_port_{}, ticks_per_rev_{};
  double wheel_separation_{}, wheel_radius_{}, wheel_position_variance_{}, wheel_yaw_variance_{},
    wheel_linear_variance_{}, wheel_angular_variance_{}, imu_angular_variance_{}, imu_acceleration_variance_{};
  std::chrono::milliseconds command_timeout_{}, link_timeout_{};
  std::chrono::steady_clock::time_point command_time_{}, last_link_time_{},
    last_status_time_{}, last_telemetry_time_{};
  std::mutex command_mutex_;
  double linear_command_{}, angular_command_{};
  bool have_command_{false}, emergency_stop_{false}, have_link_{false},
    have_status_{false}, have_telemetry_time_{false};
  uint32_t control_sequence_{}, telemetry_sequence_{}, status_sequence_{};
  bool have_telemetry_sequence_{false}, have_status_sequence_{false};
  fpga_gateway::StatusPayload last_status_{};
  double x_{}, y_{}, yaw_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_pub_, left_pub_, right_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_, receive_timer_, diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FpgaGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
