#include "udp_input.hpp"
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr std::size_t SCAN_BINS = 361;
constexpr std::uint16_t MAX_ANGLE_GAP_CDEG = 300;
// Match navigation's existing 0.5 s laser-observation budget.
constexpr std::int64_t MAX_FRONT_SCAN_NS = 500000000;
constexpr std::uint32_t COUNTER_HALF_RANGE = 0x80000000U;
constexpr std::int64_t DEVICE_HALF_PERIOD_NS = static_cast<std::int64_t>(COUNTER_HALF_RANGE) * 1000;

using ugv_bridge::u16;
using ugv_bridge::u32;

std::uint16_t gray16_to_binary(std::uint16_t value)
{
  value ^= value >> 8;
  value ^= value >> 4;
  value ^= value >> 2;
  value ^= value >> 1;
  return value;
}

std::int16_t i16(const std::uint8_t *p)
{
  const auto bits = u16(p);
  std::int16_t value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::int32_t i32(const std::uint8_t *p)
{
  const auto bits = u32(p);
  std::int32_t value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

float f32(const std::uint8_t *p)
{
  const auto bits = u32(p);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

class ClockMapper {
public:
  std::optional<bool> observe(std::uint32_t stamp, std::uint32_t sequence, std::uint32_t boot,
    std::uint32_t domain, std::int64_t receive_ns, std::int64_t steady_ns = 0)
  {
    const auto sequence_delta = static_cast<std::uint32_t>(sequence - last_sequence_);
    const auto time_delta = static_cast<std::uint32_t>(stamp - last_low_);
    const bool session_changed = valid_ && (boot != boot_ || domain != domain_);
    // A late packet from the previous command session can move both fields
    // backwards.  A control-clock restart instead begins again near zero;
    // boot can already be nonzero if a heartbeat arrived before telemetry.
    const bool restarted_without_boot_id = valid_ && sequence_delta >= COUNTER_HALF_RANGE &&
      sequence < 512U && sequence < last_sequence_ / 2U && stamp < 10000000U &&
      (!boot || (stamp < last_low_ && last_low_ - stamp > 50000U));
    const auto receive_gap = receive_ns - last_receive_ns_;
    const bool have_steady = last_steady_ns_ && steady_ns;
    const auto steady_gap = have_steady ? steady_ns - last_steady_ns_ : receive_gap;
    const bool host_clock_step = have_steady && std::abs(receive_gap - steady_gap) > 200000000LL;
    const bool ambiguous_gap = steady_gap >= DEVICE_HALF_PERIOD_NS;
    const bool counter_discontinuity = time_delta >= COUNTER_HALF_RANGE ||
      receive_gap - static_cast<std::int64_t>(time_delta) * 1000 > 1000000000LL;
    // A long outage can hide a reboot or span one or more counter wraps.
    const bool confirm_epoch = valid_ && (ambiguous_gap ||
      (receive_gap > 1000000000LL && counter_discontinuity &&
       (sequence_delta >= COUNTER_HALF_RANGE || time_delta >= COUNTER_HALF_RANGE)));
    if (confirm_epoch) {
      const auto ds = static_cast<std::uint32_t>(sequence - candidate_sequence_);
      const auto dt = static_cast<std::uint32_t>(stamp - candidate_stamp_);
      const auto elapsed = receive_ns - candidate_receive_ns_;
      if (candidate_count_ && ds > 0 && ds < COUNTER_HALF_RANGE && dt > 0 &&
          dt < COUNTER_HALF_RANGE && elapsed > 0 &&
          std::abs(static_cast<std::int64_t>(dt) * 1000 - elapsed) < 200000000LL)
        ++candidate_count_;
      else candidate_count_ = 1;
      candidate_sequence_ = sequence;
      candidate_stamp_ = stamp;
      candidate_receive_ns_ = receive_ns;
    } else candidate_count_ = 0;
    // boot is a command-session ID, not a hardware clock epoch.  A new
    // session can arrive while the device counter continues across wrap.
    const bool new_epoch = !valid_ || host_clock_step || receive_ns < last_receive_ns_ ||
      (!ambiguous_gap && restarted_without_boot_id) || candidate_count_ >= 3;
    if (new_epoch) {
      candidate_count_ = 0;
      valid_ = true;
      boot_ = boot;
      domain_ = domain;
      anchor_us_ = stamp;
      last_low_ = stamp;
      last_sequence_ = sequence;
      offset_ns_ = receive_ns - static_cast<std::int64_t>(anchor_us_) * 1000;
      last_receive_ns_ = receive_ns;
      last_steady_ns_ = steady_ns;
      return true;
    }
    const auto delta = time_delta;
    if (confirm_epoch || delta >= COUNTER_HALF_RANGE || sequence_delta >= COUNTER_HALF_RANGE ||
        (!sequence_delta && !delta && !session_changed)) return std::nullopt;
    anchor_us_ += delta;
    last_low_ = stamp;
    last_sequence_ = sequence;
    const auto sample_offset = receive_ns - static_cast<std::int64_t>(anchor_us_) * 1000;
    const auto correction = sample_offset - offset_ns_;
    // A delayed navigation packet can only inflate the receive-time offset.
    // Accept a lower-latency sample immediately so later sensor packets do
    // not remain incorrectly mapped into the future for minutes.
    if (correction < 0) offset_ns_ = sample_offset;
    else if (delta) offset_ns_ += std::min<std::int64_t>(correction / 16, 10000);
    boot_ = boot;
    domain_ = domain;
    last_receive_ns_ = receive_ns;
    last_steady_ns_ = steady_ns;
    return session_changed || correction < -200000000LL;
  }

  std::optional<rclcpp::Time> map(std::uint32_t stamp, std::int64_t receive_ns) const
  {
    if (!valid_ || candidate_count_ || receive_ns - last_receive_ns_ >= DEVICE_HALF_PERIOD_NS)
      return std::nullopt;
    const auto difference = static_cast<std::uint32_t>(stamp - static_cast<std::uint32_t>(anchor_us_));
    const auto delta = difference <= 0x7fffffffU ? static_cast<std::int64_t>(difference) :
      static_cast<std::int64_t>(difference) - 0x100000000LL;
    const auto extended = static_cast<std::int64_t>(anchor_us_) + delta;
    const auto mapped_ns = extended * 1000 + offset_ns_;
    // A sensor packet can precede the first navigation packet after a reboot.
    if (extended < 0 || mapped_ns < 0 || mapped_ns > receive_ns + 200000000LL) return std::nullopt;
    return rclcpp::Time(mapped_ns, RCL_SYSTEM_TIME);
  }

private:
  unsigned candidate_count_ = 0;
  std::uint32_t candidate_sequence_ = 0, candidate_stamp_ = 0;
  std::int64_t candidate_receive_ns_ = 0;
  bool valid_ = false;
  std::uint32_t boot_ = 0, domain_ = 0, last_low_ = 0, last_sequence_ = 0;
  std::uint64_t anchor_us_ = 0;
  std::int64_t offset_ns_ = 0, last_receive_ns_ = 0, last_steady_ns_ = 0;
};

class ScanPlane {
public:
  void reset()
  {
    have_imu_ = have_gravity_ = level_ = complete_scan_ = false;
    history_next_ = history_size_ = 0;
  }

  bool observe(std::int64_t stamp_ns, double ax, double ay, double az,
               double wx = 0.0, double wy = 0.0, double wz = 0.0)
  {
    if (have_imu_ && stamp_ns <= imu_stamp_ns_) {
      level_ = complete_scan_ = false;
      return false;
    }
    const bool continuous = !have_imu_ || valid_at(stamp_ns);
    const double dt = have_imu_ ? (stamp_ns - imu_stamp_ns_) * 1e-9 : 0.0;
    have_imu_ = true;
    imu_stamp_ns_ = stamp_ns;
    const double norm = std::sqrt(ax*ax + ay*ay + az*az);
    const bool acceleration_valid = norm >= .8*9.80665 && norm <= 1.2*9.80665;
    if (dt > .05) have_gravity_ = false;
    if (have_gravity_) {
      // Gravity in the rotating sensor frame follows g x omega. Acceleration
      // corrects drift over one second without treating short driving pulses as tilt.
      const auto previous = gravity_;
      gravity_[0] += dt * (previous[1]*wz - previous[2]*wy);
      gravity_[1] += dt * (previous[2]*wx - previous[0]*wz);
      gravity_[2] += dt * (previous[0]*wy - previous[1]*wx);
      if (acceleration_valid) {
        const double gain = dt / (1.0 + dt);
        gravity_[0] += gain * (ax/norm - gravity_[0]);
        gravity_[1] += gain * (ay/norm - gravity_[1]);
        gravity_[2] += gain * (az/norm - gravity_[2]);
      }
      const double length = std::sqrt(gravity_[0]*gravity_[0] +
        gravity_[1]*gravity_[1] + gravity_[2]*gravity_[2]);
      for (auto &axis : gravity_) axis /= length;
    } else if (acceleration_valid) {
      gravity_ = {ax/norm, ay/norm, az/norm};
      have_gravity_ = true;
    }
    // Rx(pi) mounting: level gravity points along sensor -Z. A bump can disturb
    // acceleration magnitude without tilting the scan plane; reject near free
    // fall immediately, and use gyro-predicted gravity for the 15 degree limit.
    level_ = norm >= .2*9.80665 && have_gravity_ && -gravity_[2] >= 0.9659258262890683;
    history_[history_next_] = {stamp_ns, level_};
    history_next_ = (history_next_ + 1) % history_.size();
    history_size_ = std::min(history_size_ + 1, history_.size());
    if (!level_ || !continuous) complete_scan_ = false;
    return level_ && continuous;
  }

  bool valid_at(std::int64_t stamp_ns) const
  {
    const auto *sample = imu_at(stamp_ns);
    return level_ && sample && sample->level && stamp_ns - sample->stamp_ns <= 50000000;
  }

  std::int64_t age_us(std::int64_t stamp_ns) const
  {
    const auto *sample = imu_at(stamp_ns);
    return sample ? (stamp_ns - sample->stamp_ns) / 1000 : -1;
  }

  void discard_scan() { complete_scan_ = false; }
  void complete_scan() { complete_scan_ = true; }
  double tilt_radians() const
  {
    return have_gravity_ ? std::acos(std::clamp(-gravity_[2], -1.0, 1.0)) : -1.0;
  }
  bool scan_valid_at(std::int64_t stamp_ns) const
  {
    return complete_scan_ && valid_at(stamp_ns);
  }

private:
  struct ImuSample {
    std::int64_t stamp_ns;
    bool level;
  };

  const ImuSample *imu_at(std::int64_t stamp_ns) const
  {
    // Fusion/odom telemetry can arrive after a newer IMU. Match its source
    // time to past evidence; 16 samples cover the 50 ms gate at 120 Hz.
    for (std::size_t offset = 1; offset <= history_size_; ++offset) {
      const auto &sample = history_[(history_next_ + history_.size() - offset) % history_.size()];
      if (sample.stamp_ns <= stamp_ns) return &sample;
    }
    return nullptr;
  }

  std::array<ImuSample, 16> history_{};
  std::array<double, 3> gravity_{};
  std::size_t history_next_ = 0, history_size_ = 0;
  std::int64_t imu_stamp_ns_ = 0;
  bool have_imu_ = false, have_gravity_ = false, level_ = false, complete_scan_ = false;
};

class FrontScan {
public:
  struct Diagnostics {
    std::uint64_t packets = 0, sequence_gaps = 0, angle_gaps = 0;
    std::uint64_t rejected_order = 0, rejected_format = 0, rejected_time = 0, rejected_sparse = 0;
  };

  FrontScan() { clear(); }
  const Diagnostics &diagnostics() const { return diagnostics_; }

  void reset()
  {
    clear();
    have_previous_ = false;
    have_sequence_ = false;
    have_stamp_ = false;
    last_scan_stamp_.reset();
  }

  std::optional<sensor_msgs::msg::LaserScan> consume(
    const std::uint8_t *packet, const rclcpp::Time &stamp, const std::string &frame_id,
    float range_min, float range_max)
  {
    const auto sequence = u32(packet + 8);
    ++diagnostics_.packets;
    const auto sequence_delta = static_cast<std::uint32_t>(sequence - last_sequence_);
    if (have_sequence_ && (sequence_delta == 0 || sequence_delta >= 0x80000000U)) {
      ++diagnostics_.rejected_order;
      return std::nullopt;
    }
    if (have_sequence_ && sequence_delta > 1) ++diagnostics_.sequence_gaps;
    have_sequence_ = true;
    last_sequence_ = sequence;
    const auto stamp_ns = stamp.nanoseconds();
    if (have_stamp_ && stamp_ns < previous_stamp_ns_) {
      ++diagnostics_.rejected_time;
      reset();
      return std::nullopt;
    }
    have_stamp_ = true;
    previous_stamp_ns_ = stamp_ns;
    if (active_ && stamp_ns - started_stamp_ns_ > MAX_FRONT_SCAN_NS) {
      ++diagnostics_.rejected_time;
      clear();
    }
    std::optional<sensor_msgs::msg::LaserScan> output;
    for (std::size_t i = 0; i < 16; ++i) {
      const auto *point = packet + 32 + i * 8;
      const auto angle = u16(point);
      const auto distance = u16(point + 2);
      if (angle >= 36000 || point[5] != i || u16(point + 6) != 0) {
        ++diagnostics_.rejected_format;
        if (active_) bad_ = true;
        continue;
      }
      const bool wrap = have_previous_ && previous_angle_ > 30000 && angle < 6000;
      const bool backwards = have_previous_ && !wrap && angle < previous_angle_;
      const std::uint32_t gap = !have_previous_ ? 0 :
        wrap ? 36000U + angle - previous_angle_ :
        backwards ? 36000U : static_cast<std::uint32_t>(angle - previous_angle_);
      const bool enters = have_previous_ && !wrap && previous_angle_ < 27000 && angle >= 27000;
      const bool exits = have_previous_ && !wrap && previous_angle_ <= 9000 && angle > 9000;
      if (active_ && backwards) {
        bad_ = true;
        ++diagnostics_.rejected_order;
      }
      if (active_ && gap > MAX_ANGLE_GAP_CDEG) ++diagnostics_.angle_gaps;
      if (enters) {
        clear();
        active_ = true;
        started_stamp_ns_ = stamp_ns;
      }
      if (active_ && wrap) {
        midpoint_stamp_ = stamp;
        have_midpoint_ = true;
      }
      if (active_ && exits) {
        output = finish(frame_id, range_min, range_max);
        clear();
      }
      if (active_ && (angle <= 9000 || angle >= 27000)) {
        const int ros_cdeg = angle <= 9000 ? -static_cast<int>(angle) :
          36000 - static_cast<int>(angle);
        const auto index = static_cast<std::size_t>((ros_cdeg + 9000 + 25) / 50);
        ++samples_;
        if (index < SCAN_BINS && distance != 0 && distance != 0xffff) {
          const float metres = static_cast<float>(distance) * 0.001F;
          if (metres >= range_min && metres <= range_max &&
              (std::isnan(ranges_[index]) || metres < ranges_[index])) {
            ranges_[index] = metres;
            intensities_[index] = static_cast<float>(point[4]);
          }
        }
      }
      previous_angle_ = angle;
      have_previous_ = true;
    }
    return output;
  }

private:
  void clear()
  {
    // Missing packets/angles stay unknown; they do not erase other real hits.
    ranges_.fill(std::numeric_limits<float>::quiet_NaN());
    intensities_.fill(0.0F);
    active_ = false;
    bad_ = false;
    have_midpoint_ = false;
    samples_ = 0;
  }

  std::optional<sensor_msgs::msg::LaserScan> finish(
    const std::string &frame_id, float range_min, float range_max)
  {
    if (bad_) return std::nullopt;
    if (!have_midpoint_ || samples_ < 64) {
      ++diagnostics_.rejected_sparse;
      return std::nullopt;
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = frame_id;
    scan.header.stamp = midpoint_stamp_;
    scan.angle_min = static_cast<float>(-PI / 2.0);
    scan.angle_max = static_cast<float>(PI / 2.0);
    scan.angle_increment = static_cast<float>(PI / 360.0);
    scan.time_increment = 0.0F;
    if (last_scan_stamp_ && midpoint_stamp_.nanoseconds() > last_scan_stamp_->nanoseconds())
      scan.scan_time = static_cast<float>((midpoint_stamp_ - *last_scan_stamp_).seconds());
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.ranges.assign(ranges_.begin(), ranges_.end());
    scan.intensities.assign(intensities_.begin(), intensities_.end());
    last_scan_stamp_ = midpoint_stamp_;
    return scan;
  }

  std::array<float, SCAN_BINS> ranges_{};
  std::array<float, SCAN_BINS> intensities_{};
  std::optional<rclcpp::Time> last_scan_stamp_;
  rclcpp::Time midpoint_stamp_{0, 0, RCL_SYSTEM_TIME};
  Diagnostics diagnostics_;
  std::uint32_t last_sequence_ = 0;
  std::int64_t previous_stamp_ns_ = 0, started_stamp_ns_ = 0;
  std::uint16_t previous_angle_ = 0;
  std::size_t samples_ = 0;
  bool active_ = false, bad_ = false, have_previous_ = false, have_midpoint_ = false;
  bool have_sequence_ = false, have_stamp_ = false;
};

class GatewayNode : public rclcpp::Node {
public:
  GatewayNode() : Node("robot_gateway")
  {
    const auto port = declare_parameter<int>("telemetry_port", 6104);
    const auto source_address = declare_parameter<std::string>("source_address", "203.0.113.2");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto laser_x = declare_parameter<double>("laser_x_m", 0.13);
    const auto laser_y = declare_parameter<double>("laser_y_m", 0.0);
    const auto laser_yaw = declare_parameter<double>("laser_yaw_rad", 0.0);
    range_min_ = static_cast<float>(declare_parameter<double>("range_min_m", 0.05));
    range_max_ = static_cast<float>(declare_parameter<double>("range_max_m", 12.0));
    if (port < 1 || port > 65535 || laser_frame_.empty() || imu_frame_.empty() ||
        odom_frame_.empty() || base_frame_.empty() || !std::isfinite(laser_x) ||
        !std::isfinite(laser_y) || !std::isfinite(laser_yaw) || range_min_ < 0.0F ||
        !std::isfinite(range_min_) || !std::isfinite(range_max_) || range_max_ <= range_min_)
      throw std::invalid_argument("invalid gateway parameters");
    source_addr_ = ugv_bridge::udp_source(source_address);
    socket_ = open_telemetry_socket(static_cast<std::uint16_t>(port));
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    health_pub_ = create_publisher<std_msgs::msg::String>("fpga/video_health", 10);
    nav_pub_ = create_publisher<std_msgs::msg::String>("fpga/navigation_status", 10);
    motor_pub_ = create_publisher<std_msgs::msg::String>("fpga/motor", 10);
    fusion_health_pub_ = create_publisher<std_msgs::msg::String>("fpga/fusion_health", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("fpga/local_map", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped laser_tf;
    laser_tf.header.stamp = now();
    laser_tf.header.frame_id = base_frame_;
    laser_tf.child_frame_id = laser_frame_;
    laser_tf.transform.translation.x = laser_x;
    laser_tf.transform.translation.y = laser_y;
    laser_tf.transform.rotation.z = std::sin(laser_yaw / 2.0);
    laser_tf.transform.rotation.w = std::cos(laser_yaw / 2.0);
    static_tf_->sendTransform(laser_tf);
    receiver_ = std::thread([this, port] { receive_loop(static_cast<std::uint16_t>(port)); });
    RCLCPP_INFO(get_logger(), "Receiving FPGA telemetry on UDP %d", static_cast<int>(port));
  }

  ~GatewayNode() override
  {
    running_ = false;
    if (receiver_.joinable()) receiver_.join();
    if (socket_ >= 0) close(socket_);
  }

private:
  void publish_fusion_health(const std::uint8_t *p, const rclcpp::Time &stamp)
  {
    std::ostringstream json;
    json << "{\"stamp_ns\":" << stamp.nanoseconds()
         << ",\"flags\":" << u32(p + 72) << ",\"max_control_gap_us\":" << u32(p + 84)
         << ",\"lidar_drops\":" << u32(p + 92) << ",\"lidar_errors\":" << u32(p + 96)
         << ",\"imu_age_us\":" << u32(p + 104) << ",\"lidar_age_us\":" << u32(p + 108)
         << ",\"nav_status\":" << u32(p + 112)
         << ",\"scan_plane_valid\":" << (scan_plane_.scan_valid_at(stamp.nanoseconds()) ? "true" : "false")
         << ",\"scan_plane_tilt_deg\":" << scan_plane_.tilt_radians() * 180.0 / PI
         << ",\"scan_plane_imu_age_us\":" << scan_plane_.age_us(stamp.nanoseconds())
         << ",\"raw_covariance\":[";
    for (int i = 0; i < 3; ++i) {
      if (i) json << ',';
      const auto value = f32(p + 52 + i * 4);
      if (std::isfinite(value)) json << value;
      else json << "null";
    }
    json << "]}";
    std_msgs::msg::String message;
    message.data = json.str();
    fusion_health_pub_->publish(message);
  }

  void publish_motor(const std::uint8_t *p)
  {
    std::ostringstream json;
    json << "{\"sequence\":" << u32(p + 8) << ",\"time_us\":" << u32(p + 12)
         << ",\"motor_map\":" << unsigned(p[16]) << ",\"encoder_map\":" << unsigned(p[17])
         << ",\"motor_invert\":" << unsigned(p[18]) << ",\"encoder_invert\":" << unsigned(p[19])
         << ",\"command\":" << unsigned(p[20]) << ",\"pwm_percent\":" << unsigned(p[21])
         << ",\"active_mask\":" << unsigned(p[22]) << ",\"invalid_mask\":" << unsigned(p[23]);
    for (const auto &field : {std::pair<const char *, int>{"ticks", 24},
                              {"physical_ticks", 40}, {"measured_mm_s", 56},
                              {"target_mm_s", 72}}) {
      json << ",\"" << field.first << "\":[";
      for (int i = 0; i < 4; ++i) {
        if (i) json << ',';
        json << i32(p + field.second + i * 4);
      }
      json << ']';
    }
    json << ",\"vx_mm_s\":" << i32(p + 88) << ",\"vy_mm_s\":" << i32(p + 92)
         << ",\"wz_mrad_s\":" << i32(p + 96) << ",\"dropped\":" << u32(p + 100) << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    motor_pub_->publish(message);
  }

  void publish_navigation(const std::uint8_t *p)
  {
    const auto boot = u32(p + 16), domain = u32(p + 20);
    if (boot != grid_boot_ || domain != grid_domain_) {
      grid_boot_ = boot;
      grid_domain_ = domain;
      grid_seen_ = 0;
      grid_active_ = false;
      grid_invalid_ = grid_completed_ = false;
    }
    const auto token = (static_cast<std::uint64_t>(u32(p + 40)) << 32) | u32(p + 44);
    std::ostringstream json;
    json << "{\"boot_id\":" << boot << ",\"odom_domain\":" << domain
         << ",\"task_id\":" << u32(p + 24) << ",\"last_task_id\":" << u32(p + 28)
         << ",\"expiry_us\":" << u32(p + 32) << ",\"now_us\":" << u32(p + 36)
         << ",\"session_token\":" << token << ",\"status\":" << unsigned(p[48])
         << ",\"error\":" << unsigned(p[49]) << ",\"mode\":" << unsigned(p[50])
         << ",\"flags\":" << unsigned(p[51])
         << ",\"guard_missing\":" << ((p[51] & 8U) != 0)
         << ",\"guard_obstacle\":" << ((p[51] & 16U) != 0)
         << ",\"vx_mm_s\":" << i16(p + 52) << ",\"vy_mm_s\":" << i16(p + 54)
         << ",\"wz_mrad_s\":" << i16(p + 56)
         << ",\"planner_expansions\":" << u16(p + 58)
         << ",\"goal_x_m\":" << static_cast<double>(i32(p + 60)) / 65536.0
         << ",\"goal_y_m\":" << static_cast<double>(i32(p + 64)) / 65536.0
         << ",\"progress\":" << u32(p + 68)
         << ",\"map_age_us\":" << u32(p + 72)
         << ",\"pose_age_us\":" << u32(p + 76)
         << ",\"map_version\":" << u32(p + 80) << "}";
    std_msgs::msg::String message;
    message.data = json.str();
    nav_pub_->publish(message);
  }

  void receive_grid(const std::uint8_t *p, const rclcpp::Time &stamp)
  {
    const auto boot = u32(p + 16), domain = u32(p + 20), id = u32(p + 24);
    const auto part = u32(p + 92);
    if (!boot || !domain || boot != grid_boot_ || domain != grid_domain_ ||
        (part & 0xfffffff8U) != 0x01320800U || (u32(p + 84) & ~31U) ||
        static_cast<std::uint32_t>(u32(p + 88) - u32(p + 28)) > 100000U ||
        i32(p + 56) || i32(p + 60) || i32(p + 64) || i32(p + 68) || i32(p + 72)) return;
    const auto at = std::chrono::steady_clock::now();
    if (!grid_active_ || id != grid_id_) {
      if (grid_active_ && static_cast<std::int32_t>(id - grid_id_) <= 0) return;
      std::copy(p + 16, p + 92, grid_header_.begin());
      grid_id_ = id;
      grid_seen_ = 0;
      grid_active_ = true;
      grid_invalid_ = grid_completed_ = false;
      grid_at_ = at;
    }
    if (grid_invalid_ || grid_completed_) return;
    if (at - grid_at_ > 250ms || !std::equal(p + 16, p + 92, grid_header_.begin())) {
      grid_invalid_ = true;
      return;
    }
    const auto index = part & 7U;
    const auto bit = static_cast<std::uint8_t>(1U << index);
    const auto destination = grid_cells_.begin() + index * 128U;
    if (grid_seen_ & bit) {
      if (!std::equal(p + 96, p + 224, destination)) grid_invalid_ = true;
      return;
    }
    for (std::size_t i = 96; i < 224; ++i)
      for (unsigned shift = 0; shift < 8; shift += 2)
        if (((p[i] >> shift) & 3U) == 3U) { grid_invalid_ = true; return; }
    std::copy(p + 96, p + 224, destination);
    grid_seen_ |= bit;
    if (grid_seen_ != 0xffU) return;
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = odom_frame_;
    grid.header.stamp = stamp;
    grid.info.resolution = 0.05F;
    grid.info.width = grid.info.height = 64;
    grid.info.origin.position.x = i32(p + 40) * 0.05;
    grid.info.origin.position.y = i32(p + 44) * 0.05;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(4096);
    for (std::size_t i = 0; i < grid.data.size(); ++i) {
      const auto state = (grid_cells_[i / 4] >> ((i % 4) * 2)) & 3U;
      grid.data[i] = state == 0 ? -1 : state == 1 ? 0 : 100;
    }
    grid_pub_->publish(grid);
    grid_completed_ = true;
  }

  void publish_imu(const std::uint8_t *p, const rclcpp::Time &stamp)
  {
    // A UDP duplicate is the same physical sample even if a newer clock
    // anchor would give it a slightly different ROS timestamp.
    const std::uint64_t sample = (static_cast<std::uint64_t>(u32(p + 8)) << 32) | u32(p + 12);
    if (last_imu_packet_ && *last_imu_packet_ == sample) return;
    last_imu_packet_ = sample;
    if (p[16] != 15 || p[17] != 0x70) {
      scan_plane_.observe(stamp.nanoseconds(), 0.0, 0.0, 0.0);
      scan_.reset();
      return;
    }
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation_covariance[0] = -1.0;
    constexpr double gyro_scale = 0.0175 * PI / 180.0;
    constexpr double accel_scale = 0.000122 * 9.80665;
    imu.angular_velocity.x = i16(p + 20) * gyro_scale;
    imu.angular_velocity.y = i16(p + 22) * gyro_scale;
    imu.angular_velocity.z = i16(p + 24) * gyro_scale;
    imu.linear_acceleration.x = i16(p + 26) * accel_scale;
    imu.linear_acceleration.y = i16(p + 28) * accel_scale;
    imu.linear_acceleration.z = i16(p + 30) * accel_scale;
    if (!scan_plane_.observe(stamp.nanoseconds(), imu.linear_acceleration.x,
        imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.x,
        imu.angular_velocity.y, imu.angular_velocity.z)) scan_.reset();
    imu_pub_->publish(imu);
  }

  void publish_odom(const std::uint8_t *p, const rclcpp::Time &stamp)
  {
    const auto flags = u32(p + 72);
    if ((flags & 0xc0000001U) != 0xc0000001U) return;
    const double x = i32(p + 16) * 0.001;
    const double y = i32(p + 20) * 0.001;
    const double yaw = i32(p + 24) * 0.001;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odom.twist.twist.linear.x = i32(p + 28) * 0.001;
    odom.twist.twist.linear.y = i32(p + 32) * 0.001;
    odom.twist.twist.angular.z = i32(p + 36) * 0.001;
    for (const auto index : {0, 7, 14, 21, 28, 35}) {
      odom.pose.covariance[index] = 1e6;
      odom.twist.covariance[index] = 1e6;
    }
    const bool error_estimate_layout = (flags & (1U << 14)) != 0;
    for (std::size_t i = 0; i < 3; ++i) {
      const float raw = f32(p + 52 + i * 4);
      if (!std::isfinite(raw) || raw < 0.0F) continue;
      const double value = error_estimate_layout ? std::pow(raw * 0.001, 2) :
        static_cast<double>(raw) * (i < 2 ? 1e-6 : 1.0);
      odom.pose.covariance[i == 0 ? 0 : i == 1 ? 7 : 35] = value;
    }
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(transform);
  }

  void publish_health(const std::uint8_t *p)
  {
    const auto flags = u32(p + 16);
    const auto cpu = u32(p + 124);
    const bool cpu_layout = (cpu >> 16) == 0x4752U;
    const auto calibration = u32(p + 32);
    const auto camera_diag = u32(p + 72);
    const auto camera_guard = u32(p + 76);
    std::ostringstream json;
    json << std::boolalpha
         << "{\"pll_locked\":" << ((flags & 1U) != 0)
         << ",\"camera_initialized\":" << ((flags & 2U) != 0)
         << ",\"ddr_calibrated\":" << ((flags & 4U) != 0)
         << ",\"camera_frame_seen\":" << ((flags & 8U) != 0)
         << ",\"capture_enabled\":" << ((flags & 16U) != 0)
         << ",\"camera_frames\":" << u32(p + 20)
         << ",\"ethernet_packets\":" << u32(p + 24)
         << ",\"video_packets\":" << u32(p + 28)
         << ",\"video_dropped_frames\":" << u32(p + 36)
         << ",\"video_skipped_frames\":" << u32(p + 40)
         << ",\"camera_bad_mod256\":" << ((flags >> 8) & 255U)
         << ",\"pixel_count_bad_mod256\":" << ((flags >> 16) & 255U)
         << ",\"fifo_full_mod256\":" << ((flags >> 24) & 255U)
         << ",\"cpu_health_valid\":" << cpu_layout;
    if (cpu_layout) {
      json << ",\"lidar_uart_bytes\":" << u32(p + 84)
           << ",\"lidar_uart_errors\":" << u32(p + 88)
           << ",\"lidar_packets\":" << u32(p + 92)
           << ",\"lidar_crc_errors\":" << u32(p + 96)
           << ",\"lidar_length_errors\":" << u32(p + 100)
           << ",\"lidar_points\":" << u32(p + 104)
           << ",\"imu_status_bits\":" << (u32(p + 108) & 15U)
           << ",\"imu_identity\":" << (u32(p + 112) & 255U)
           << ",\"imu_samples\":" << u32(p + 116)
           << ",\"imu_errors\":" << u32(p + 120);
      json << ",\"cpu_ready\":" << ((cpu & 1U) != 0)
           << ",\"cpu_fault\":" << ((cpu & 2U) != 0)
           << ",\"cpu_fault_code\":" << ((cpu >> 4) & 255U)
           << ",\"camera_calibration_ready\":" << ((calibration & (1U << 27)) != 0)
           << ",\"camera_calibration_fault\":" << ((calibration & (1U << 26)) != 0)
           << ",\"camera_calibration_state\":" << ((calibration >> 28) & 15U);
      if (cpu & 4U) {
        json << ",\"camera_guard_source_incomplete_seen\":" << ((cpu & (1U << 12)) != 0)
             << ",\"camera_guard_pixel_geometry_seen\":" << ((cpu & (1U << 13)) != 0)
             << ",\"camera_guard_end_timeout_seen\":" << ((cpu & (1U << 14)) != 0)
             << ",\"camera_guard_missing_frame_end_seen\":" << ((cpu & (1U << 15)) != 0)
             << ",\"camera_guard_bad_frames_mod65536\":" <<
                gray16_to_binary(static_cast<std::uint16_t>(camera_guard >> 16))
             << ",\"camera_guard_good_frames_mod65536\":" <<
                gray16_to_binary(static_cast<std::uint16_t>(camera_guard));
      }
    }
    if ((camera_diag >> 28) == 0xcU || (camera_diag >> 28) == 0xdU)
      json << ",\"camera_failure_reason\":" << ((camera_diag >> 24) & 15U)
           << ",\"camera_retry_count\":" << (camera_diag & 255U);
    if ((camera_diag >> 28) == 0xdU)
      json << ",\"camera_source_failure_bits\":" << ((camera_diag >> 18) & 63U)
           << ",\"camera_tap_lane0_div8\":" << ((camera_diag >> 13) & 31U)
           << ",\"camera_tap_lane1_div8\":" << ((camera_diag >> 8) & 31U);
    json << "}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    health_pub_->publish(msg);
  }

  static int open_telemetry_socket(std::uint16_t port)
  {
    return ugv_bridge::open_udp_input(in_addr{htonl(INADDR_ANY)}, port, 4 * 1024 * 1024,
      ugv_bridge::ReceiveBuffer::best_effort, "telemetry");
  }

  void invalidate_socket()
  {
    close(socket_);
    socket_ = -1;
    scan_.reset();
    scan_plane_.reset();
    grid_invalid_ = true;
  }

  void receive_loop(std::uint16_t port)
  {
    std::array<std::uint8_t, 2048> data{};
    auto last_scan_output = std::chrono::steady_clock::now();
    auto last_scan_warning = last_scan_output;
    std::uint64_t lidar_packets = 0, lidar_clock_rejected = 0;
    while (running_) {
      if (socket_ < 0) {
        try {
          socket_ = open_telemetry_socket(port);
          RCLCPP_INFO(get_logger(), "Reopened FPGA telemetry on UDP %u", static_cast<unsigned>(port));
        } catch (const std::system_error &error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Reopening FPGA telemetry: %s", error.what());
          std::this_thread::sleep_for(250ms);
          continue;
        }
      }
      pollfd descriptor{socket_, POLLIN, 0};
      const int result = poll(&descriptor, 1, 50);
      if (result < 0 && errno == EINTR) continue;
      if (const int error = result > 0 ? ugv_bridge::clear_udp_async_error(descriptor) : 0)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Telemetry UDP error: %s", std::strerror(error));
      if (result < 0 || (descriptor.revents & ugv_bridge::UDP_ERROR_EVENTS)) {
        if (result < 0) {
          RCLCPP_ERROR(get_logger(), "Telemetry poll failed: %s", std::strerror(errno));
        } else {
          RCLCPP_ERROR(get_logger(), "Telemetry poll revents=0x%x, reopening socket",
            static_cast<unsigned>(descriptor.revents));
        }
        invalidate_socket();
        continue;
      }
      if (!(descriptor.revents & POLLIN)) continue;
      sockaddr_in peer{};
      socklen_t peer_len = sizeof(peer);
      const auto count = recvfrom(socket_, data.data(), data.size(), MSG_DONTWAIT,
        reinterpret_cast<sockaddr *>(&peer), &peer_len);
      if (count < 0) {
        if (ugv_bridge::retry_udp_receive(errno)) continue;
        RCLCPP_ERROR(get_logger(), "Telemetry receive failed: %s", std::strerror(errno));
        invalidate_socket();
        continue;
      }
      if (count < 16 || (source_addr_ && peer.sin_addr.s_addr != source_addr_->s_addr))
        continue;
      const auto size = static_cast<std::size_t>(count);
      if (std::memcmp(data.data(), "BX72", 4) != 0 || data[4] != 1 ||
          u16(data.data() + 6) != size) continue;
      const auto type = data[5];
      const auto receive_ns = now().nanoseconds();
      if (type == 5 && size == 160) ++lidar_packets;
      try {
        if (type == 8 && size == 84) {
          const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          const auto observed = clock_.observe(u32(data.data() + 36), u32(data.data() + 8),
              u32(data.data() + 16), u32(data.data() + 20), receive_ns, steady_ns);
          if (observed) {
            if (*observed) {
              scan_.reset();
              scan_plane_.reset();
              last_imu_packet_.reset();
            }
            publish_navigation(data.data());
          }
        } else if (type == 7 && size == 128) {
          publish_health(data.data());
        } else if (const auto stamp = clock_.map(u32(data.data() + 12), receive_ns)) {
          if (type == 3 && size == 48) {
            publish_imu(data.data(), *stamp);
          } else if (type == 4 && size == 104) {
            publish_motor(data.data());
          } else if (type == 6 && size == 120) {
            publish_odom(data.data(), *stamp);
            publish_fusion_health(data.data(), *stamp);
          } else if (type == 5 && size == 160) {
            if (!scan_plane_.valid_at(stamp->nanoseconds())) {
              scan_.reset();
              scan_plane_.discard_scan();
            } else if (auto scan = scan_.consume(data.data(), *stamp, laser_frame_, range_min_, range_max_)) {
              scan_plane_.complete_scan();
              scan_pub_->publish(*scan);
              last_scan_output = std::chrono::steady_clock::now();
            }
          } else if (type == 10 && size == 224) {
            if (const auto captured = clock_.map(u32(data.data() + 88), receive_ns))
              receive_grid(data.data(), *captured);
          }
        } else if (type == 5 && size == 160) {
          ++lidar_clock_rejected;
        }
      } catch (const std::exception &error) {
        if (!rclcpp::ok()) break;
        RCLCPP_ERROR(get_logger(), "Telemetry processing failed: %s", error.what());
      }
      const auto checked_at = std::chrono::steady_clock::now();
      if (checked_at - last_scan_output > 5s && checked_at - last_scan_warning > 5s) {
        const auto &d = scan_.diagnostics();
        RCLCPP_WARN(get_logger(),
          "No LaserScan for %.1fs: type5=%llu clock_rejected=%llu consumed=%llu "
          "sequence_gaps=%llu angle_gaps=%llu order=%llu format=%llu time=%llu sparse=%llu",
          std::chrono::duration<double>(checked_at - last_scan_output).count(),
          static_cast<unsigned long long>(lidar_packets), static_cast<unsigned long long>(lidar_clock_rejected),
          static_cast<unsigned long long>(d.packets), static_cast<unsigned long long>(d.sequence_gaps),
          static_cast<unsigned long long>(d.angle_gaps), static_cast<unsigned long long>(d.rejected_order),
          static_cast<unsigned long long>(d.rejected_format), static_cast<unsigned long long>(d.rejected_time),
          static_cast<unsigned long long>(d.rejected_sparse));
        last_scan_warning = checked_at;
      }
    }
  }

  int socket_ = -1;
  std::optional<in_addr> source_addr_;
  std::atomic<bool> running_{true};
  std::thread receiver_;
  std::string laser_frame_, imu_frame_, odom_frame_, base_frame_;
  float range_min_ = 0.05F, range_max_ = 12.0F;
  ClockMapper clock_;
  FrontScan scan_;
  ScanPlane scan_plane_;
  std::uint32_t grid_boot_ = 0, grid_domain_ = 0, grid_id_ = 0;
  bool grid_active_ = false;
  bool grid_invalid_ = false, grid_completed_ = false;
  std::optional<std::uint64_t> last_imu_packet_;
  std::uint8_t grid_seen_ = 0;
  std::array<std::uint8_t, 76> grid_header_{};
  std::array<std::uint8_t, 1024> grid_cells_{};
  std::chrono::steady_clock::time_point grid_at_{};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fusion_health_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GatewayNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_gateway"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
