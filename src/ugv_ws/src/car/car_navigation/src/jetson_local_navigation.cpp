// Jetson 局部地图及航点执行。/scan 缺少逐束时间，使用整圈起始位姿近似。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <ugv_bridge/msg/authorized_twist.hpp>
#include <ugv_bridge/srv/navigation_command.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
#include "navigation.h"
}

namespace {
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
constexpr double kPi = 3.14159265358979323846;
std::uint32_t session_boot = 0, session_domain = 0, clock_base = 0;
Clock::time_point clock_anchor;
bool clock_valid = false;

std::uint64_t session_token() {
  return (static_cast<std::uint64_t>(session_boot) << 32) | session_domain;
}
std::uint32_t device_now() {
  if (!clock_valid) return 0;
  return clock_base + static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - clock_anchor).count());
}
bool u32(const Json& object, const char* key, std::uint32_t& out) {
  if (!object.contains(key) || !object[key].is_number_unsigned()) return false;
  const auto value = object[key].get<std::uint64_t>();
  if (value > std::numeric_limits<std::uint32_t>::max()) return false;
  out = static_cast<std::uint32_t>(value);
  return true;
}
double wrap(double value) { return std::atan2(std::sin(value), std::cos(value)); }
bool rounded_i32(double value, double scale, std::int32_t& output) {
  const double converted = value * scale;
  if (!std::isfinite(converted) || converted < std::numeric_limits<std::int32_t>::min() ||
      converted > std::numeric_limits<std::int32_t>::max()) return false;
  output = static_cast<std::int32_t>(std::llround(converted));
  return true;
}
std::int64_t ns(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

extern "C" std::uint32_t bx72_jetson_session_read(std::uint32_t offset) {
  if (offset == BX72_IO_BOOT_ID) return session_boot;
  if (offset == BX72_IO_ODOM_DOMAIN) return session_domain;
  if (offset == BX72_IO_SESSION_LO) return static_cast<std::uint32_t>(session_token());
  if (offset == BX72_IO_SESSION_HI) return static_cast<std::uint32_t>(session_token() >> 32);
  return 0;
}
extern "C" std::uint32_t bx72_jetson_live_time(void) { return device_now(); }

class LocalNavigation final : public rclcpp::Node {
#ifdef BX72_NAV_COMMAND_TEST
  friend class JetsonNavigationCommandHarness;
#endif
  using Command = ugv_bridge::srv::NavigationCommand;
  struct PoseRecord { std::int64_t ros_ns; bx72_pose_t pose; };
  struct FusionRecord { bx72_pose_t pose{}; std::uint32_t accepted = 0; bool valid = false; };

 public:
  LocalNavigation() : Node("jetson_local_navigation") {
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame_id", "n10_sensor");
    const double lidar_x = declare_parameter<double>("lidar_x_m", 0.13);
    const double lidar_y = declare_parameter<double>("lidar_y_m", 0.0);
    const double lidar_yaw = declare_parameter<double>("lidar_yaw_rad", 0.0);
    if (odom_frame_.empty() || base_frame_.empty() || lidar_frame_.empty() ||
        !std::isfinite(lidar_x) || !std::isfinite(lidar_y) || !std::isfinite(lidar_yaw) ||
        std::abs(lidar_x - 0.13) > 1e-6 || std::abs(lidar_y) > 1e-6 || std::abs(lidar_yaw) > 1e-6)
      throw std::invalid_argument("共享栅格当前要求雷达相对车体 x=0.13m、y=0、yaw=0");

    const auto sensor_qos = rclcpp::SensorDataQoS();
    nav_sub_ = create_subscription<std_msgs::msg::String>("/fpga/round5/navigation", sensor_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { receive_navigation(*msg); });
    fusion_sub_ = create_subscription<std_msgs::msg::String>("/fpga/round5/fusion", sensor_qos,
        [this](std_msgs::msg::String::ConstSharedPtr msg) { receive_fusion(*msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { receive_odom(*msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", sensor_qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { receive_scan(*msg); });
    control_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/fpga/bidirectional_status", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { receive_control_status(*msg); });
    state_pub_ = create_publisher<std_msgs::msg::String>("/jetson/navigation/state", 10);
    grid_pub_ = create_publisher<std_msgs::msg::String>("/jetson/navigation/local_grid", 10);
    plan_pub_ = create_publisher<std_msgs::msg::String>("/jetson/navigation/local_plan", 10);
    velocity_pub_ = create_publisher<ugv_bridge::msg::AuthorizedTwist>("/jetson/navigation/cmd_vel", 10);
    command_service_ = create_service<Command>("/jetson/navigation/command",
        [this](const std::shared_ptr<Command::Request> request,
               std::shared_ptr<Command::Response> response) { command(*request, *response); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    bx72_nav_init();
  }

 private:
  std::string odom_frame_, base_frame_, lidar_frame_, bridge_id_;
  std::uint64_t authorization_epoch_ = 0;
  // Persists across FPGA domain resets so delayed requests cannot cross them.
  std::uint64_t last_applied_issued_ns_ = 0;
  std::uint32_t snapshot_id_ = 0, plan_sequence_ = 0;
  std::uint32_t grid_epoch_base_ = 0, reported_task_ = 0;
  Clock::time_point nav_at_{}, fusion_at_{}, scan_at_{}, pose_at_{};
  Clock::time_point negative_clock_at_{};
  std::uint32_t last_nav_source_us_ = 0, negative_clock_source_us_ = 0;
  unsigned negative_clock_samples_ = 0;
  bool have_nav_source_ = false;
  bool external_control_active_ = false;
  std::int64_t latest_control_status_ns_ = 0, latest_start_ns_ = 0;
  // 仅在 FPGA 自身未提供局部规划时启用此兼容执行器。
  bool local_execution_enabled_ = false;
  bool have_nav_ = false, have_scan_ = false, have_pose_ = false, scan_gap_ = false, guard_stop_ = false;
  bool nav_pose_valid_ = false, fusion_pose_valid_ = false;
  std::int64_t last_scan_ros_ns_ = 0;
  bx72_pose_t pose_{};
  bx72_nav_output_t output_{};
  FusionRecord fusion_{};
  std::deque<PoseRecord> history_;
  std::uint32_t terminal_status_ = 0, terminal_task_ = 0, terminal_last_task_ = 0;
  bool have_terminal_ = false;
  std::uint64_t tick_count_ = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_sub_, fusion_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr control_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, grid_pub_, plan_pub_;
  rclcpp::Publisher<ugv_bridge::msg::AuthorizedTwist>::SharedPtr velocity_pub_;
  rclcpp::Service<Command>::SharedPtr command_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool fresh(Clock::time_point stamp, double seconds) const {
    return stamp != Clock::time_point{} &&
        std::chrono::duration<double>(Clock::now() - stamp).count() <= seconds;
  }
  void receive_control_status(const diagnostic_msgs::msg::DiagnosticArray& message) {
    if (!local_execution_enabled_ || !clock_valid) return;
    const auto stamp = ns(message.header.stamp);
    if (stamp <= latest_control_status_ns_ || stamp < latest_start_ns_) return;
    for (const auto& status : message.status) {
      if (status.name != "bx72_udp") continue;
      for (const auto& field : status.values) {
        if (field.key != "control_source") continue;
        latest_control_status_ns_ = stamp;
        const bool external = field.value == "pc" || field.value == "ros";
        if (external && !external_control_active_) {
          if (output_.autonomous) force_cancel();
          bx72_nav_external_takeover(device_now());
        }
        external_control_active_ = external;
        return;
      }
    }
  }
  void publish_velocity(int vx, int vy, int wz) {
    if (bridge_id_.empty() || authorization_epoch_ == 0) return;
    ugv_bridge::msg::AuthorizedTwist msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.bridge_id = bridge_id_;
    msg.authorization_epoch = authorization_epoch_;
    msg.twist.linear.x = vx * 0.001;
    msg.twist.linear.y = vy * 0.001;
    msg.twist.angular.z = wz * 0.001;
    velocity_pub_->publish(msg);
  }
  void invalidate_pose() {
    // 融合输出无效后，不能继续把上一帧 odom 当成当前有效位姿。
    if (output_.autonomous) publish_velocity(0, 0, 0);
    have_pose_ = false;
    history_.clear();
  }
  void reset_domain(std::uint32_t boot, std::uint32_t domain, std::uint32_t source_now) {
    if (output_.autonomous) publish_velocity(0, 0, 0);
    session_boot = boot;
    session_domain = domain;
    clock_base = source_now;
    clock_anchor = Clock::now();
    clock_valid = boot != 0 && domain != 0;
    ++grid_epoch_base_;
    bx72_nav_init();
    output_ = {};
    output_.task_mode = 1;
    history_.clear();
    fusion_ = {};
    have_scan_ = have_pose_ = false;
    nav_pose_valid_ = fusion_pose_valid_ = false;
    have_nav_source_ = false;
    external_control_active_ = false;
    latest_control_status_ns_ = latest_start_ns_ = 0;
    negative_clock_samples_ = 0;
    last_scan_ros_ns_ = 0;
    scan_gap_ = false;
    guard_stop_ = false;
    have_terminal_ = false;
    reported_task_ = 0;
    bridge_id_.clear();
    authorization_epoch_ = 0;
  }
  void receive_navigation(const std_msgs::msg::String& msg) {
    const auto data = Json::parse(msg.data, nullptr, false);
    std::uint32_t boot, domain, source_now;
    if (!data.is_object() || !u32(data, "boot_id", boot) || !u32(data, "odom_domain", domain) ||
        !u32(data, "now_us", source_now) || data.value("rx_rejection", false)) return;
    if (boot == 0 || domain == 0) {
      // An invalid initial domain has nothing to reset; clear an old domain once.
      if (clock_valid) reset_domain(boot, domain, source_now);
      local_execution_enabled_ = false;
      have_nav_ = false;
      return;
    }
    const bool use_local = data.value("protocol", std::string{}) == "velocity_only_v1";
    if (!use_local) {
      if (local_execution_enabled_) reset_domain(0, 0, 0);
      local_execution_enabled_ = false;
      have_nav_ = false;
      return;
    }
    if (!local_execution_enabled_ || !clock_valid || boot != session_boot || domain != session_domain) {
      reset_domain(boot, domain, source_now);
      local_execution_enabled_ = true;
    } else {
      // Reject replay before considering a source clock correction.
      if (have_nav_source_ && static_cast<std::int32_t>(source_now - last_nav_source_us_) <= 0) return;
      const auto drift = static_cast<std::int32_t>(source_now - device_now());
      if (drift < -250000) {
        // One delayed frame is not a clock reset. Re-anchor only after three
        // increasing samples advance at approximately host-clock speed.
        const auto now = Clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            now - negative_clock_at_).count();
        const auto source_step = static_cast<std::uint32_t>(source_now - negative_clock_source_us_);
        if (negative_clock_samples_ && elapsed > 0 && elapsed <= 500000 &&
            source_step != 0 && source_step < 0x80000000u &&
            std::abs(static_cast<std::int64_t>(source_step) - elapsed) <= 50000)
          ++negative_clock_samples_;
        else
          negative_clock_samples_ = 1;
        negative_clock_at_ = now;
        negative_clock_source_us_ = source_now;
        if (negative_clock_samples_ < 3) return;
        reset_domain(boot, domain, source_now);
      } else {
        negative_clock_samples_ = 0;
        if (drift > 250000) reset_domain(boot, domain, source_now);
      }
    }
    last_nav_source_us_ = source_now;
    have_nav_source_ = true;
    have_nav_ = clock_valid;
    nav_at_ = Clock::now();
    nav_pose_valid_ = data.contains("pose_valid") && data["pose_valid"].is_boolean() &&
        data["pose_valid"].get<bool>();
    if (!nav_pose_valid_) invalidate_pose();
    // 新版为固件同款去 hard-kill 的 guard 反馈；旧编码 0x4e 可能被其他故障覆盖。
    guard_stop_ = data.contains("guard_stop") && data["guard_stop"].is_boolean() ?
        data["guard_stop"].get<bool>() : data.value("guard_error", 0) == 14;
    if (guard_stop_ && output_.autonomous) publish_velocity(0, 0, 0);
  }
  bool ros_stamp_to_device(std::int64_t stamp_ns, std::uint32_t& stamp) const {
    if (!clock_valid || stamp_ns <= 0) return false;
    const auto age = now().nanoseconds() - stamp_ns;
    if (age < -20000000LL || age > 1500000000LL) return false;
    stamp = device_now() - static_cast<std::uint32_t>(std::max<std::int64_t>(0, age) / 1000);
    return true;
  }
  void receive_odom(const nav_msgs::msg::Odometry& msg) {
    if (!clock_valid || msg.header.frame_id != odom_frame_ || msg.child_frame_id != base_frame_) return;
    const auto stamp_ns = ns(msg.header.stamp);
    std::uint32_t stamp;
    if (!ros_stamp_to_device(stamp_ns, stamp)) return;
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    const auto& v = msg.twist.twist;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) return;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (norm < 0.99 || norm > 1.01) return;
    const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
    bx72_pose_t next{};
    if (!rounded_i32(p.x, 1000, next.x_mm) || !rounded_i32(p.y, 1000, next.y_mm) ||
        !rounded_i32(yaw, 1000, next.yaw_mrad) || !rounded_i32(v.linear.x, 1000, next.vx_mm_s) ||
        !rounded_i32(v.linear.y, 1000, next.vy_mm_s) || !rounded_i32(v.angular.z, 1000, next.wz_mrad_s)) return;
    next.stamp_us = stamp;
    next.flags = BX72_POSE_VALID;
    pose_ = next;
    have_pose_ = true;
    pose_at_ = Clock::now();
  }
  bool pose_at(std::int64_t stamp_ns, bx72_pose_t& result) const {
    if (history_.empty()) return false;
    const auto closest = std::min_element(history_.begin(), history_.end(),
        [stamp_ns](const PoseRecord& a, const PoseRecord& b) {
          return std::abs(a.ros_ns - stamp_ns) < std::abs(b.ros_ns - stamp_ns);
        });
    if (std::abs(closest->ros_ns - stamp_ns) > 100000000LL) return false;
    result = closest->pose;
    return true;
  }
  void receive_scan(const sensor_msgs::msg::LaserScan& scan) {
    if (!clock_valid || scan.header.frame_id != lidar_frame_ || scan.ranges.empty() ||
        !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0 ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) || scan.range_max <= scan.range_min) return;
    const auto stamp_ns = ns(scan.header.stamp);
    std::uint32_t stamp;
    bx72_pose_t scan_pose{};
    if (stamp_ns <= last_scan_ros_ns_ || !ros_stamp_to_device(stamp_ns, stamp) ||
        static_cast<std::uint32_t>(device_now() - stamp) > 500000u ||
        !pose_at(stamp_ns, scan_pose)) {
      bx72_nav_lidar_gap(device_now(), BX72_NAV_GAP_POSE);
      return;
    }
    std::array<std::uint16_t, 72> ranges{};
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double distance = scan.ranges[i];
      if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max ||
          distance < 0.05 || distance > 8.0) continue;
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double clockwise = std::fmod(2 * kPi - std::fmod(angle, 2 * kPi) + 2 * kPi, 2 * kPi);
      const unsigned bin = static_cast<unsigned>(std::llround(clockwise * 36.0 / kPi)) % 72;
      const auto mm = static_cast<std::uint16_t>(std::lround(distance * 1000));
      if (ranges[bin] == 0 || mm < ranges[bin]) ranges[bin] = mm;
    }
    const auto live = device_now();
    // 一圈数据共用起始位姿和起始时标；这不是逐束运动补偿。
    for (unsigned bin = 0; bin < ranges.size(); ++bin)
      bx72_nav_lidar_validated(bin * 500u, ranges[bin], ranges[bin] != 0,
                               &scan_pose, stamp, live);
    last_scan_ros_ns_ = stamp_ns;
    scan_at_ = Clock::now();
    have_scan_ = true;
    scan_gap_ = false;
  }
  void receive_fusion(const std_msgs::msg::String& msg) {
    if (!clock_valid) return;
    const auto data = Json::parse(msg.data, nullptr, false);
    if (!data.is_object()) return;
    if (!data.contains("pose_valid") || !data["pose_valid"].is_boolean() ||
        !data["pose_valid"].get<bool>()) {
      fusion_pose_valid_ = false;
      invalidate_pose();
      return;
    }
    if (!data.contains("state") || !data["state"].is_object()) return;
    std::uint32_t stamp, accepted;
    if (!u32(data, "pose_stamp_us", stamp) ||
        !(u32(data, "scan_match_accepted", accepted) || u32(data, "csm_accepted", accepted))) return;
    const auto source_age_us = static_cast<std::int32_t>(device_now() - stamp);
    if (source_age_us < -20000 || source_age_us > 500000) return;
    const auto age_us = std::max<std::int32_t>(0, source_age_us);
    const auto& s = data["state"];
    bx72_pose_t current{};
    try {
      if (!rounded_i32(s.at("x_m").get<double>(), 1000, current.x_mm) ||
          !rounded_i32(s.at("y_m").get<double>(), 1000, current.y_mm) ||
          !rounded_i32(s.at("yaw_rad").get<double>(), 1000, current.yaw_mrad) ||
          !rounded_i32(s.at("vx_m_s").get<double>(), 1000, current.vx_mm_s) ||
          !rounded_i32(s.at("vy_m_s").get<double>(), 1000, current.vy_mm_s) ||
          !rounded_i32(s.at("wz_rad_s").get<double>(), 1000, current.wz_mrad_s)) return;
    } catch (const Json::exception&) { return; }
    fusion_pose_valid_ = true;
    fusion_at_ = Clock::now();
    current.flags = BX72_POSE_VALID;
    current.stamp_us = stamp;
    if (fusion_.valid && accepted != fusion_.accepted) {
      const auto dt_us = static_cast<std::uint32_t>(stamp - fusion_.pose.stamp_us);
      if (accepted - fusion_.accepted != 1u || dt_us == 0 || dt_us > 200000u) {
        // 无法估计同一时刻修正前位姿时，废弃旧图和活动任务。
        if (output_.autonomous) publish_velocity(0, 0, 0);
        bx72_nav_map_reset(&current, device_now());
        history_.clear();
        have_pose_ = have_scan_ = false;
      } else {
        const double dt = dt_us * 1e-6;
        const double yaw = fusion_.pose.yaw_mrad * 0.001;
        bx72_pose_t old = fusion_.pose;
        old.x_mm += static_cast<std::int32_t>(std::lround(
            (std::cos(yaw) * old.vx_mm_s - std::sin(yaw) * old.vy_mm_s) * dt));
        old.y_mm += static_cast<std::int32_t>(std::lround(
            (std::sin(yaw) * old.vx_mm_s + std::cos(yaw) * old.vy_mm_s) * dt));
        old.yaw_mrad = static_cast<std::int32_t>(std::lround(wrap(yaw + old.wz_mrad_s * 0.001 * dt) * 1000));
        old.stamp_us = stamp;
        bx72_nav_rebase(&old, &current);
        const double delta = wrap((current.yaw_mrad - old.yaw_mrad) * 0.001);
        const double c = std::cos(delta), sn = std::sin(delta);
        for (auto& entry : history_) {
          const double dx = entry.pose.x_mm - old.x_mm, dy = entry.pose.y_mm - old.y_mm;
          entry.pose.x_mm = current.x_mm + static_cast<std::int32_t>(std::lround(c * dx - sn * dy));
          entry.pose.y_mm = current.y_mm + static_cast<std::int32_t>(std::lround(sn * dx + c * dy));
          entry.pose.yaw_mrad = static_cast<std::int32_t>(std::lround(
              wrap(entry.pose.yaw_mrad * 0.001 + delta) * 1000));
        }
        if (have_pose_) pose_ = current;
      }
    }
    fusion_ = {current, accepted, true};
    // 原始融合包按源时间追加；只修正此前历史，避免 ROS 跨话题到达顺序造成重复修正。
    const auto ros_stamp = now().nanoseconds() - static_cast<std::int64_t>(age_us) * 1000;
    if (history_.empty() || ros_stamp > history_.back().ros_ns) {
      history_.push_back({ros_stamp, current});
      while (!history_.empty() && ros_stamp - history_.front().ros_ns > 2000000000LL)
        history_.pop_front();
    }
  }

  std::uint32_t sensor_status() const {
    std::uint32_t status = BX72_STATUS_AUTO;
    // 旧控制接口需要两个位；它们代表 FPGA 发布的聚合融合有效性，
    // 并非 Jetson 独立诊断出的 IMU 和轮速硬件状态。
    if (have_pose_ && fresh(pose_at_, 0.5) && nav_pose_valid_ && fusion_pose_valid_ &&
        fresh(nav_at_, 0.5) && fresh(fusion_at_, 0.5))
      status |= BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
    if (have_scan_ && fresh(scan_at_, 0.8)) status |= BX72_STATUS_LIDAR_OK;
    if (!have_nav_ || !fresh(nav_at_, 0.5)) status |= BX72_STATUS_LINK_DOWN;
    if (guard_stop_) status |= BX72_STATUS_GUARD_STOP;
    return status;
  }
  void run_control(std::uint32_t now_us) {
    bx72_nav_control(have_pose_ ? &pose_ : nullptr, now_us, sensor_status(), 0, 0, 0, 0, 0, &output_);
  }
  void force_cancel() {
    if (!clock_valid || !output_.autonomous) return;
    publish_velocity(0, 0, 0);
    const auto previous = reported_task_;
    bx72_nav_force_cancel(device_now());
    run_control(device_now());
    have_terminal_ = true;
    terminal_status_ = BX72_NAV_STATUS_CANCELLED;
    terminal_task_ = previous;
    terminal_last_task_ = output_.last_task_id;
  }
  void command(const Command::Request& req, Command::Response& response) {
    response.success = false;
    const auto now_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
    if (req.issued_steady_ns == 0 || req.issued_steady_ns > static_cast<std::uint64_t>(now_steady_ns)) {
      response.message = "导航请求发送时刻无效";
      return;
    }
    if (req.issued_steady_ns <= last_applied_issued_ns_) {
      // An old force-cancel is already superseded by a newer operation.
      response.success = req.action == 3 && req.task_id == 0;
      response.message = "导航请求已被较新的操作取代";
      return;
    }
    if (req.action == 3 && req.task_id == 0) {
      last_applied_issued_ns_ = req.issued_steady_ns;
      force_cancel();
      response.success = true;
      response.message = "旧任务已清除";
      return;
    }
    if (!clock_valid || !have_nav_ || !fresh(nav_at_, 0.5) || req.boot_id != session_boot ||
        req.odom_domain != session_domain || req.task_id == 0 || req.action < 1 || req.action > 3) {
      response.message = "会话、坐标域或任务编号无效";
      return;
    }
    bx72_nav_task_request_t task{};
    bool replace_existing = false;
    if (req.action == 1) {
      if (req.start_deadline_steady_ns == 0 ||
          static_cast<std::uint64_t>(now_steady_ns) >= req.start_deadline_steady_ns) {
        response.message = "导航启动请求已过期";
        return;
      }
      if (req.bridge_id.empty() || req.authorization_epoch == 0 ||
          (req.mode != 2 && req.mode != 3) || (req.flags & ~3u) ||
          !rounded_i32(req.goal_x_m, 65536, task.goal_x_q16) ||
          !rounded_i32(req.goal_y_m, 65536, task.goal_y_q16) ||
          !std::isfinite(req.hold_yaw_rad) || std::abs(req.hold_yaw_rad) > kPi ||
          !std::isfinite(req.final_yaw_rad) || std::abs(req.final_yaw_rad) > kPi ||
          !rounded_i32(req.hold_yaw_rad, 65536, task.hold_yaw_q16) ||
          !rounded_i32(req.final_yaw_rad, 65536, task.final_yaw_q16) ||
          req.max_speed_mm_s == 0 || req.max_speed_mm_s > 400 ||
          req.max_accel_mm_s2 == 0 || req.max_accel_mm_s2 > 1000 ||
          req.tolerance_mm == 0 || req.tolerance_mm > 500 ||
          req.yaw_tolerance_mrad == 0 || req.yaw_tolerance_mrad > 1000 ||
          !std::isfinite(req.duration_s) || req.duration_s <= 0 || req.duration_s > 120 ||
          std::llround(req.duration_s * 1000000.0) == 0) {
        response.message = "任务参数或控制来源无效";
        return;
      }
      replace_existing = output_.autonomous && req.bridge_id != bridge_id_;
      if (output_.autonomous && !replace_existing) {
        response.message = "已有导航任务正在运行";
        return;
      }
    } else {
      if (req.flags & ~3u) {
        response.message = "任务参数或控制来源无效";
        return;
      }
    }
    const auto previous_task = reported_task_;
    const auto stamp = device_now();
    task.action = req.action;
    task.mode = req.mode >= 2 && req.mode <= 3 ? req.mode : 2;
    task.flags = req.flags;
    task.task_id = req.task_id;
    task.max_speed_mm_s = req.max_speed_mm_s;
    task.max_accel_mm_s2 = req.max_accel_mm_s2;
    task.tolerance_mm = req.tolerance_mm;
    task.yaw_tolerance_mrad = req.yaw_tolerance_mrad;
    const auto lifetime = req.action == 1 ?
        static_cast<std::uint32_t>(std::llround(req.duration_s * 1000000.0)) : 1000000u;
    task.sent_at_us = stamp;
    task.expiry_us = stamp + lifetime;
    const bool accepted = bx72_nav_submit_task(&task, stamp, replace_existing ? 1 : 0) != 0;
    if (accepted && (replace_existing || req.action != 1)) publish_velocity(0, 0, 0);
    run_control(device_now());
    if (!accepted) {
      response.message = "任务编号或时间被局部规划器拒绝";
      return;
    }
    last_applied_issued_ns_ = req.issued_steady_ns;
    if (req.action == 1) {
      bridge_id_ = req.bridge_id;
      authorization_epoch_ = req.authorization_epoch;
      latest_start_ns_ = now().nanoseconds();
      external_control_active_ = false;
      have_terminal_ = false;
      reported_task_ = req.task_id;
    } else {
      have_terminal_ = true;
      terminal_status_ = BX72_NAV_STATUS_CANCELLED;
      terminal_task_ = previous_task;
      terminal_last_task_ = req.task_id;
    }
    response.success = true;
    response.message = "任务已接收，执行结果以状态反馈为准";
  }

  void publish_json(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher, const Json& value) {
    std_msgs::msg::String message;
    message.data = value.dump();
    publisher->publish(message);
  }
  void publish_state() {
    const bool active = output_.autonomous != 0;
    std::uint32_t status = output_.status, task = output_.task_id, last = output_.last_task_id;
    if (!active && have_terminal_) {
      status = terminal_status_; task = terminal_task_; last = terminal_last_task_;
    } else if (active && status == BX72_NAV_STATUS_STALE) status = BX72_NAV_STATUS_BLOCKED;
    publish_json(state_pub_, {{"bridge_id", bridge_id_}, {"authorization_epoch", authorization_epoch_},
        {"boot_id", session_boot}, {"odom_domain", session_domain}, {"task_id", task},
        {"last_task_id", last}, {"status", status}, {"mode", active ? output_.task_mode : 1},
        {"progress", output_.progress}, {"active_task", active}});
  }
  void publish_grid() {
    if (!clock_valid) return;
    bx72_nav_grid_snapshot_t snapshot{};
    std::array<std::uint8_t, BX72_GRID_EXPORT_BYTES> cells{};
    const auto begin = device_now();
    if (!bx72_nav_grid_snapshot(&snapshot, cells.data(), begin + 50000u)) return;
    const auto end = device_now();
    ++snapshot_id_;
    publish_json(grid_pub_, {{"type", "local_grid"}, {"protocol", "jetson_grid_snapshot_v1"},
        {"source", "jetson"}, {"sequence", snapshot_id_}, {"time_us", end},
        {"boot_id", snapshot.boot_id}, {"odom_domain", snapshot.odom_domain},
        {"snapshot_id", snapshot_id_}, {"capture_begin_us", begin}, {"capture_end_us", end},
        {"grid_generation", snapshot.generation}, {"scan_generation", snapshot.scan_generation},
        {"complete_scan_stamp_us", snapshot.complete_scan_stamp_us}, {"last_ray_stamp_us", snapshot.last_ray_stamp_us},
        {"grid_frame_epoch", snapshot.grid_frame_epoch + grid_epoch_base_},
        {"transform_revision", snapshot.transform_revision}, {"flags", snapshot.flags},
        {"width", 64}, {"height", 64}, {"resolution_mm", 50},
        {"origin_x_cell", snapshot.origin_x_cell}, {"origin_y_cell", snapshot.origin_y_cell},
        {"grid_from_odom", {{"map_anchor_x_mm", snapshot.map_anchor_x_mm},
                            {"map_anchor_y_mm", snapshot.map_anchor_y_mm},
                            {"odom_anchor_x_mm", snapshot.odom_anchor_x_mm},
                            {"odom_anchor_y_mm", snapshot.odom_anchor_y_mm},
                            {"yaw_offset_mrad", snapshot.map_yaw_offset_mrad}}},
        {"packed_cells", cells}});
  }
  void publish_plan() {
    const auto stamp = device_now();
    publish_json(plan_pub_, {{"type", "local_plan"}, {"protocol", "jetson_local_segment_v1"},
        {"source", "jetson"}, {"sequence", ++plan_sequence_}, {"time_us", stamp},
        {"boot_id", session_boot}, {"odom_domain", session_domain}, {"task_id", output_.task_id},
        {"plan_stamp_us", output_.plan_stamp_us}, {"valid", output_.plan_valid != 0},
        {"guard_stop", guard_stop_}, {"grid_direction_preview", true}, {"map_version", output_.map_version},
        {"frame_id", "odom"},
        {"points", Json::array({{{"x", output_.plan_start_x_mm * .001}, {"y", output_.plan_start_y_mm * .001}},
                                {{"x", output_.plan_end_x_mm * .001}, {"y", output_.plan_end_y_mm * .001}}})}});
  }
  void tick() {
    if (!local_execution_enabled_) return;
    // 控制输出优先，同一 20 ms 拍中后台规划总预算为 12 ms。
    const auto planning_deadline = Clock::now() + std::chrono::milliseconds(12);
    ++tick_count_;
    if (clock_valid) {
      const auto stamp = device_now();
      if (have_scan_ && !fresh(scan_at_, 0.8) && !scan_gap_) {
        bx72_nav_lidar_gap(stamp, BX72_NAV_GAP_STALE);
        scan_gap_ = true;
      }
      run_control(stamp);
      if (!output_.autonomous && output_.status >= BX72_NAV_STATUS_ARRIVED &&
          output_.status <= BX72_NAV_STATUS_STALE && !have_terminal_) {
        have_terminal_ = true;
        terminal_status_ = output_.status;
        terminal_task_ = output_.task_id;
        terminal_last_task_ = output_.last_task_id;
      }
      if (output_.autonomous) {
        reported_task_ = output_.task_id;
        publish_velocity(output_.vx_mm_s, output_.vy_mm_s, output_.wz_mrad_s);
      } else if (!bridge_id_.empty()) publish_velocity(0, 0, 0);
      const auto required_pose = BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
      const bool valid_pose = (sensor_status() & required_pose) == required_pose;
      for (unsigned i = 0; i < 8 && Clock::now() + std::chrono::milliseconds(4) < planning_deadline; ++i)
        bx72_nav_step(valid_pose ? &pose_ : nullptr, device_now());
    }
    if (tick_count_ % 5 == 0) { publish_state(); if (clock_valid) publish_plan(); }
    if (tick_count_ % 10 == 0) publish_grid();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<LocalNavigation>()); }
  catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("jetson_local_navigation"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
