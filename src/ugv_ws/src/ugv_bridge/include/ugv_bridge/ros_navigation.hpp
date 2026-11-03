#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ugv_bridge::ros_navigation {

// 雷达相对车体中心前移约130mm；须与FPGA和部署配置一致。
inline constexpr double kDefaultN10OffsetXMetres = 0.13;

// A stale Jetson executor must be stopped before a Jetson start; an FPGA
// executor's independently pending start does not depend on that RPC.
inline constexpr bool failed_jetson_cancel_blocks_start(bool jetson_backend, bool pending_start) {
  return jetson_backend && pending_start;
}

inline void set_yaw(geometry_msgs::msg::Quaternion& orientation, double yaw_rad);
inline void set_unknown_covariance(std::array<double, 36>& covariance);

// 位姿误差估计单位为毫米和毫弧度；平方换算仅用于ROS显示，不是完整EKF协方差。
inline std::optional<std::array<double, 7>> estimate_covariance(const std::array<double, 3>& estimate) {
  std::array<double, 7> covariance;
  covariance.fill(1e6);
  for (std::size_t index=0;index<estimate.size();++index) {
    if (!std::isfinite(estimate[index])) return std::nullopt;
    const auto si=estimate[index]*0.001;
    covariance[index]=si*si;
  }
  return covariance;
}

struct N10Point {
  std::uint16_t angle_cdeg = 0;
  std::uint16_t distance_mm = 0xffff;
  std::uint8_t strength = 0;
};

struct N10Group {
  std::uint32_t sequence = 0;
  std::uint32_t timestamp_us = 0;
  std::uint32_t dropped = 0;
  std::uint32_t crc_errors = 0;
  std::uint32_t length_errors = 0;
  std::array<N10Point, 16> points{};
};

// 导航流发送32位模微秒时钟；接收端用无符号差扩展回单调64位，再映射到ROS时钟。
// 旧32位传感器时标以同一锚点展开。映射误差包含UDP延迟，不是硬件时钟同步。
class FpgaRosTimeMapper {
 public:
  bool observe_navigation(std::uint32_t fpga_us32, std::int64_t ros_receive_ns) {
    if (ros_receive_ns < 0) {
      return false;
    }
    if (!valid_) {
      anchor_fpga_us_ = fpga_us32;
      last_fpga_us_ = fpga_us32;
      last_navigation_us32_ = fpga_us32;
      const auto source_ns = static_cast<std::int64_t>(anchor_fpga_us_ * 1000ULL);
      offset_ns_ = ros_receive_ns - source_ns;
      last_ros_receive_ns_ = ros_receive_ns;
      valid_ = true;
      return true;
    }

    const auto delta_us = static_cast<std::uint32_t>(fpga_us32 - last_navigation_us32_);
    if (delta_us >= 0x80000000U && delta_us != 0) return false;
    const auto extended_us = last_fpga_us_ + delta_us;
    if (extended_us > static_cast<std::uint64_t>(
            std::numeric_limits<std::int64_t>::max() / 1000)) {
      return false;
    }
    const auto source_ns = static_cast<std::int64_t>(extended_us * 1000ULL);
    const auto sample_offset = ros_receive_ns - source_ns;
    if (ros_receive_ns < last_ros_receive_ns_) {
      offset_ns_ = sample_offset;
    } else if (delta_us == 0) {
      last_ros_receive_ns_ = ros_receive_ns;
      return true;
    } else {
      // 跟踪缓慢时钟漂移，并限制大部分到包抖动的影响。
      const auto residual = sample_offset - offset_ns_;
      auto correction = residual / 16;
      if (correction > 10000) correction = 10000;
      if (correction < -10000) correction = -10000;
      offset_ns_ += correction;
    }
    anchor_fpga_us_ = extended_us;
    last_fpga_us_ = extended_us;
    last_navigation_us32_ = fpga_us32;
    last_ros_receive_ns_ = ros_receive_ns;
    return true;
  }

  void reset() {
    valid_ = false;
    anchor_fpga_us_ = 0;
    last_fpga_us_ = 0;
    last_navigation_us32_ = 0;
    last_ros_receive_ns_ = 0;
    offset_ns_ = 0;
  }

  bool valid() const { return valid_; }
  std::uint64_t anchor_fpga_us() const { return anchor_fpga_us_; }
  std::int64_t offset_ns() const { return offset_ns_; }

  std::optional<std::int64_t> map_fpga_us(std::uint64_t fpga_us) const {
    if (!valid_ || fpga_us > static_cast<std::uint64_t>(
            std::numeric_limits<std::int64_t>::max() / 1000)) {
      return std::nullopt;
    }
    const auto source_ns = static_cast<std::int64_t>(fpga_us * 1000);
    if ((offset_ns_ > 0 && source_ns > std::numeric_limits<std::int64_t>::max() - offset_ns_) ||
        (offset_ns_ < 0 && source_ns < std::numeric_limits<std::int64_t>::min() - offset_ns_)) {
      return std::nullopt;
    }
    return source_ns + offset_ns_;
  }

  std::optional<std::uint64_t> unwrap_legacy_us(std::uint32_t fpga_us) const {
    if (!valid_) return std::nullopt;
    const auto low = static_cast<std::uint32_t>(anchor_fpga_us_);
    const auto wrapped_delta = static_cast<std::uint32_t>(fpga_us - low);
    const std::int64_t delta = wrapped_delta <= 0x7fffffffU
        ? static_cast<std::int64_t>(wrapped_delta)
        : static_cast<std::int64_t>(wrapped_delta) - 0x100000000LL;
    if (delta < 0 && static_cast<std::uint64_t>(-delta) > anchor_fpga_us_) return std::nullopt;
    return delta < 0 ? anchor_fpga_us_ - static_cast<std::uint64_t>(-delta)
                     : anchor_fpga_us_ + static_cast<std::uint64_t>(delta);
  }

  std::optional<std::int64_t> map_legacy_us(std::uint32_t fpga_us) const {
    const auto unwrapped = unwrap_legacy_us(fpga_us);
    return unwrapped ? map_fpga_us(*unwrapped) : std::nullopt;
  }

 private:
  bool valid_ = false;
  std::uint64_t anchor_fpga_us_ = 0;
  std::uint64_t last_fpga_us_ = 0;
  std::uint32_t last_navigation_us32_ = 0;
  std::int64_t last_ros_receive_ns_ = 0;
  std::int64_t offset_ns_ = 0;
};

class N10ScanAssembler {
 public:
  static constexpr std::size_t kAngleBins = 36000;
  static constexpr std::uint16_t kFullCoverageCdeg = 35000;
  static constexpr std::size_t kMinimumSamples = 64;

  N10ScanAssembler() { reset_scan(); }

  void reset() {
    has_sequence_ = false;
    has_counters_ = false;
    has_angle_ = false;
    saw_first_wrap_ = false;
    have_scan_start_ = false;
    last_angle_ = 0;
    last_sequence_ = 0;
    last_dropped_ = 0;
    last_crc_errors_ = 0;
    last_length_errors_ = 0;
    previous_scan_start_ns_.reset();
    reset_scan();
  }

  std::optional<sensor_msgs::msg::LaserScan> consume(
      const N10Group& group, std::int64_t mapped_group_stamp_ns,
      const std::string& frame_id, double range_min_m = 0.0,
      double range_max_m = 65.534) {
    if (mapped_group_stamp_ns < 0 || frame_id.empty() ||
        !std::isfinite(range_min_m) || !std::isfinite(range_max_m) ||
        range_min_m < 0 || range_max_m <= range_min_m) {
      invalidate_partial();
      return std::nullopt;
    }

    if (has_sequence_ && group.sequence != static_cast<std::uint32_t>(last_sequence_ + 1))
      invalidate_partial();
    if (has_counters_ && (group.dropped != last_dropped_ ||
        group.crc_errors != last_crc_errors_ || group.length_errors != last_length_errors_))
      invalidate_partial();
    has_sequence_ = true;
    has_counters_ = true;
    last_sequence_ = group.sequence;
    last_dropped_ = group.dropped;
    last_crc_errors_ = group.crc_errors;
    last_length_errors_ = group.length_errors;

    std::optional<sensor_msgs::msg::LaserScan> completed;
    for (const auto& point : group.points) {
      if (point.angle_cdeg >= kAngleBins) {
        invalidate_partial();
        continue;
      }

      const bool wrap = has_angle_ && point.angle_cdeg < last_angle_ &&
          static_cast<std::uint16_t>(last_angle_ - point.angle_cdeg) > 18000;
      if (wrap) {
        if (saw_first_wrap_) completed = finish_scan(frame_id, range_min_m, range_max_m);
        saw_first_wrap_ = true;
        reset_scan();
        have_scan_start_ = true;
        scan_start_angle_ = point.angle_cdeg;
        scan_start_stamp_ns_ = mapped_group_stamp_ns;
      } else if (has_angle_ && point.angle_cdeg <= last_angle_) {
        invalidate_partial();
      }

      if (!have_scan_start_) {
        have_scan_start_ = true;
        scan_start_angle_ = point.angle_cdeg;
        scan_start_stamp_ns_ = mapped_group_stamp_ns;
      }
      last_angle_ = point.angle_cdeg;
      has_angle_ = true;
      ++sample_count_;

      // Raw N10 angles increase clockwise; ROS angles increase counterclockwise.
      const auto bin = (kAngleBins - point.angle_cdeg) % kAngleBins;
      auto& range = ranges_[bin];
      auto& intensity = intensities_[bin];
      if (seen_bins_[bin]) {
        invalidate_partial();
      } else {
        seen_bins_[bin] = true;
      }
      if (seen_bins_[bin] && std::isnan(range) &&
          point.distance_mm != 0U && point.distance_mm != 0xffffU) {
        const auto metres = static_cast<float>(point.distance_mm) * 0.001F;
        if (metres >= range_min_m && metres <= range_max_m) {
          range = metres;
          intensity = static_cast<float>(point.strength);
        }
      }
    }
    return completed;
  }

  std::uint64_t dropped_scans() const { return dropped_scans_; }

 private:
  void reset_scan() {
    ranges_.assign(kAngleBins, std::numeric_limits<float>::quiet_NaN());
    intensities_.assign(kAngleBins, 0.0F);
    seen_bins_.assign(kAngleBins, false);
    sample_count_ = 0;
    scan_bad_ = false;
    have_scan_start_ = false;
    scan_start_angle_ = 0;
    scan_start_stamp_ns_ = 0;
  }

  void invalidate_partial() { scan_bad_ = true; }

  std::optional<sensor_msgs::msg::LaserScan> finish_scan(
      const std::string& frame_id, double range_min_m, double range_max_m) {
    const auto coverage = last_angle_ >= scan_start_angle_
        ? static_cast<std::uint32_t>(last_angle_ - scan_start_angle_) : 0U;
    if (scan_bad_ || !have_scan_start_ || sample_count_ < kMinimumSamples ||
        coverage < kFullCoverageCdeg) {
      ++dropped_scans_;
      return std::nullopt;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = frame_id;
    scan.header.stamp.sec = static_cast<std::int32_t>(scan_start_stamp_ns_ / 1000000000LL);
    scan.header.stamp.nanosec = static_cast<std::uint32_t>(scan_start_stamp_ns_ % 1000000000LL);
    scan.angle_min = 0.0F;
    scan.angle_increment = static_cast<float>(std::acos(-1.0) / 18000.0);
    scan.angle_max = scan.angle_increment * static_cast<float>(kAngleBins - 1);
    scan.time_increment = 0.0F;  // N10只提供包时标，没有逐束时间。
    if (previous_scan_start_ns_ && scan_start_stamp_ns_ > *previous_scan_start_ns_)
      scan.scan_time = static_cast<float>(scan_start_stamp_ns_ - *previous_scan_start_ns_) / 1e9F;
    else
      scan.scan_time = 0.0F;
    scan.range_min = static_cast<float>(range_min_m);
    scan.range_max = static_cast<float>(range_max_m);
    scan.ranges = ranges_;
    scan.intensities = intensities_;
    previous_scan_start_ns_ = scan_start_stamp_ns_;
    return scan;
  }

  std::vector<float> ranges_;
  std::vector<float> intensities_;
  std::vector<bool> seen_bins_;
  std::optional<std::int64_t> previous_scan_start_ns_;
  std::uint32_t last_sequence_ = 0;
  std::uint32_t last_dropped_ = 0;
  std::uint32_t last_crc_errors_ = 0;
  std::uint32_t last_length_errors_ = 0;
  std::uint16_t last_angle_ = 0;
  std::uint16_t scan_start_angle_ = 0;
  std::int64_t scan_start_stamp_ns_ = 0;
  std::size_t sample_count_ = 0;
  std::uint64_t dropped_scans_ = 0;
  bool has_sequence_ = false;
  bool has_counters_ = false;
  bool has_angle_ = false;
  bool saw_first_wrap_ = false;
  bool have_scan_start_ = false;
  bool scan_bad_ = false;
};

class WheelOdometryIntegrator {
 public:
  void reset() {
    seeded_ = false;
    last_source_us_ = 0;
    x_m_ = 0;
    y_m_ = 0;
    yaw_rad_ = 0;
  }

  std::optional<nav_msgs::msg::Odometry> update(
      double vx_m_s, double vy_m_s, double wz_rad_s, std::uint64_t source_us,
      std::int64_t stamp_ns, bool valid, const std::string& odom_frame,
      const std::string& base_frame) {
    if (!valid || !std::isfinite(vx_m_s) || !std::isfinite(vy_m_s) ||
        !std::isfinite(wz_rad_s) || source_us == 0 || stamp_ns < 0 ||
        odom_frame.empty() || base_frame.empty()) {
      seeded_ = false;
      return std::nullopt;
    }
    if (!seeded_) {
      seeded_ = true;
      last_source_us_ = source_us;
    } else {
      if (source_us <= last_source_us_) return std::nullopt;
      if (source_us - last_source_us_ > 200000) {
        last_source_us_ = source_us;
      } else {
        const double dt = static_cast<double>(source_us - last_source_us_) * 1e-6;
        const double turn = wz_rad_s * dt;
        double dx_body = vx_m_s * dt;
        double dy_body = vy_m_s * dt;
        if (std::abs(wz_rad_s) > 1e-9) {
          dx_body = (vx_m_s * std::sin(turn) + vy_m_s * (std::cos(turn) - 1.0)) / wz_rad_s;
          dy_body = (vx_m_s * (1.0 - std::cos(turn)) + vy_m_s * std::sin(turn)) / wz_rad_s;
        }
        x_m_ += std::cos(yaw_rad_) * dx_body - std::sin(yaw_rad_) * dy_body;
        y_m_ += std::sin(yaw_rad_) * dx_body + std::cos(yaw_rad_) * dy_body;
        yaw_rad_ = std::atan2(std::sin(yaw_rad_ + turn), std::cos(yaw_rad_ + turn));
        last_source_us_ = source_us;
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;
    odom.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    odom.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    set_yaw(odom.pose.pose.orientation, yaw_rad_);
    odom.twist.twist.linear.x = vx_m_s;
    odom.twist.twist.linear.y = vy_m_s;
    odom.twist.twist.angular.z = wz_rad_s;
    // 轮速尺度、车体几何和编码器噪声仍需实车标定。
    set_unknown_covariance(odom.pose.covariance);
    set_unknown_covariance(odom.twist.covariance);
    return odom;
  }

 private:
  bool seeded_ = false;
  std::uint64_t last_source_us_ = 0;
  double x_m_ = 0;
  double y_m_ = 0;
  double yaw_rad_ = 0;
};

inline void set_yaw(geometry_msgs::msg::Quaternion& orientation, double yaw_rad) {
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(yaw_rad * 0.5);
  orientation.w = std::cos(yaw_rad * 0.5);
}

inline void set_unknown_covariance(std::array<double, 36>& covariance) {
  covariance.fill(0.0);
  covariance[0] = 1e6;
  covariance[7] = 1e6;
  covariance[14] = 1e6;
  covariance[21] = 1e6;
  covariance[28] = 1e6;
  covariance[35] = 1e6;
}

inline nav_msgs::msg::Odometry make_fusion_odometry(
    const std::array<double, 7>& state, const std::array<double, 7>& covariance_diag,
    std::int64_t stamp_ns, const std::string& odom_frame, const std::string& base_frame) {
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_frame;
  if (stamp_ns >= 0) {
    odom.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    odom.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  }
  odom.pose.pose.position.x = state[0];
  odom.pose.pose.position.y = state[1];
  set_yaw(odom.pose.pose.orientation, state[2]);
  odom.twist.twist.linear.x = state[3];
  odom.twist.twist.linear.y = state[4];
  odom.twist.twist.angular.z = state[5];
  set_unknown_covariance(odom.pose.covariance);
  set_unknown_covariance(odom.twist.covariance);
  const std::array<std::size_t, 3> pose_indices{0, 7, 35};
  const std::array<std::size_t, 3> twist_indices{0, 7, 35};
  for (std::size_t i = 0; i < pose_indices.size(); ++i) {
    if (std::isfinite(covariance_diag[i]) && covariance_diag[i] >= 0)
      odom.pose.covariance[pose_indices[i]] = covariance_diag[i];
  }
  for (std::size_t i = 0; i < twist_indices.size(); ++i) {
    if (std::isfinite(covariance_diag[i + 3]) && covariance_diag[i + 3] >= 0)
      odom.twist.covariance[twist_indices[i]] = covariance_diag[i + 3];
  }
  return odom;
}

inline geometry_msgs::msg::TransformStamped odom_transform(
    const nav_msgs::msg::Odometry& odom) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = odom.child_frame_id;
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = odom.pose.pose.orientation;
  return transform;
}

inline geometry_msgs::msg::TransformStamped n10_sensor_transform(
    const std::string& base_frame_id, const std::string& sensor_frame_id,
    double x_m, double y_m, double yaw_rad) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = base_frame_id;
  transform.child_frame_id = sensor_frame_id;
  transform.transform.translation.x = x_m;
  transform.transform.translation.y = y_m;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = std::sin(yaw_rad * 0.5);
  transform.transform.rotation.w = std::cos(yaw_rad * 0.5);
  return transform;
}

}  // 命名空间 ugv_bridge::ros_navigation
