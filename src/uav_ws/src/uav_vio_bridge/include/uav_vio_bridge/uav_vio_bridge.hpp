#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace uav_vio_bridge
{
struct BridgeConfig
{
  std::string input_frame = "global", input_child_frame = "imu";
  std::string output_frame = "vio_odom", output_child_frame = "vio_body";
  // R_BI rotates IMU vectors to body FLU; r_IB is I->B expressed in I.
  Eigen::Quaterniond rotation_body_imu = Eigen::Quaterniond::Identity();
  Eigen::Vector3d position_body_in_imu = Eigen::Vector3d::Zero();
  double world_yaw_rad = 0.0;
  double publish_rate_hz = 40.0;
  double max_sample_age_s = 0.15, max_future_s = 0.01;
  double visual_timeout_s = 0.3, input_timeout_s = 0.2;
  double max_position_step_m = 1.0, max_attitude_step_rad = 0.75;
};

// Geometry and stream checks have no ROS graph or flight-command side effects.
class OdometryAdapter
{
public:
  explicit OdometryAdapter(const BridgeConfig & config);
  std::string observe_health(
    const diagnostic_msgs::msg::DiagnosticArray & health,
    int64_t ros_now_ns, int64_t steady_now_ns);
  std::string process(
    const nav_msgs::msg::Odometry & input, int64_t ros_now_ns, int64_t steady_now_ns,
    nav_msgs::msg::Odometry & output);
  std::string check_timeout(int64_t ros_now_ns, int64_t steady_now_ns);
  bool faulted() const {return !fault_.empty();}

private:
  std::string observe_visual(int64_t stamp_ns, int64_t ros_now_ns, int64_t steady_now_ns);
  std::string check_stamp(int64_t stamp_ns, int64_t ros_now_ns) const;
  std::string latch(const std::string & reason);
  BridgeConfig config_;
  std::string fault_;
  std::string visual_session_id_;
  uint64_t reset_counter_ = 0;
  int64_t visual_stamp_ns_ = 0, visual_receive_ns_ = 0;
  int64_t input_stamp_ns_ = 0, input_receive_ns_ = 0, publish_ns_ = 0, ros_now_ns_ = 0;
  bool have_input_ = false, have_output_ = false;
  Eigen::Vector3d previous_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond previous_attitude_ = Eigen::Quaterniond::Identity();
};
}  // namespace uav_vio_bridge
