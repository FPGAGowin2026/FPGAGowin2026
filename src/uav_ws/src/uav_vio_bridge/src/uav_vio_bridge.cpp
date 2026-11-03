#include "uav_vio_bridge/uav_vio_bridge.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <Eigen/Eigenvalues>

namespace uav_vio_bridge
{
namespace
{
using Matrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}
bool valid_covariance(const Matrix6 & covariance)
{
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-7) ||
    covariance.diagonal().minCoeff() < 0.0 || covariance.trace() <= 0.0)
  {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(covariance, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= -1e-9 * std::max(1.0, covariance.norm());
}
double seconds(int64_t ns) {return static_cast<double>(ns) * 1e-9;}
}  // namespace

OdometryAdapter::OdometryAdapter(const BridgeConfig & config)
: config_(config)
{
  for (const auto value : {config.publish_rate_hz, config.max_sample_age_s,
      config.visual_timeout_s, config.input_timeout_s, config.max_position_step_m,
      config.max_attitude_step_rad})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Bridge rates, timeouts and step limits must be positive");
    }
  }
  if (!std::isfinite(config.world_yaw_rad) || !std::isfinite(config.max_future_s) ||
    config.max_future_s < 0.0 || !config.position_body_in_imu.allFinite() ||
    !config.rotation_body_imu.coeffs().allFinite() ||
    std::abs(config.rotation_body_imu.norm() - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Invalid IMU/body transform or timestamp tolerance");
  }
  for (const auto & frame : {config.input_frame, config.input_child_frame,
      config.output_frame, config.output_child_frame})
  {
    if (frame.empty() || frame.front() == '/') {
      throw std::invalid_argument("Frame ids must be nonempty and relative");
    }
  }
  if (config.output_frame == config.output_child_frame) {
    throw std::invalid_argument("Output parent and child frames must differ");
  }
  config_.rotation_body_imu.normalize();
}

std::string OdometryAdapter::latch(const std::string & reason)
{
  fault_ = reason + "; restart bridge after checking VIO and timebase";
  return fault_;
}
std::string OdometryAdapter::check_stamp(int64_t stamp_ns, int64_t ros_now_ns) const
{
  if (stamp_ns <= 0) {return "missing sample timestamp";}
  const double age = seconds(ros_now_ns - stamp_ns);
  if (age > config_.max_sample_age_s) {return "stale sample timestamp";}
  if (age < -config_.max_future_s) {return "future sample timestamp";}
  return "";
}
std::string OdometryAdapter::observe_visual(
  int64_t stamp_ns, int64_t ros_now_ns, int64_t steady_now_ns)
{
  if (faulted()) {return fault_;}
  if (have_output_) {
    const auto timeout_error = check_timeout(ros_now_ns, steady_now_ns);
    if (!timeout_error.empty()) {return timeout_error;}
  }
  const auto error = check_stamp(stamp_ns, ros_now_ns);
  if (!error.empty()) {return error;}
  if (visual_stamp_ns_ != 0 && stamp_ns < visual_stamp_ns_) {
    return latch("visual timestamp moved backwards");
  }
  // Repeated health reports must not renew the observation time.
  if (stamp_ns == visual_stamp_ns_) {return "duplicate visual timestamp";}
  visual_stamp_ns_ = stamp_ns;
  visual_receive_ns_ = steady_now_ns;
  return "";
}
std::string OdometryAdapter::observe_health(
  const diagnostic_msgs::msg::DiagnosticArray & health,
  int64_t ros_now_ns, int64_t steady_now_ns)
{
  if (faulted()) {return fault_;}
  if (health.header.frame_id != config_.input_frame ||
    health.header.stamp.nanosec >= 1000000000u || health.status.size() != 1)
  {
    return "invalid visual health frame, timestamp or status count";
  }
  const auto & status = health.status.front();
  std::string version, initialized, counter;
  for (const auto & value : status.values) {
    std::string * field = nullptr;
    if (value.key == "contract_version") {field = &version;}
    if (value.key == "initialized") {field = &initialized;}
    if (value.key == "reset_counter") {field = &counter;}
    if (field) {
      if (value.value.empty()) {return "empty visual health field";}
      if (!field->empty()) {return "duplicate visual health field";}
      *field = value.value;
    }
  }
  uint64_t generation = 0;
  const auto parsed = std::from_chars(counter.data(), counter.data() + counter.size(), generation);
  if (status.name != "openvins/visual_fusion" || status.hardware_id.empty() || version != "1" ||
    (initialized != "true" && initialized != "false") || counter.empty() ||
    parsed.ec != std::errc{} || parsed.ptr != counter.data() + counter.size())
  {
    return "invalid visual health contract";
  }
  const bool changed = !visual_session_id_.empty() &&
    (status.hardware_id != visual_session_id_ || generation != reset_counter_);
  if (changed && have_output_) {return latch("VIO session or initialization changed");}
  if (changed) {visual_stamp_ns_ = 0; visual_receive_ns_ = 0;}
  visual_session_id_ = status.hardware_id;
  reset_counter_ = generation;
  if (initialized != "true" || generation == 0) {
    return have_output_ ? latch("VIO no longer initialized") : "waiting for VIO initialization";
  }
  if (status.level != status.OK) {
    visual_stamp_ns_ = 0;
    return have_output_ ? latch("VIO reports unhealthy fusion") :
           "waiting for accepted visual fusion";
  }
  const int64_t stamp = static_cast<int64_t>(health.header.stamp.sec) * 1000000000LL +
    health.header.stamp.nanosec;
  return observe_visual(stamp, ros_now_ns, steady_now_ns);
}
std::string OdometryAdapter::check_timeout(int64_t ros_now_ns, int64_t steady_now_ns)
{
  if (faulted()) {return fault_;}
  if (ros_now_ns_ != 0 && ros_now_ns < ros_now_ns_) {
    return latch("ROS clock moved backwards");
  }
  ros_now_ns_ = ros_now_ns;
  const bool visual_stale = visual_stamp_ns_ == 0 ||
    seconds(steady_now_ns - visual_receive_ns_) > config_.visual_timeout_s ||
    seconds(ros_now_ns - visual_stamp_ns_) > config_.visual_timeout_s ||
    seconds(visual_stamp_ns_ - ros_now_ns) > config_.max_future_s;
  if (visual_stale) {
    return have_output_ ? latch("visual update expired") : "waiting for fresh visual update";
  }
  if (have_output_ && seconds(steady_now_ns - input_receive_ns_) > config_.input_timeout_s) {
    return latch("odometry stream expired");
  }
  return "";
}
std::string OdometryAdapter::process(
  const nav_msgs::msg::Odometry & input, int64_t ros_now_ns, int64_t steady_now_ns,
  nav_msgs::msg::Odometry & output)
{
  const auto timeout_error = check_timeout(ros_now_ns, steady_now_ns);
  if (!timeout_error.empty()) {return timeout_error;}
  if (input.header.frame_id != config_.input_frame ||
    input.child_frame_id != config_.input_child_frame)
  {
    return "unexpected input frames";
  }
  if (input.header.stamp.nanosec >= 1000000000u) {return "invalid timestamp nanoseconds";}
  const int64_t stamp = static_cast<int64_t>(input.header.stamp.sec) * 1000000000LL +
    input.header.stamp.nanosec;
  const auto stamp_error = check_stamp(stamp, ros_now_ns);
  if (!stamp_error.empty()) {return stamp_error;}
  if (have_input_ && stamp < input_stamp_ns_) {return latch("odometry timestamp moved backwards");}
  if (have_input_ && stamp == input_stamp_ns_) {return "duplicate odometry timestamp";}

  const auto & p = input.pose.pose.position;
  const auto & q = input.pose.pose.orientation;
  const auto & v = input.twist.twist.linear;
  const auto & w = input.twist.twist.angular;
  const Eigen::Vector3d position(p.x, p.y, p.z), velocity(v.x, v.y, v.z), omega(w.x, w.y, w.z);
  Eigen::Quaterniond attitude(q.w, q.x, q.y, q.z);
  const Matrix6 pose_cov = Eigen::Map<const Matrix6>(input.pose.covariance.data());
  const Matrix6 twist_cov = Eigen::Map<const Matrix6>(input.twist.covariance.data());
  if (!position.allFinite() || !velocity.allFinite() || !omega.allFinite() ||
    !attitude.coeffs().allFinite() || std::abs(attitude.norm() - 1.0) > 1e-3 ||
    !valid_covariance(pose_cov) || !valid_covariance(twist_cov))
  {
    return "invalid pose, twist, quaternion or covariance";
  }
  attitude.normalize();
  if (have_input_ && ((position - previous_position_).norm() > config_.max_position_step_m ||
    attitude.angularDistance(previous_attitude_) > config_.max_attitude_step_rad))
  {
    return latch("VIO pose discontinuity");
  }
  input_stamp_ns_ = stamp;
  input_receive_ns_ = steady_now_ns;
  previous_position_ = position;
  previous_attitude_ = attitude;
  have_input_ = true;
  if (have_output_ && seconds(steady_now_ns - publish_ns_) < 1.0 / config_.publish_rate_hz) {
    return "rate limited";
  }

  const Eigen::Matrix3d r_wg = Eigen::AngleAxisd(
    config_.world_yaw_rad,
    Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d r_gi = attitude.toRotationMatrix();
  const Eigen::Matrix3d r_bi = config_.rotation_body_imu.toRotationMatrix();
  const Eigen::Vector3d & lever = config_.position_body_in_imu;
  const Eigen::Vector3d p_out = r_wg * (position + r_gi * lever);
  const Eigen::Quaterniond q_out(r_wg * r_gi * r_bi.transpose());
  const Eigen::Vector3d v_out = r_bi * (velocity + omega.cross(lever));
  const Eigen::Vector3d w_out = r_bi * omega;

  // OpenVINS errors are [p_G, theta_I]; ROS pose covariance uses world axes.
  Matrix6 j_pose = Matrix6::Zero();
  j_pose.block<3, 3>(0, 0) = r_wg;
  j_pose.block<3, 3>(0, 3) = -r_wg * r_gi * skew(lever);
  j_pose.block<3, 3>(3, 3) = r_wg * r_gi;
  Matrix6 transformed_pose = j_pose * pose_cov * j_pose.transpose();
  // shortcut: keep an isotropic attitude bound until the deployed MAVROS contract
  // is verified. Inflate both blocks before dropping their cross covariance.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> attitude_solver(
    transformed_pose.block<3, 3>(3, 3), Eigen::EigenvaluesOnly);
  if (attitude_solver.info() != Eigen::Success) {
    return latch("attitude covariance decomposition failed");
  }
  const double cross_bound = transformed_pose.block<3, 3>(0, 3).norm();
  transformed_pose.block<3, 3>(0, 0).diagonal().array() += cross_bound;
  const double attitude_variance = attitude_solver.eigenvalues().maxCoeff() + cross_bound;
  transformed_pose.block<3, 3>(3, 3) = attitude_variance * Eigen::Matrix3d::Identity();
  transformed_pose.block<3, 3>(0, 3).setZero();
  transformed_pose.block<3, 3>(3, 0).setZero();
  Matrix6 j_twist = Matrix6::Zero();
  j_twist.block<3, 3>(0, 0) = r_bi;
  j_twist.block<3, 3>(0, 3) = -r_bi * skew(lever);
  j_twist.block<3, 3>(3, 3) = r_bi;
  const Matrix6 transformed_twist = j_twist * twist_cov * j_twist.transpose();
  if (!transformed_pose.allFinite() ||
    !transformed_twist.allFinite() || !p_out.allFinite() || !v_out.allFinite())
  {
    return latch("nonfinite transformed odometry");
  }

  output = input;
  output.header.frame_id = config_.output_frame;
  output.child_frame_id = config_.output_child_frame;
  output.pose.pose.position.x = p_out.x();
  output.pose.pose.position.y = p_out.y();
  output.pose.pose.position.z = p_out.z();
  output.pose.pose.orientation.x = q_out.x();
  output.pose.pose.orientation.y = q_out.y();
  output.pose.pose.orientation.z = q_out.z();
  output.pose.pose.orientation.w = q_out.w();
  output.twist.twist.linear.x = v_out.x();
  output.twist.twist.linear.y = v_out.y();
  output.twist.twist.linear.z = v_out.z();
  output.twist.twist.angular.x = w_out.x();
  output.twist.twist.angular.y = w_out.y();
  output.twist.twist.angular.z = w_out.z();
  Eigen::Map<Matrix6>(output.pose.covariance.data()) = transformed_pose;
  Eigen::Map<Matrix6>(output.twist.covariance.data()) = transformed_twist;
  have_output_ = true;
  publish_ns_ = steady_now_ns;
  return "";
}
}  // namespace uav_vio_bridge
