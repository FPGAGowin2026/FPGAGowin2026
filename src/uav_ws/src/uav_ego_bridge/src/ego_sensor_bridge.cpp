#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using SteadyClock = std::chrono::steady_clock;

bool finite_vector(const tf2::Vector3 & v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool normalize_quaternion(tf2::Quaternion & q)
{
  const double n = q.length2();
  if (!std::isfinite(n) || n < 1e-12) return false;
  q.normalize();
  return true;
}

bool transform_from_message(const geometry_msgs::msg::Transform & input, tf2::Transform & output)
{
  tf2::Quaternion q;
  tf2::fromMsg(input.rotation, q);
  const tf2::Vector3 p(input.translation.x, input.translation.y, input.translation.z);
  if (!finite_vector(p) || !normalize_quaternion(q)) return false;
  output = tf2::Transform(q, p);
  return true;
}

// Rotate both 3x3 components, including cross-covariance, into the output frame.
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & covariance, const tf2::Quaternion & rotation)
{
  const tf2::Matrix3x3 matrix(rotation);
  std::array<double, 36> output{};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      for (int a = 0; a < 6; ++a) {
        if (row / 3 != a / 3) continue;
        for (int b = 0; b < 6; ++b) {
          if (col / 3 != b / 3) continue;
          output[row * 6 + col] += matrix[row % 3][a % 3] * covariance[a * 6 + b] *
            matrix[col % 3][b % 3];
        }
      }
    }
  }
  return output;
}

struct PoseSample
{
  int64_t stamp_ns;
  tf2::Transform planning_from_body;
};

class EgoSensorBridge final : public rclcpp::Node
{
public:
  EgoSensorBridge() : Node("ego_sensor_bridge")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "");
    planning_frame_ = declare_parameter<std::string>("planning_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "");
    max_source_age_s_ = declare_parameter<double>("max_source_age_s", 0.5);
    max_receive_age_s_ = declare_parameter<double>("max_receive_age_s", 0.5);
    max_pose_gap_s_ = declare_parameter<double>("max_pose_gap_s", 0.2);
    future_tolerance_s_ = declare_parameter<double>("future_tolerance_s", 0.05);
    if (odom_topic.empty() || depth_topic.empty() || planning_frame_.empty() ||
        body_frame_.empty() || camera_frame_.empty() || camera_frame_ == body_frame_ ||
        camera_frame_ == planning_frame_ ||
        planning_frame_ == body_frame_ ||
        !std::isfinite(max_source_age_s_) || max_source_age_s_ <= 0 ||
        !std::isfinite(max_receive_age_s_) || max_receive_age_s_ <= 0 ||
        !std::isfinite(max_pose_gap_s_) || max_pose_gap_s_ <= 0 ||
        !std::isfinite(future_tolerance_s_) || future_tolerance_s_ < 0) {
      throw std::invalid_argument("Invalid EGO sensor topics, frames, or timing limits");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/uav_ego/odom", reliable);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/uav_ego/depth", reliable);
    camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/uav_ego/camera_pose", reliable);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/uav_ego/sensor_ready", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_depth(std::move(msg)); });
    ready_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_ready(); });
  }

private:
  bool stamp_ok(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
        (stamp.sec == 0 && stamp.nanosec == 0)) return false;
    return stamp_ok_ns(rclcpp::Time(stamp).nanoseconds());
  }

  bool stamp_ok_ns(int64_t stamp_ns) const
  {
    if (stamp_ns <= 0) return false;
    const rclcpp::Time source(stamp_ns, get_clock()->get_clock_type());
    const double age = (now() - source).seconds();
    return std::isfinite(age) && age >= -future_tolerance_s_ && age <= max_source_age_s_;
  }

  bool lookup_transform(const std::string & target, const std::string & source,
                        const builtin_interfaces::msg::Time & stamp, tf2::Transform & transform)
  {
    if (target == source) {
      transform.setIdentity();
      return true;
    }
    try {
      const auto result = tf_buffer_->lookupTransform(
        target, source, rclcpp::Time(stamp, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(0.0));
      return transform_from_message(result.transform, transform);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (!stamp_ok(msg.header.stamp) || msg.header.frame_id.empty() ||
        msg.child_frame_id != body_frame_) return;
    const int64_t stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (!history_.empty() && stamp_ns <= history_.back().stamp_ns) return;

    tf2::Quaternion source_from_body_rotation;
    tf2::fromMsg(msg.pose.pose.orientation, source_from_body_rotation);
    const tf2::Vector3 source_from_body_position(
      msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    const tf2::Vector3 body_velocity(
      msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    const tf2::Vector3 body_angular(
      msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
    if (!normalize_quaternion(source_from_body_rotation) || !finite_vector(source_from_body_position) ||
        !finite_vector(body_velocity) || !finite_vector(body_angular)) return;

    tf2::Transform planning_from_source;
    if (!lookup_transform(planning_frame_, msg.header.frame_id, msg.header.stamp, planning_from_source)) {
      odom_transform_ok_ = false;
      return;
    }
    const tf2::Transform planning_from_body = planning_from_source *
      tf2::Transform(source_from_body_rotation, source_from_body_position);
    const auto world_linear = planning_from_body.getBasis() * body_velocity;
    const auto world_angular = planning_from_body.getBasis() * body_angular;

    auto output = msg;
    output.header.frame_id = planning_frame_;
    output.pose.pose.position.x = planning_from_body.getOrigin().x();
    output.pose.pose.position.y = planning_from_body.getOrigin().y();
    output.pose.pose.position.z = planning_from_body.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(planning_from_body.getRotation());
    output.twist.twist.linear.x = world_linear.x();
    output.twist.twist.linear.y = world_linear.y();
    output.twist.twist.linear.z = world_linear.z();
    output.twist.twist.angular.x = world_angular.x();
    output.twist.twist.angular.y = world_angular.y();
    output.twist.twist.angular.z = world_angular.z();
    output.pose.covariance = rotate_covariance(msg.pose.covariance, planning_from_source.getRotation());
    output.twist.covariance = rotate_covariance(msg.twist.covariance, planning_from_body.getRotation());
    odom_pub_->publish(output);

    history_.push_back({stamp_ns, planning_from_body});
    while (history_.size() > 200) history_.pop_front();
    last_odom_received_ = SteadyClock::now();
    odom_transform_ok_ = true;
    try_pending_depth();
  }

  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const bool encoding_ok = msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    const uint64_t bytes_per_pixel = msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ? 2 : 4;
    if (!stamp_ok(msg->header.stamp) || msg->header.frame_id != camera_frame_ || !encoding_ok ||
        msg->width == 0 || msg->height == 0 || msg->step < bytes_per_pixel * msg->width ||
        msg->data.size() < static_cast<uint64_t>(msg->step) * msg->height) {
      depth_transform_ok_ = false;
      return;
    }
    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp_ns <= last_depth_stamp_ns_ ||
        (pending_depth_ && stamp_ns <= rclcpp::Time(pending_depth_->header.stamp).nanoseconds())) return;
    pending_depth_ = std::move(msg);  // 只保留最新未完成的图像，避免无限积压。
    try_pending_depth();
  }

  void try_pending_depth()
  {
    if (!pending_depth_ || history_.empty()) return;
    if (!stamp_ok(pending_depth_->header.stamp)) {
      pending_depth_.reset();
      depth_transform_ok_ = false;
      return;
    }
    const int64_t stamp_ns = rclcpp::Time(pending_depth_->header.stamp).nanoseconds();
    if (stamp_ns < history_.front().stamp_ns) {
      pending_depth_.reset();
      depth_transform_ok_ = false;
      return;
    }
    const auto upper = std::lower_bound(history_.begin(), history_.end(), stamp_ns,
      [](const PoseSample & sample, int64_t stamp) { return sample.stamp_ns < stamp; });
    if (upper == history_.end()) return;  // 等待图像之后的一帧里程计，不外推。

    tf2::Transform planning_from_body;
    if (upper->stamp_ns == stamp_ns) {
      planning_from_body = upper->planning_from_body;
    } else {
      if (upper == history_.begin()) return;
      const auto lower = std::prev(upper);
      const int64_t span_ns = upper->stamp_ns - lower->stamp_ns;
      if (span_ns <= 0 || static_cast<double>(span_ns) * 1e-9 > max_pose_gap_s_) {
        pending_depth_.reset();
        depth_transform_ok_ = false;
        return;
      }
      const double ratio = static_cast<double>(stamp_ns - lower->stamp_ns) / span_ns;
      const auto position = lower->planning_from_body.getOrigin().lerp(
        upper->planning_from_body.getOrigin(), ratio);
      const auto rotation = lower->planning_from_body.getRotation().slerp(
        upper->planning_from_body.getRotation(), ratio).normalized();
      planning_from_body = tf2::Transform(rotation, position);
    }

    tf2::Transform body_from_camera;
    if (!lookup_transform(body_frame_, camera_frame_, pending_depth_->header.stamp, body_from_camera)) {
      depth_transform_ok_ = false;
      return;
    }
    const auto planning_from_camera = planning_from_body * body_from_camera;
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header.stamp = pending_depth_->header.stamp;
    camera_pose.header.frame_id = planning_frame_;
    camera_pose.pose.position.x = planning_from_camera.getOrigin().x();
    camera_pose.pose.position.y = planning_from_camera.getOrigin().y();
    camera_pose.pose.position.z = planning_from_camera.getOrigin().z();
    camera_pose.pose.orientation = tf2::toMsg(planning_from_camera.getRotation());
    camera_pose_pub_->publish(camera_pose);
    depth_pub_->publish(*pending_depth_);
    last_depth_stamp_ns_ = stamp_ns;
    last_depth_received_ = SteadyClock::now();
    depth_transform_ok_ = true;
    pending_depth_.reset();
  }

  void publish_ready()
  {
    const auto current = SteadyClock::now();
    const bool odom_recent = !history_.empty() && odom_transform_ok_ &&
      std::chrono::duration<double>(current - last_odom_received_).count() <= max_receive_age_s_ &&
      stamp_ok_ns(history_.back().stamp_ns);
    const bool depth_recent = last_depth_stamp_ns_ > 0 && depth_transform_ok_ &&
      std::chrono::duration<double>(current - last_depth_received_).count() <= max_receive_age_s_ &&
      stamp_ok_ns(last_depth_stamp_ns_);
    std_msgs::msg::Bool ready;
    ready.data = odom_recent && depth_recent;
    ready_pub_->publish(ready);
  }

  std::string planning_frame_, body_frame_, camera_frame_;
  double max_source_age_s_, max_receive_age_s_, max_pose_gap_s_, future_tolerance_s_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PoseSample> history_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_depth_;
  int64_t last_depth_stamp_ns_{0};
  SteadyClock::time_point last_odom_received_{}, last_depth_received_{};
  bool odom_transform_ok_{false}, depth_transform_ok_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EgoSensorBridge>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("ego_sensor_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
