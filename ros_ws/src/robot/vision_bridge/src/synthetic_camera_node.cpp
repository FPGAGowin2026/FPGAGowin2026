#include "vision_bridge/detector.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace vision_bridge {
namespace {

struct Pose2D {
  double x;
  double y;
  double yaw;
};

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class SyntheticCameraNode : public rclcpp::Node {
 public:
  SyntheticCameraNode()
  : Node("synthetic_camera_node")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/wheel/odom");
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
    target_x_ = declare_parameter<double>("target_x_m", 2.5);
    target_y_ = declare_parameter<double>("target_y_m", 0.8);
    field_of_view_ = declare_parameter<double>("field_of_view_rad", 1.0472);
    const auto width = declare_parameter<int64_t>("width", 160);
    const auto height = declare_parameter<int64_t>("height", 120);
    size_scale_ = declare_parameter<double>("size_scale", 18.0);
    const double rate = declare_parameter<double>("publish_rate_hz", 10.0);
    if (width <= 0 || height <= 0 || field_of_view_ <= 0.0 || rate <= 0.0) {
      throw std::invalid_argument("合成相机参数无效");
    }
    width_ = static_cast<std::size_t>(width);
    height_ = static_cast<std::size_t>(height);

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&SyntheticCameraNode::on_odom, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&SyntheticCameraNode::publish_image, this));
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & orientation = message->pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    pose_ = Pose2D{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      yaw,
    };
  }

  void publish_image()
  {
    if (!pose_) {
      return;
    }
    const double offset_x = target_x_ - pose_->x;
    const double offset_y = target_y_ - pose_->y;
    const double distance = std::hypot(offset_x, offset_y);
    const double bearing = normalize_angle(std::atan2(offset_y, offset_x) - pose_->yaw);

    sensor_msgs::msg::Image message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.height = static_cast<std::uint32_t>(height_);
    message.width = static_cast<std::uint32_t>(width_);
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(width_ * 3U);
    message.data = render_target(
      width_, height_, distance, bearing, field_of_view_, size_scale_);
    publisher_->publish(message);
  }

  std::string frame_id_;
  double target_x_{};
  double target_y_{};
  double field_of_view_{};
  double size_scale_{};
  std::size_t width_{};
  std::size_t height_{};
  std::optional<Pose2D> pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vision_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vision_bridge::SyntheticCameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("synthetic_camera_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
