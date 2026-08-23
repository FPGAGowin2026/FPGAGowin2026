#include "robot_navigation/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot_navigation {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Pose2D {
  double x;
  double y;
  double yaw;
};

}  // namespace

class SimLaserNode : public rclcpp::Node {
 public:
  SimLaserNode()
  : Node("sim_laser_node")
  {
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/wheel/odom");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_link");
    const double rate = declare_parameter<double>("publish_rate_hz", 10.0);
    const auto samples = declare_parameter<int64_t>("samples", 181);
    range_min_ = declare_parameter<double>("range_min_m", 0.05);
    range_max_ = declare_parameter<double>("range_max_m", 8.0);
    room_ = {
      declare_parameter<double>("room_x_min_m", -3.0),
      declare_parameter<double>("room_x_max_m", 3.0),
      declare_parameter<double>("room_y_min_m", -2.0),
      declare_parameter<double>("room_y_max_m", 2.0),
    };
    obstacle_ = CircularObstacle{
      declare_parameter<double>("obstacle_x_m", 1.0),
      declare_parameter<double>("obstacle_y_m", -0.6),
      declare_parameter<double>("obstacle_radius_m", 0.25),
    };

    if (samples < 2 || rate <= 0.0 || !(0.0 < range_min_ && range_min_ < range_max_)) {
      throw std::invalid_argument("激光样本数、频率或量程参数无效");
    }
    if (!(room_[0] < room_[1] && room_[2] < room_[3])) {
      throw std::invalid_argument("房间边界参数无效");
    }
    samples_ = static_cast<std::size_t>(samples);
    scan_time_ = 1.0 / rate;

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&SimLaserNode::on_odom, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(scan_time_)),
      std::bind(&SimLaserNode::publish_scan, this));
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

  void publish_scan()
  {
    if (!pose_) {
      return;
    }

    const double angle_min = -kPi;
    const double angle_max = kPi;
    const double angle_increment = (angle_max - angle_min) /
      static_cast<double>(samples_ - 1U);
    std::vector<float> ranges;
    ranges.reserve(samples_);
    for (std::size_t index = 0; index < samples_; ++index) {
      const double distance = ray_distance(
        pose_->x, pose_->y,
        pose_->yaw + angle_min + static_cast<double>(index) * angle_increment,
        room_, obstacle_, range_max_);
      ranges.push_back(static_cast<float>(std::clamp(distance, range_min_, range_max_)));
    }

    sensor_msgs::msg::LaserScan message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.angle_min = static_cast<float>(angle_min);
    message.angle_max = static_cast<float>(angle_max);
    message.angle_increment = static_cast<float>(angle_increment);
    message.scan_time = static_cast<float>(scan_time_);
    message.time_increment = static_cast<float>(scan_time_ / static_cast<double>(samples_));
    message.range_min = static_cast<float>(range_min_);
    message.range_max = static_cast<float>(range_max_);
    message.ranges = std::move(ranges);
    publisher_->publish(message);
  }

  std::string frame_id_;
  std::size_t samples_{};
  double range_min_{};
  double range_max_{};
  double scan_time_{};
  Room room_{};
  std::optional<CircularObstacle> obstacle_;
  std::optional<Pose2D> pose_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_navigation::SimLaserNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_laser_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
