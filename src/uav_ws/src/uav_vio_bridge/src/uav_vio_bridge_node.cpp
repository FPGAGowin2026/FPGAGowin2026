#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "uav_vio_bridge/uav_vio_bridge.hpp"

class UavVioBridge : public rclcpp::Node
{
public:
  UavVioBridge()
  : Node("uav_vio_bridge")
  {
    enabled_ = parameter<bool>("enabled", false);
    const auto input_topic = parameter<std::string>("input_topic", "/ov_msckf/odomimu");
    const auto visual_topic = parameter<std::string>(
      "visual_health_topic",
      "/ov_msckf/visual_health");
    const auto output_topic = parameter<std::string>("output_topic", "/mavros/odometry/out");
    if (input_topic.empty() || visual_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("Bridge topics must not be empty");
    }
    uav_vio_bridge::BridgeConfig config;
    config.input_frame = parameter<std::string>("input_frame_id", config.input_frame);
    config.input_child_frame = parameter<std::string>(
      "input_child_frame_id",
      config.input_child_frame);
    config.output_frame = parameter<std::string>("output_frame_id", config.output_frame);
    config.output_child_frame = parameter<std::string>(
      "output_child_frame_id",
      config.output_child_frame);
    for (const auto & entry : std::vector<std::pair<std::string, double *>>{
      {"world_yaw_rad", &config.world_yaw_rad}, {"publish_rate_hz", &config.publish_rate_hz},
      {"max_sample_age_s", &config.max_sample_age_s}, {"max_future_s", &config.max_future_s},
      {"visual_timeout_s", &config.visual_timeout_s}, {"input_timeout_s", &config.input_timeout_s},
      {"max_position_step_m", &config.max_position_step_m},
      {"max_attitude_step_rad", &config.max_attitude_step_rad}})
    {
      *entry.second = parameter<double>(entry.first, *entry.second);
    }
    const auto rotation = parameter<std::vector<double>>(
      "rotation_body_imu_xyzw", {0.0, 0.0, 0.0, 1.0});
    const auto translation = parameter<std::vector<double>>(
      "position_body_in_imu_m", {0.0, 0.0, 0.0});
    if (rotation.size() != 4 || translation.size() != 3) {
      throw std::invalid_argument(
              "IMU/body transform requires quaternion xyzw[4] and translation[3]");
    }
    config.rotation_body_imu =
      Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2]);
    config.position_body_in_imu = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    adapter_ = std::make_unique<uav_vio_bridge::OdometryAdapter>(config);
    publisher_ = create_publisher<Odom>(output_topic, rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<Odom>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](Odom::ConstSharedPtr msg) {
        ++received_;
        Odom output;
        const auto error = adapter_->process(*msg, now().nanoseconds(), steady_now(), output);
        if (!error.empty()) {
          if (error == "rate limited") {++rate_limited_;} else {++rejected_; report(error);}
          return;
        }
        ++adapted_;
        if (enabled_) {publisher_->publish(output); ++published_;}
      });
    visual_subscription_ = create_subscription<Health>(
      visual_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](Health::ConstSharedPtr msg) {
        const auto error = adapter_->observe_health(*msg, now().nanoseconds(), steady_now());
        if (!error.empty()) {report(error);}
      });

    // Only axis-convention TFs, never a fabricated physical mounting transform.
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    const double pi = std::acos(-1.0);
    const Eigen::Quaterniond world_rotation(
      Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond body_rotation(Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()));
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & entry : std::vector<std::pair<std::string, Eigen::Quaterniond>>{
      {config.output_frame, world_rotation}, {config.output_child_frame, body_rotation}})
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now();
      tf.header.frame_id = entry.first;
      tf.child_frame_id = entry.first + (entry.first == config.output_frame ? "_ned" : "_frd");
      tf.transform.rotation.x = entry.second.x();
      tf.transform.rotation.y = entry.second.y();
      tf.transform.rotation.z = entry.second.z();
      tf.transform.rotation.w = entry.second.w();
      transforms.push_back(tf);
    }
    static_tf_->sendTransform(transforms);
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {
        const auto error = adapter_->check_timeout(now().nanoseconds(), steady_now());
        if (!error.empty()) {report(error);}
        RCLCPP_INFO_THROTTLE(
          get_logger(), steady_clock_, 5000,
          "enabled=%d faulted=%d received=%lu adapted=%lu published=%lu rejected=%lu rate_limited=%lu",
          enabled_, adapter_->faulted(), received_, adapted_, published_, rejected_, rate_limited_);
      });
    RCLCPP_INFO(
      get_logger(), "Odometry: %s -> %s; visual updates: %s; enabled=%d",
      subscription_->get_topic_name(), publisher_->get_topic_name(),
      visual_subscription_->get_topic_name(), enabled_);
    if (!enabled_) {
      RCLCPP_WARN(
        get_logger(), "Output disabled; verify mounting, timebase and MAVROS before enabling");
    }
  }

private:
  using Odom = nav_msgs::msg::Odometry;
  using Health = diagnostic_msgs::msg::DiagnosticArray;
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  int64_t steady_now() {return steady_clock_.now().nanoseconds();}
  void report(const std::string & reason)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "VIO withheld: %s", reason.c_str());
  }
  bool enabled_ = false;
  uint64_t received_ = 0, adapted_ = 0, published_ = 0, rejected_ = 0, rate_limited_ = 0;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::unique_ptr<uav_vio_bridge::OdometryAdapter> adapter_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<Odom>::SharedPtr publisher_;
  rclcpp::Subscription<Odom>::SharedPtr subscription_;
  rclcpp::Subscription<Health>::SharedPtr visual_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<UavVioBridge>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("uav_vio_bridge"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
